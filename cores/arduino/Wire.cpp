/*
  TwoWire.cpp - TWI/I2C library for Wiring & Arduino
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

extern "C" {
  #include <stdlib.h>
  #include <string.h>
  #include <inttypes.h>
  #include <chip.h>
}

#include <Wire.h>

#define USEDI2CDEV_M 1

#if (USEDI2CDEV_M == 0)
  #define I2CDEV_M I2C0
#elif (USEDI2CDEV_M == 1)
  #define I2CDEV_M I2C1
#elif (USEDI2CDEV_M == 2)
  #define I2CDEV_M I2C2
#else
  #error "Master I2C device not defined!"
#endif

// Initialize Class Variables //////////////////////////////////////////////////

uint8_t TwoWire::rxBuffer[BUFFER_LENGTH];
uint8_t TwoWire::rxBufferIndex = 0;
uint8_t TwoWire::rxBufferLength = 0;

uint8_t TwoWire::txAddress = 0;
uint8_t TwoWire::txBuffer[BUFFER_LENGTH];
uint8_t TwoWire::txBufferIndex = 0;
uint8_t TwoWire::txBufferLength = 0;

uint8_t TwoWire::transmitting = 0;

// Constructors ////////////////////////////////////////////////////////////////

TwoWire::TwoWire() {
}

// Public Methods //////////////////////////////////////////////////////////////
namespace {
  // Constexpr wrappers to improve readability
  constexpr uint8_t PORT(uint8_t portNum) { return portNum; }
  constexpr uint8_t PIN(uint8_t pinNum) { return pinNum; }
  constexpr uint32_t MODE(uint32_t mode) { return mode; }  
}

void TwoWire::begin(void) {
  rxBufferIndex = 0;
  rxBufferLength = 0;

  txBufferIndex = 0;
  txBufferLength = 0;

  // Init I2C pin connect
  #if USEDI2CDEV_M == 0
    Chip_IOCON_PinMux(LPC_IOCON, PORT(0), PIN(27), MODE(0), FUNC1); // SDA0 / D57  AUX-1
    Chip_IOCON_PinMux(LPC_IOCON, PORT(0), PIN(28), MODE(0), FUNC1); // SCL0 / D58  AUX-1
  #elif USEDI2CDEV_M == 1
    Chip_IOCON_PinMux(LPC_IOCON, PORT(0), PIN(0), MODE(0), FUNC3); // SDA1 / D20 SCA
    Chip_IOCON_PinMux(LPC_IOCON, PORT(0), PIN(1), MODE(0), FUNC3); // SCL1 / D21 SCL
  #elif USEDI2CDEV_M == 2
    Chip_IOCON_PinMux(LPC_IOCON, PORT(0), PIN(10), MODE(0), FUNC2); // SDA2 / D38  X_ENABLE_PIN
    Chip_IOCON_PinMux(LPC_IOCON, PORT(0), PIN(11), MODE(0), FUNC2); // SCL2 / D55  X_DIR_PIN
  #endif

  // Initialize I2C peripheral
  Chip_I2C_Init(I2CDEV_M);
  Chip_I2C_SetClockRate(I2CDEV_M, 100000);

  // Enable Master I2C operation
  // TODO: Looks like this is no longer needed
  //I2C_Cmd(I2CDEV_M, I2C_MASTER_MODE, ENABLE);
}

uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity) {
  // clamp to buffer length
  if (quantity > BUFFER_LENGTH)
    quantity = BUFFER_LENGTH;

  // perform blocking read into buffer
  I2C_XFER_T transferMCfg;
  transferMCfg.slaveAddr = address >> 1; // not sure about the right shift
  transferMCfg.txBuff = NULL;
  transferMCfg.txSz = 0;
  transferMCfg.rxBuff = rxBuffer;
  transferMCfg.rxSz = quantity;
  //transferMCfg.retransmissions_max = 3;
  // TODO: Low confidence this will work
  Chip_I2C_MasterTransfer(I2CDEV_M, &transferMCfg);

  // set rx buffer iterator vars
  rxBufferIndex = 0;
  rxBufferLength = transferMCfg.rxSz;

  return transferMCfg.rxSz;
}

uint8_t TwoWire::requestFrom(int address, int quantity) {
  return requestFrom((uint8_t)address, (uint8_t)quantity);
}

void TwoWire::beginTransmission(uint8_t address) {
  // indicate that we are transmitting
  transmitting = 1;
  // set address of targeted slave
  txAddress = address;
  // reset tx buffer iterator vars
  txBufferIndex = 0;
  txBufferLength = 0;
}

void TwoWire::beginTransmission(int address) {
  beginTransmission((uint8_t)address);
}

uint8_t TwoWire::endTransmission(void) {
  // transmit buffer (blocking)
  I2C_XFER_T transferMCfg;
  transferMCfg.slaveAddr = txAddress >> 1; // not sure about the right shift
  transferMCfg.txBuff = txBuffer;
  transferMCfg.txSz = txBufferLength;
  transferMCfg.rxBuff = NULL;
  transferMCfg.rxSz = 0;
  //transferMCfg.retransmissions_max = 3;
  // TODO: Low confidence this will work
  uint32_t status = Chip_I2C_MasterTransfer(I2CDEV_M, &transferMCfg);

  // reset tx buffer iterator vars
  txBufferIndex = 0;
  txBufferLength = 0;

  // indicate that we are done transmitting
  transmitting = 0;

  return status == I2C_STATUS_DONE ? 0 : 4;
}

// must be called after beginTransmission(address)
size_t TwoWire::write(uint8_t data) {
  if (transmitting) {
    // don't bother if buffer is full
    if (txBufferLength >= BUFFER_LENGTH) return 0;

    // put byte in tx buffer
    txBuffer[txBufferIndex++] = data;

    // update amount in buffer
    txBufferLength = txBufferIndex;
  }

  return 1;
}

// must be called after beginTransmission(address)
size_t TwoWire::write(const uint8_t *data, size_t quantity) {
  size_t sent = 0;
  if (transmitting)
    for (sent = 0; sent < quantity; ++sent)
      if (!write(data[sent])) break;

  return sent;
}

// Must be called after requestFrom(address, numBytes)
int TwoWire::available(void) {
  return rxBufferLength - rxBufferIndex;
}

// Must be called after requestFrom(address, numBytes)
int TwoWire::read(void) {
  return rxBufferIndex < rxBufferLength ? rxBuffer[rxBufferIndex++] : -1;
}

// Must be called after requestFrom(address, numBytes)
int TwoWire::peek(void) {
  return rxBufferIndex < rxBufferLength ? rxBuffer[rxBufferIndex] : -1;
}

// Preinstantiate Objects //////////////////////////////////////////////////////

TwoWire Wire = TwoWire();
