/**********************************************************************
* $Id$    debug_frmwrk.c        2010-05-21
*
* @file   debug_frmwrk.c
* @brief  Contains some utilities that used for debugging through UART
* @version  2.0
* @date   21. May. 2010
* @author NXP MCU SW Application Team
*
* Copyright(C) 2010, NXP Semiconductor
* All rights reserved.
*
***********************************************************************
* Software that is described herein is for illustrative purposes only
* which provides customers with programming information regarding the
* products. This software is supplied "AS IS" without any warranties.
* NXP Semiconductors assumes no responsibility or liability for the
* use of the software, conveys no license or title under any patent,
* copyright, or mask work right to the product. NXP Semiconductors
* reserves the right to make changes in the software without
* notification. NXP Semiconductors also make no representation or
* warranty that such application will be suitable for the specified
* use without further testing or modification.
* Permission to use, copy, modify, and distribute this software and its
* documentation is hereby granted, under NXP Semiconductors'
* relevant copyright in the software, without fee, provided that it
* is used in conjunction with NXP Semiconductors microcontrollers.  This
* copyright, permission, and disclaimer notice must appear in all copies of
* this code.
**********************************************************************/

#include "debug_frmwrk.h"
#include "chip.h"

/* If this source file built with example, the LPC17xx FW library configuration
 * file in each example directory ("lpc17xx_libcfg.h") must be included,
 * otherwise the default FW library configuration file must be included instead
 */
#ifdef __BUILD_WITH_EXAMPLE__
  #include "lpc17xx_libcfg.h"
#else
  #include "lpc17xx_libcfg_default.h"
#endif

#ifdef _DBGFWK

/* Debug framework */
static Bool debug_frmwrk_initialized = FALSE;

void (*_db_msg)(LPC_USART_T *UARTx, const void *s) = UARTPuts;
void (*_db_msg_)(LPC_USART_T *UARTx, const void *s) = UARTPuts_;
void (*_db_char)(LPC_USART_T *UARTx, uint8_t ch) = UARTPutChar;
void (*_db_dec)(LPC_USART_T *UARTx, uint8_t decn) = UARTPutHex;
void (*_db_dec_16)(LPC_USART_T *UARTx, uint16_t decn) = UARTPutHex16;
void (*_db_dec_32)(LPC_USART_T *UARTx, uint32_t decn) = UARTPutHex32;
void (*_db_hex)(LPC_USART_T *UARTx, uint8_t hexn) = UARTPutDec;
void (*_db_hex_16)(LPC_USART_T *UARTx, uint16_t hexn) = UARTPutDec16;
void (*_db_hex_32)(LPC_USART_T *UARTx, uint32_t hexn) = UARTPutDec32;
uint8_t (*_db_get_char)(LPC_USART_T *UARTx) = UARTGetChar;


/*********************************************************************//**
 * @brief   Puts a character to UART port
 * @param[in] UARTx Pointer to UART peripheral
 * @param[in] ch    Character to put
 * @return    None
 **********************************************************************/
void UARTPutChar(LPC_USART_T *UARTx, uint8_t ch) {
  if (debug_frmwrk_initialized)
     Chip_UART_SendBlocking(UARTx, &ch, 1);
}

/*********************************************************************//**
 * @brief   Get a character to UART port
 * @param[in] UARTx Pointer to UART peripheral
 * @return    character value that returned
 **********************************************************************/
uint8_t UARTGetChar(LPC_USART_T *UARTx) {
  uint8_t tmp = 0;

  if (debug_frmwrk_initialized)
    Chip_UART_ReadBlocking(UARTx, &tmp, 1);

  return(tmp);
}

/*********************************************************************//**
 * @brief   Puts a string to UART port
 * @param[in] UARTx   Pointer to UART peripheral
 * @param[in] str   string to put
 * @return    None
 **********************************************************************/
void UARTPuts(LPC_USART_T *UARTx, const void *str) {
  if (!debug_frmwrk_initialized) return;

  uint8_t *s = (uint8_t*)str;
  while (*s) UARTPutChar(UARTx, *s++);
}

/*********************************************************************//**
 * @brief   Puts a string to UART port and print new line
 * @param[in] UARTx Pointer to UART peripheral
 * @param[in] str   String to put
 * @return    None
 **********************************************************************/
void UARTPuts_(LPC_USART_T *UARTx, const void *str) {
  if (!debug_frmwrk_initialized) return;

  UARTPuts (UARTx, str);
  UARTPuts (UARTx, "\n\r");
}

/*********************************************************************//**
 * @brief   Puts a decimal number to UART port
 * @param[in] UARTx Pointer to UART peripheral
 * @param[in] decnum  Decimal number (8-bit long)
 * @return    None
 **********************************************************************/
void UARTPutDec(LPC_USART_T *UARTx, uint8_t decnum) {
  if (!debug_frmwrk_initialized) return;

  uint8_t c1 = decnum%10;
  uint8_t c2 = (decnum /             10) % 10;
  uint8_t c3 = (decnum /            100) % 10;
  UARTPutChar(UARTx, '0'+c3);
  UARTPutChar(UARTx, '0'+c2);
  UARTPutChar(UARTx, '0'+c1);
}

/*********************************************************************//**
 * @brief   Puts a decimal number to UART port
 * @param[in] UARTx Pointer to UART peripheral
 * @param[in] decnum  Decimal number (8-bit long)
 * @return    None
 **********************************************************************/
void UARTPutDec16(LPC_USART_T *UARTx, uint16_t decnum) {
  if (!debug_frmwrk_initialized) return;

  uint8_t c1 = decnum%10;
  uint8_t c2 = (decnum /             10) % 10;
  uint8_t c3 = (decnum /            100) % 10;
  uint8_t c4 = (decnum /           1000) % 10;
  uint8_t c5 = (decnum /          10000) % 10;
  UARTPutChar(UARTx, '0'+c5);
  UARTPutChar(UARTx, '0'+c4);
  UARTPutChar(UARTx, '0'+c3);
  UARTPutChar(UARTx, '0'+c2);
  UARTPutChar(UARTx, '0'+c1);
}

/*********************************************************************//**
 * @brief   Puts a decimal number to UART port
 * @param[in] UARTx Pointer to UART peripheral
 * @param[in] decnum  Decimal number (8-bit long)
 * @return    None
 **********************************************************************/
void UARTPutDec32(LPC_USART_T *UARTx, uint32_t decnum) {
  if (!debug_frmwrk_initialized) return;

  const uint8_t  c1 =  decnum               % 10,
                 c2 = (decnum /         10) % 10,
                 c3 = (decnum /        100) % 10,
                 c4 = (decnum /       1000) % 10,
                 c5 = (decnum /      10000) % 10,
                 c6 = (decnum /     100000) % 10,
                 c7 = (decnum /    1000000) % 10,
                 c8 = (decnum /   10000000) % 10,
                 c9 = (decnum /  100000000) % 10,
                c10 = (decnum / 1000000000) % 10;
  UARTPutChar(UARTx, '0' + c10);
  UARTPutChar(UARTx, '0' +  c9);
  UARTPutChar(UARTx, '0' +  c8);
  UARTPutChar(UARTx, '0' +  c7);
  UARTPutChar(UARTx, '0' +  c6);
  UARTPutChar(UARTx, '0' +  c5);
  UARTPutChar(UARTx, '0' +  c4);
  UARTPutChar(UARTx, '0' +  c3);
  UARTPutChar(UARTx, '0' +  c2);
  UARTPutChar(UARTx, '0' +  c1);
}

/*********************************************************************//**
 * @brief   Puts a hex number to UART port
 * @param[in] UARTx Pointer to UART peripheral
 * @param[in] hexnum  Hex number (8-bit long)
 * @return    None
 **********************************************************************/
void UARTPutHex(LPC_USART_T *UARTx, uint8_t hexnum) {
  if (!debug_frmwrk_initialized) return;

  UARTPuts(UARTx, "0x");
  uint8_t nibble, i = 1;
  do {
    nibble = (hexnum >> (4 * i)) & 0x0F;
    UARTPutChar(UARTx, (nibble > 9) ? ('A' + nibble - 10) : ('0' + nibble));
  } while (i--);
}

/*********************************************************************//**
 * @brief   Puts a hex number to UART port
 * @param[in] UARTx Pointer to UART peripheral
 * @param[in] hexnum  Hex number (16-bit long)
 * @return    None
 **********************************************************************/
void UARTPutHex16(LPC_USART_T *UARTx, uint16_t hexnum) {
  if (!debug_frmwrk_initialized) return;

  UARTPuts(UARTx, "0x");
  uint8_t nibble, i = 3;
  do {
    nibble = (hexnum >> (4 * i)) & 0x0F;
    UARTPutChar(UARTx, (nibble > 9) ? ('A' + nibble - 10) : ('0' + nibble));
  } while (i--);
}

/*********************************************************************//**
 * @brief   Puts a hex number to UART port
 * @param[in] UARTx Pointer to UART peripheral
 * @param[in] hexnum  Hex number (32-bit long)
 * @return    None
 **********************************************************************/
void UARTPutHex32(LPC_USART_T *UARTx, uint32_t hexnum) {
  if (!debug_frmwrk_initialized) return;

  UARTPuts(UARTx, "0x");
  uint8_t nibble, i = 7;
  do {
    nibble = (hexnum >> (4 * i)) & 0x0F;
    UARTPutChar(UARTx, (nibble > 9) ? ('A' + nibble - 10) : ('0' + nibble));
  } while (i--);
}

/*********************************************************************//**
 * @brief   print function that supports format as same as printf()
 *        function of <stdio.h> library
 * @param[in] None
 * @return    None
 **********************************************************************/
//void  _printf (const  char *format, ...) {
//  static  char  buffer[512 + 1];
//          va_list     vArgs;
//          char  *tmp;
//  va_start(vArgs, format);
//  vsprintf((char *)buffer, (char const *)format, vArgs);
//  va_end(vArgs);
//
//  _DBG(buffer);
//}

/*********************************************************************//**
 * @brief   Initialize Debug frame work through initializing UART port
 * @param[in] None
 * @return    None
 **********************************************************************/
void debug_frmwrk_init(void) {
  // Values common to all supported UARTs
  const uint8_t portNum = 0, pinMode = 0, pinFunc = 0;

  #if (USED_UART_DEBUG_PORT==0)
    const uint8_t pinNum1 = 2, pinNum2 = 3;
    LPC_USART_T * const DEBUG_UART = LPC_UART0;
  #elif (USED_UART_DEBUG_PORT==1)
    const uint8_t pinNum1 = 15, pinNum2 = 16;
    LPC_USART_T * const DEBUG_UART = LPC_UART1;
  #else
    #error "USED_UART_DEBUG_PORT value is not supported"
  #endif

  Chip_IOCON_PinMux(LPC_IOCON, portNum, pinNum1, pinMode, pinFunc);
  Chip_IOCON_PinMux(LPC_IOCON, portNum, pinNum2, pinMode, pinFunc);

  Chip_UART_Init(DEBUG_UART);
  Chip_UART_SetBaudFDR(DEBUG_UART, 115200);
  Chip_UART_ConfigData(DEBUG_UART, (UART_LCR_WLEN8 | UART_LCR_SBS_1BIT));
  Chip_UART_TXEnable(DEBUG_UART);

  debug_frmwrk_initialized = TRUE;
}

#endif // _DBGFWK
