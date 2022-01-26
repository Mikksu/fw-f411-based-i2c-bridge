/*
 * FreeModbus Libary: BARE Port
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id$
 */

#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- static functions ---------------------------------*/
/*
static void prvvUARTTxReadyISR( void );
static void prvvUARTRxISR( void );
*/

#ifdef MB_OVER_VCP

/**
 * @brief     The handle of the FIFO to save the data received from VCP.
 */
fifo_t                vcpRxFifo, *hVcpRxFifo;
fifo_t                vcpTxFifo, *hVcpTxFifo;

#else

/*
 * @brief   The pointer to the usart which is used by the Modbus.
 */
UART_HandleTypeDef   *huart               = NULL;

/*
 * @brief   The Gpio which is used to control the DE pin of the RS485.
 */
GPIO_TypeDef         *rs485DeGpio         = NULL;
uint16_t             rs485DePin           = 0;

/*
 * @brief   RS485 Rx state activation level.
 *          When the RS485 ED pin level is set to this value, it's in the receiving state.
 */
GPIO_PinState       rs485DeActivateLevel  = GPIO_PIN_RESET;

#endif




/* ----------------------- Start implementation -----------------------------*/
void
vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
{

#ifdef MB_OVER_VCP

  if(xTxEnable == TRUE)
  {
    osThreadResume(vcpMbTxPollTaskHandle);
  }

  /*
  if(xRxEnable == TRUE)
  {
    osThreadResume(vcpMbRxTaskHandle);
  }
  else
  {
    osThreadSuspend(vcpMbRxTaskHandle);
  }

  if(xTxEnable == TRUE)
    {
      osThreadResume(vcpMbTxPollTaskHandle);
    }
    else
    {
      osThreadSuspend(vcpMbTxPollTaskHandle);
    }
    */

#else
  /* If xRXEnable enable serial receive interrupts. If xTxENable enable
    * transmitter empty interrupts.
    */

  // disable the USART to avoid receiving data.
  __HAL_UART_DISABLE(huart);

  // switch the RS485 to RX state.
  // the rs485 should be always in RX state.
  if(rs485DeGpio != NULL)
    HAL_GPIO_WritePin(rs485DeGpio, rs485DePin, rs485DeActivateLevel);

  if (xRxEnable == TRUE)
  {
    // enable the RX interrupt.
    huart->Instance->CR1 |= UART_MODE_RX;
    __HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);
  }
  else if (xRxEnable == FALSE)
  {
    // disable the RX interrupt.
    huart->Instance->CR1 &= (~UART_MODE_RX);
    __HAL_UART_DISABLE_IT(huart, UART_IT_RXNE);
  }

  //发送使能
  if (xTxEnable == TRUE)
  {
    huart->Instance->CR1 |= UART_MODE_TX;
    __HAL_UART_ENABLE_IT(huart, UART_IT_TC);

    // switch the RS485 to TX state.
    if(rs485DeGpio != NULL)
        HAL_GPIO_WritePin(rs485DeGpio, rs485DePin, (GPIO_PinState)(!rs485DeActivateLevel));

  }
  else if (xTxEnable == FALSE)
  {
    huart->Instance->CR1 &= (~UART_MODE_TX);
    __HAL_UART_DISABLE_IT(huart, UART_IT_TC);
  }


  // 打开其它中断
  if(xRxEnable | xTxEnable)
  {
    __HAL_UART_ENABLE_IT(huart, UART_IT_PE);
    __HAL_UART_ENABLE_IT(huart, UART_IT_ERR);
    __HAL_UART_ENABLE(huart);
  }
#endif
}

BOOL xMBPortSerialInit( UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
{

#ifdef MB_OVER_VCP

  // initialize the FIFO to save data received from VCP.
  hVcpRxFifo = &vcpRxFifo;
  fifo_init(hVcpRxFifo);

  hVcpTxFifo = &vcpTxFifo;
  fifo_init(hVcpTxFifo);

  return TRUE;

#else
	/* 
  Do nothing, Initialization is handled by MX_USART3_UART_Init() 
  Fixed port, baudrate, databit and parity  
  */

  if(ucPORT == 1)
    huart = &huart1;

  assert(huart != NULL);

  HAL_UART_DeInit(huart);

  huart->Init.BaudRate = (uint32_t)ulBaudRate;


  if(eParity ==MB_PAR_ODD)
    huart->Init.Parity = UART_PARITY_ODD;
  else if(eParity ==MB_PAR_EVEN)
      huart->Init.Parity = UART_PARITY_EVEN;
  else
    huart->Init.Parity = UART_PARITY_NONE;

  if (HAL_UART_Init(huart) != HAL_OK)
   {
     Error_Handler();
   }

	return TRUE;
#endif
}

BOOL
xMBPortSerialPutByte( CHAR ucByte )
{

#ifdef MB_OVER_VCP

  fifo_put(hVcpTxFifo, (uint8_t)ucByte);
  return TRUE;

#else
    /* Put a byte in the UARTs transmit buffer. This function is called
     * by the protocol stack if pxMBFrameCBTransmitterEmpty( ) has been
     * called. */

	return (HAL_OK == HAL_UART_Transmit(huart, (uint8_t*)&ucByte, 1, 10));

#endif
}

BOOL
xMBPortSerialGetByte( CHAR * pucByte )
{
    /* Return the byte in the UARTs receive buffer. This function is called
     * by the protocol stack after pxMBFrameCBByteReceived( ) has been called.
     */

#ifdef MB_OVER_VCP

  int ret = fifo_get(hVcpRxFifo, (uint8_t*)pucByte);
  return ret;

#else

  *pucByte = (uint8_t)(huart->Instance->DR & (uint8_t)0x00FF);
  return TRUE;

#endif
}


/* Create an interrupt handler for the transmit buffer empty interrupt
 * (or an equivalent) for your target processor. This function should then
 * call pxMBFrameCBTransmitterEmpty( ) which tells the protocol stack that
 * a new character can be sent. The protocol stack will then call 
 * xMBPortSerialPutByte( ) to send the character.
 */
/*
static void prvvUARTTxReadyISR( void )
{
    pxMBFrameCBTransmitterEmpty(  );
}
*/

/* Create an interrupt handler for the receive interrupt for your target
 * processor. This function should then call pxMBFrameCBByteReceived( ). The
 * protocol stack will then call xMBPortSerialGetByte( ) to retrieve the
 * character.
 */
/*
static void prvvUARTRxISR( void )
{
    pxMBFrameCBByteReceived(  );
}
*/


