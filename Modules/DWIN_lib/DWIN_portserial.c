#include "DWIN_lib.h"

/* ----------------------- static functions ---------------------------------*/
static void UARTTxReadyISR( void );
static void UARTRxISR( void );

/* ----------------------- Variables ----------------------------------------*/
extern UART_HandleTypeDef* DWIN_uart;
uint8_t DWIN_txByte = 0x00;
uint8_t DWIN_rxByte = 0x00;

BOOL xDWINPortSerialInit( UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eDWINParity eParity ) {
		/* Initilize some data */
}

void
vDWINPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
{
    /* If xRXEnable enable serial receive interrupts. If xTxENable enable
     * transmitter empty interrupts.
     */
		if (xRxEnable == FALSE)
		{
			HAL_UART_AbortReceive_IT(DWIN_uart);
		}
		else
		{
			HAL_UART_Receive_IT(DWIN_uart, &DWIN_rxByte, 1);
		}
		if (xTxEnable == FALSE)
		{
			HAL_UART_AbortTransmit_IT(DWIN_uart);
		}
		else
		{
			if (DWIN_uart->gState == HAL_UART_STATE_READY)
			{
				UARTTxReadyISR();
			}
		}
}

BOOL
xDWINPortSerialGetByte( CHAR * pucByte )
{
    /* Return the byte in the UARTs receive buffer. This function is called
     * by the protocol stack after pxMBFrameCBByteReceived( ) has been called.
     */
		
		*pucByte = DWIN_rxByte;
		HAL_UART_Receive_IT(DWIN_uart, &DWIN_rxByte, 1);
    return TRUE;
}

/* Create an interrupt handler for the transmit buffer empty interrupt
 * (or an equivalent) for your target processor. This function should then
 * call pxMBFrameCBTransmitterEmpty( ) which tells the protocol stack that
 * a new character can be sent. The protocol stack will then call 
 * xMBPortSerialPutByte( ) to send the character.
 */

static void UARTTxReadyISR( void )
{

}

/* Create an interrupt handler for the receive interrupt for your target
 * processor. This function should then call pxMBFrameCBByteReceived( ). The
 * protocol stack will then call xMBPortSerialGetByte( ) to retrieve the
 * character.
 */
static void UARTRxISR( void )
{

}

/* --------------------------------------------------------------------------*/
#if DWIN_SERIAL_PORT_ENABLE
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{

}
/* --------------------------------------------------------------------------*/



extern volatile UCHAR  ucDWINBuf[];

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	 if (huart->Instance == DWIN_uart->Instance)
		{
				CMSIS_DMA_Config(DMA2_Stream2, &(USART1->DR), (uint32_t*)ucDWINBuf, 3);
				DWIN_uart->Instance->SR &= ~USART_SR_RXNE;				
				//xDWINPortEventPost(DWIN_EV_FRAME_RECEIVED);

		}
}
#endif
/* --------------------------------------------------------------------------*/

void DMA2_Stream2_IRQHandler(void)
{
	eDWINEventType xEvent;
	xDWINPortEventGet(&xEvent);
	
	if (xEvent == DWIN_EV_READY) {
		xDWINPortEventPost(DWIN_EV_FRAME_RECEIVED);
	} else if (xEvent == DWIN_EV_FRAME_RECEIVED) {
		xDWINPortEventPost(DWIN_EV_DATA_RECEIVED);
	}
	
	if (DMA2->LISR & DMA_LISR_TCIF2){
		DMA2->LIFCR |= DMA_LIFCR_CTCIF2;
		DMA2->LIFCR |= DMA_LIFCR_CHTIF2;
	}
}