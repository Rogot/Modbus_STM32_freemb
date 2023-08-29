#include "DWIN_lib.h"

/* ----------------------- Type definitions ---------------------------------*/
typedef enum
{
    DWIN_STATE_RX_INIT,              /*!< Receiver is in initial state. */
    DWIN_STATE_RX_IDLE,              /*!< Receiver is in idle state. */
    DWIN_STATE_RX_RCV,               /*!< Frame is beeing received. */
    DWIN_STATE_RX_ERROR              /*!< If the frame is invalid. */
} eDWINRcvState;

typedef enum
{
    DWIN_STATE_TX_IDLE,              /*!< Transmitter is in idle state. */
    DWIN_STATE_TX_XMIT               /*!< Transmitter is in transfer state. */
} eDWINSndState;

/* ----------------------- Static variables ---------------------------------*/

//extern t_modbus_him_pack rxData;
DMA_Stream_TypeDef* DWIN_dma;
UART_HandleTypeDef* DWIN_uart;
extern eDWINEventType xEvent;

static UCHAR    ucDWINAddress;
static eDWINMode  eDWINCurrentMode;

static volatile eDWINSndState eDWSndState;
static volatile eDWINRcvState eDWRcvState;

volatile UCHAR  ucDWINBuf[DWIN_SER_PDU_SIZE_MAX];

volatile USHORT  ucRegistersBuf[DWIN_SER_PDU_SIZE_MAX];

static volatile UCHAR *pucDWSndBufferCur;
static volatile USHORT usDWSndBufferCount;

static volatile USHORT usDWRcvBufferPos;

/* 
 * Functions pointer which are initialized in eMBInit( ).
 */
static peDWINFrameSend peDWINFrameSendCur;
static pvDWINFrameStart pvDWINFrameStartCur;
static pvDWINFrameStop pvDWINFrameStopCur;
static peDWINFrameReceive peDWINFrameReceiveCur;
static pvDWINFrameClose pvDWINFrameCloseCur;

/* Callback functions required by the porting layer. They are called when
 * an external event has happend which includes a timeout or the reception
 * or transmission of a character.
 */

BOOL( *pxDWINFrameCBReceiveFSMCur ) ( void );
BOOL( *pxDWINFrameCBTransmitFSMCur ) ( void );

static enum
{
    DWIN_STATE_ENABLED,
    DWIN_STATE_DISABLED,
    DWIN_STATE_NOT_INITIALIZED
} eDWINState = DWIN_STATE_NOT_INITIALIZED;

eDWINErrorCode
eDWINInit(UCHAR ucSlaveAddress, UCHAR ucPort, ULONG ulBaudRate, eDWINParity eParity) {
	eDWINErrorCode    eStatus = DWIN_ENOERR;
	
	/* check preconditions */
	if ( (ucSlaveAddress == DWIN_ADDRESS_BROADCAST) || 
			 (ucSlaveAddress < DWIN_ADDRESS_MIN) || (ucSlaveAddress > DWIN_ADDRESS_MAX)) {
		eStatus = DWIN_EINVAL;
	} else {
		ucDWINAddress = ucSlaveAddress;
		
		peDWINFrameSendCur = eDWINSend;
		pvDWINFrameStartCur = eDWINStart;
		pvDWINFrameStopCur = eDWINStop;
		peDWINFrameReceiveCur = eDWINReceive;
		pvDWINFrameCloseCur = eDWINClose;
		
		eStatus = DWIN_EPORTERR;
		/* DWIN protocol uses 8 Databits. */
		eStatus = xDWINPortSerialInit(ucDWINAddress, ulBaudRate, 8, eParity);
	}
	
	eDWINState = DWIN_STATE_DISABLED;
	xDWINPortEventPost(DWIN_EV_READY);
	
	return eStatus;
}

eDWINErrorCode
eDWINReceive( UCHAR * pucRcvAddress, UCHAR ** pucFrame,
														USHORT * pusLength, eDWINEventType * evType) {
	 BOOL            xFrameReceived = FALSE;
   eDWINErrorCode    eStatus = DWIN_ENOERR;
	 eDWINEventType		 eType =  *evType;
	 eDWINEventType eDWEvent;
															
															
	 ENTER_CRITICAL_SECTION(  );
	 assert( usDWRcvBufferPos < DWIN_SER_PDU_SIZE_MAX );
	
	 #if DWIN_CRC_ENABLE	
	 /* Length and CRC check */
   if( ( usDWRcvBufferPos >= DWIN_SER_PDU_SIZE_MIN )
       && ( usMBCRC16( ( UCHAR * ) ucDWINBuf, usDWRcvBufferPos ) == 0 ) )
   {
	 #endif
	 switch (eType) {
		 case DWIN_EV_FRAME_RECEIVED:
			if (ucDWINBuf[DWIN_START_POS] == DWIN_START_BIT) {
				*pucRcvAddress = ucDWINBuf[DWIN_SLAVE_ID_POS];
				*pusLength = ucDWINBuf[DWIN_WORD_LENGTH_POS];
				CMSIS_DMA_Config(DWIN_dma, &(DWIN_uart->Instance->DR),
												(uint32_t*)(ucDWINBuf + 3),
												 ucDWINBuf[DWIN_WORD_LENGTH_POS]);
				xFrameReceived = TRUE;
			} else { 
				//xDWINPortEventPost(DWIN_EV_READY);
				//CMSIS_DMA_Config(DWIN_dma, &(USART1->DR), (uint32_t*)ucDWINBuf, 3);
			}
			break;
		 case DWIN_EV_DATA_RECEIVED:
				CMSIS_DMA_Config(DWIN_dma, &(DWIN_uart->Instance->DR),
												(uint32_t*)ucDWINBuf, 3);
				(void) DWIN_uart->Instance->DR;
				//DWIN_uart->Instance->CR1 |= USART_CR1_RXNEIE;
				xDWINPortEventPost(DWIN_EV_READY);
			 break;
	 }
	
	 #if DWIN_CRC_ENABLE	
	 }
	 #endif
	 
	 EXIT_CRITICAL_SECTION( );
	 return eStatus;
}
eDWINErrorCode eDWINSend( UCHAR slaveAddress, const UCHAR * pucFrame, USHORT usLength ) {
		eDWINErrorCode		eStatus = DWIN_ENOERR;
		USHORT						usCRC16;
	
		ENTER_CRITICAL_SECTION();
	
		/* First byte before the Modbus-PDU is the slave address. */
		//pucDWSndBufferCur = ( UCHAR * ) pucFrame - 1;
		//usDWSndBufferCount = 1;
			
		ucDWINBuf[0] = 0x01;
		ucDWINBuf[1] = 0x02;
		ucDWINBuf[2] = 0x03;
		/* Now copy the Modbus-PDU into the Modbus-Serial-Line-PDU. */
		CMSIS_DMA_Config(DMA2_Stream7, (uint32_t*)ucDWINBuf, &(DWIN_uart->Instance->DR), 3);
		
}
	
void eDWINStart( void ) {
	ENTER_CRITICAL_SECTION(  );
	
	eDWRcvState = DWIN_STATE_RX_INIT;
	vDWINPortSerialEnable( TRUE, FALSE );
	
	EXIT_CRITICAL_SECTION(  );
}	

void eDWINStop( void ) {
	vDWINPortSerialEnable( FALSE, FALSE );
}

void eDWINClose( void ) {}
eDWINErrorCode eDWINEnable( void ) {
	
	
	eDWINErrorCode    eStatus = DWIN_ENOERR;
	if( eDWINState == DWIN_STATE_DISABLED )
   {
       /* Activate the protocol stack. */
       pvDWINFrameStartCur(  );
       eDWINState = DWIN_STATE_ENABLED;
   }
   else
   {
       eStatus = DWIN_EILLSTATE;
   }
   return eStatus;
	
}
void eDWINDisable( void ) {}
void eDWINSetSlaveID( void ) {}
	
void eDWINFuncReadRegister(UCHAR * pucFrame, USHORT * usLen) {
		USHORT          	usRegAddress;
    USHORT          	usRegCount;
    UCHAR          		*pucFrameCur;
	
	  eDWINException    eStatus = DWIN_EX_NONE;
    eDWINErrorCode    eRegStatus;
		
		for (uint8_t i = 0; i < *usLen - 1; i += 4) {
			usRegAddress = ( USHORT )( pucFrame[DWIN_ADDR_START_DATA + i] << 8 );
			usRegAddress |= ( USHORT )( pucFrame[DWIN_ADDR_START_DATA + i + 1] );
			//usRegAddress++;
			
			pucFrame[DWIN_ADDR_START_DATA + i] = usRegAddress >> 8;
			pucFrame[DWIN_ADDR_START_DATA + i + 1] |= usRegAddress & 0xFF;
			
			pucFrame[DWIN_ADDR_START_DATA + i + 2] = ucRegistersBuf[usRegAddress] >> 8;
			pucFrame[DWIN_ADDR_START_DATA + i + 3] |= ucRegistersBuf[usRegAddress] & 0xFF;
			
			//usRegCount = *usLen / 2;
		}
}

void eDWINFuncWriteRegister(UCHAR * pucFrame, USHORT * usLen) {
		USHORT          	usRegAddress;
    USHORT          	usRegCount;
    UCHAR          		*pucFrameCur;
	
	  eDWINException    eStatus = DWIN_EX_NONE;
    eDWINErrorCode    eRegStatus;
		
		for (uint8_t i = 0; i < *usLen - 1; i += 4) {
			
			usRegAddress = ( USHORT )( pucFrame[DWIN_ADDR_START_DATA + i] << 8);
			usRegAddress |= ( USHORT )( pucFrame[DWIN_ADDR_START_DATA + i + 1] );
			//usRegAddress++;
			
			usRegCount = (pucFrame[DWIN_ADDR_START_DATA + i + 2] << 8) | 
									(pucFrame[DWIN_ADDR_START_DATA + i + 3]);
			
			pucFrame[DWIN_ADDR_START_DATA + i + 4] = ucRegistersBuf[usRegAddress] >> 8;
			pucFrame[DWIN_ADDR_START_DATA + i + 5] = ucRegistersBuf[usRegAddress] & 0xFF;
		}
}

eDWINErrorCode eDWINPoll( void ) {
	static UCHAR*	 ucDWINFrame;
	static UCHAR	 ucDWINFrame_test[5];
	static UCHAR    ucRcvAddress;
  static UCHAR    ucFunctionCode;
  static USHORT 	 usLength;
	
	static eDWINException eException;
  
	int								i;
	eDWINErrorCode    eStatus = DWIN_ENOERR;
	eDWINEventType    eEvent;
	
	/* Check if the protocol stack is ready. */
	if( eDWINState != DWIN_STATE_ENABLED )
  {
		return DWIN_EILLSTATE;
  }
	
	/* Check if there is a event available. If not return control to caller.
   * Otherwise we will handle the event. */
  if( xDWINPortEventGet( &eEvent ) == TRUE )
  {
			switch (eEvent) {
				case DWIN_EV_READY:

					break;
				
				case DWIN_EV_FRAME_RECEIVED:
					eStatus = peDWINFrameReceiveCur( &ucRcvAddress, &ucDWINFrame, &usLength, &eEvent);
					if( eStatus == DWIN_ENOERR )
          {
						/* Check if the frame is for us. If not ignore the frame. */
            if( ( ucRcvAddress != ucDWINAddress ) && ( ucRcvAddress != DWIN_ADDRESS_BROADCAST ) )
            {
							( void )xDWINPortEventPost( DWIN_EV_READY );
						}
          }
					break;
				
				case DWIN_EV_DATA_RECEIVED:
					eStatus = peDWINFrameReceiveCur( &ucRcvAddress, &ucDWINFrame, &usLength, &eEvent);
					xDWINPortEventPost(DWIN_EV_EXECUTE);
					break;
				
				case DWIN_EV_EXECUTE:
					//CMSIS_DMA_Config(DMA2_Stream7, (uint32_t*)ucDWINBuf, &(DWIN_uart->Instance->DR), 5);
					HAL_UART_Transmit_DMA(DWIN_uart, (uint8_t *)ucDWINBuf, sizeof(ucDWINBuf));
					#if 0
					ucFunctionCode = ucDWINBuf[DWIN_CMD_POS];
					eException = DWIN_EX_ILLEGAL_FUNCTION;
				
					if (ucFunctionCode == FUNC_CODE_READ) {
						eDWINFuncReadRegister(ucDWINBuf, &usLength);
					} 
					else if (ucFunctionCode == FUNC_CODE_WRITE) {
						eDWINFuncWriteRegister(ucDWINBuf, &usLength);
					}
					/* If the request was not sent to the broadcast address we
           * return a reply. */
					if ( ucDWINAddress != DWIN_ADDRESS_BROADCAST ) 
					{
						if ( eException != DWIN_EX_NONE ) 
						{
							/* An exception occured. Build an error frame. */
							usLength = 0;
							//( UCHAR ) 
							ucDWINFrame[0] = ( ucFunctionCode );
							usLength++;
							ucDWINFrame[1] = eException;
							ucDWINFrame_test[0] = 0x01;
							ucDWINFrame_test[1] = 0x02;
							ucDWINFrame_test[2] = 0x03;
							ucDWINFrame_test[3] = 0x04;
							ucDWINFrame_test[4] = 0x05;
						}
					}
					//eStatus = peDWINFrameSendCur ( ucDWINAddress, ucDWINFrame_test, usLength );
					//eStatus = peDWINFrameSendCur ( ucDWINAddress, ucDWINFrame, usLength );
					#endif
					//( void )xDWINPortEventPost( DWIN_EV_FRAME_SENT );
					break;
				
				case DWIN_EV_FRAME_SENT:
					( void )xDWINPortEventPost( DWIN_EV_READY );
					break;
			}
	}
	
	return DWIN_ENOERR;
}



