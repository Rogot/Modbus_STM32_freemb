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
eDWINErrorCode eDWINSend( UCHAR slaveAddress, const UCHAR * pucFrame, USHORT usLength ) {}
	
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

eDWINErrorCode eDWINPoll( void ) {
	static UCHAR*	 ucDWINFrame;
	static UCHAR    ucRcvAddress;
  static UCHAR    ucFunctionCode;
  static USHORT 	 usLength;
  
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
					xDWINPortEventPost(DWIN_EV_READY);
					break;
				
				case DWIN_EV_EXECUTE:
					//( void )xDWINPortEventPost( DWIN_EV_FRAME_SENT);
					break;
				
				case DWIN_EV_FRAME_SENT:
				//	( void )xDWINPortEventPost( DWIN_EV_READY);
					break;
			}
	}
	
	return DWIN_ENOERR;
}



