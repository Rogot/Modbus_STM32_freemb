#ifndef DWIN_PORT_H
#define DWIN_PORT_H
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
/* Declarations and definitions ----------------------------------------------*/
/* Functions -----------------------------------------------------------------*/
extern void DWIN_PORT_SetDMAModule(DMA_Stream_TypeDef* dma);
extern void DWIN_PORT_SetUartModule(UART_HandleTypeDef* uart);
#endif // #DWIN_PORT_H