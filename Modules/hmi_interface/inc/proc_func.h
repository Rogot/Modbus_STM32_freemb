#ifndef PROC_FUNC_H
#define PROC_FUNC_H

#include <stm32f405xx.h>
#include <stdio.h>

#define DATA_RX_MAX_SIZE								( 255 )

/*********** symbolic defines ***********/
#define NEW_LINE														( 0x0D )
#define RET_CAR															( 0x0A )

typedef enum
{
	STATE_READ_COMMAND, 				/*!< Read command for proccesing */
	STATE_EXECUTE_COMMAND, 			/*!< Execute command */
} eProcState;

typedef struct Program_descriptor {
	uint8_t usDataRx[DATA_RX_MAX_SIZE];
	uint8_t size;
	uint8_t state;
}prog_dscrptr;


void proccesing_HMI_request(const uint8_t * usRegBuf, uint8_t usLen);
uint8_t copy_to(const uint8_t* src, uint8_t* dst, uint8_t start_data_pos, uint8_t usLen);
void parser(const uint8_t* program, uint8_t usLen);
void split(const uint8_t* src, uint8_t sign, uint8_t* start_pos, uint8_t usLen);


#endif //! PROC_FUNC_H