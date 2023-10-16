#ifndef PROC_FUNC_H
#define PROC_FUNC_H

#include <stm32f405xx.h>
#include <stdio.h>
#include "utils.h"

#define DATA_RX_MAX_SIZE								( 255 )
#define MAX_SIZE_COMMAND								( 15 )
#define MAX_SIZE_PARAMETR								( 10 )

/*********** symbolic defines ***********/
#define TWO_BYTE_CNTRL_SIGN									( 1 )
#define NEW_LINE														( 0x0D )
#define RET_CAR															( 0x0A )

typedef enum
{
	STATE_IDLE_COMMAND = 0, 						/*!< IDLE rescive command from serial port */
	STATE_READ_COMMAND = 1, 				/*!< Read command for proccesing */
	STATE_EXECUTE_COMMAND = 2, 				/*!< Execute command */
	STATE_BUSY_COMMAND = 3, 				/*!< Busy */
} eProcState;

typedef struct COMMAND_DISCR 
{
	uint8_t name[4];
	uint8_t p1[MAX_SIZE_PARAMETR];
	uint8_t p2[MAX_SIZE_PARAMETR];
	uint8_t p3[MAX_SIZE_PARAMETR];
} command_dscrptr;

typedef struct PROGRAM_DISCCR 
{
	uint8_t usDataRx[DATA_RX_MAX_SIZE];		/* Buffer with RX data from serial port */
	command_dscrptr com_dscr;				/* symbolic command parameters */
	uint8_t prog_name[MAX_SIZE_PARAMETR];
	int16_t par1;  							/* numeric parameters */
	int16_t par2;							/* numeric parameters */
	int16_t par3;							/* numeric parameters */
	uint8_t read_cnt;
	uint8_t size;
	eProcState state;
} prog_dscrptr;

void proc_funct_init(prog_dscrptr* pd);
void proccesing_HMI_request(prog_dscrptr* pd);
uint8_t copy_to(const uint8_t* src, uint8_t* dst, uint8_t start_data_pos, uint8_t usLen);
eProcState parser(const uint8_t* program, command_dscrptr* com_dscr, uint8_t* start_pos, uint8_t usLen);

#if TWO_BYTE_CNTRL_SIGN
void split_d(const uint8_t* src, uint8_t* cmd, 
						uint8_t sign1, uint8_t sign2, uint8_t* start_pos, uint8_t usLen);
#endif

#if !TWO_BYTE_CNTRL_SIGN
void split(const uint8_t* src, uint8_t* cmd,
						uint8_t sign, uint8_t* start_pos, uint8_t usLen);
#endif

#endif //! PROC_FUNC_H
