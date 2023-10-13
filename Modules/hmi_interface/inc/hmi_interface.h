#ifndef HMI_INTERFACE_H
#define HMI_INTERFACE_H

#include <stm32f405xx.h>
#include "proc_func.h"
#include "flash_cmsis.h"
#include "step_engine.h"
#include <string.h>
#include "dac_cmsis.h"
#include "config_DWIN.h"

#define MAX_PRGRMS_NUM								10
#define BASE_PROGRAM_ADDRESS					0x0800C000
#define STEPS_NUM 										3

#define MAX_VEL_PROG									500000

#define STAGE_NUM											( 3 )

/* MODBUS DEFINES BEGIN */
#define REG_INPUT_START 5002
#define REG_INPUT_NREGS 1

#define REG_HOLDING_START 0x01
#define REG_HOLDING_NREGS 30
/* MODBUS DEFINES END */


/* REGISTERS DEFINES BEGIN */

/* STEP ENGINE */								/* NUM REG */  /* NUM REG HMI */
#define STEP_ENGINE_ON_MC						( 1 )			 		/* 5002 */
#define STEP_ENGINE_VEL_MC					( 2 )			 		/* 5010 */
#define STEP_ENGINE_POS_MC					( 3 ) 		 		/* 5012 */
#define STEP_ENGINE_START_POS_MS		( 4 )			 		/* 5000 */
#define STEP_ENGINE_MOVE_RIGHT			( 5	)					/* 5056 */
#define STEP_ENGINE_MOVE_LEFT				( 21 )				/* 5054 */

/* PROGRAM */
#define NUM_EXE_PROGRAM							( 6 ) 		 	  /* 5014 */
#define LAUNCH_PROGRAM							( 7 )			 		/* 5050 */
#define SAVE_PROGRAM								( 8 )			 		/* 5052 */

#define STAGE_1_POS									( 9 )			 		/* 5032 */
#define STAGE_1_VEL									( 10 )		 		/* 5034 */
#define STAGE_1_DEL									( 11 )		 		/* 5036 */

#define STAGE_2_POS									( 12 )	 	 		/* 5038 */
#define STAGE_2_VEL									( 13 )		 		/* 5040 */
#define STAGE_2_DEL									( 14 )		 		/* 5042 */

#define STAGE_3_POS									( 15 ) 				/* 5044 */
#define STAGE_3_VEL									( 16 )		 		/* 5046 */
#define STAGE_3_DEL									( 17 )		 		/* 5048 */

#define TRANSFUSE_ANGLE							( 18 )		 		/* 5020 */
#define MIXING_TIME									( 19 )		 		/* 5022 */
#define VACUUM_TIME									( 20 )		 		/* 5024 */

/* REGISTERS DEFINES END */

#define START_POS_LOCALITY					( 48 )

typedef struct DEVICES {
	t_step_engine* step_engine;
} t_devices;

typedef struct CONTROL {
	uint16_t is_launch;
	uint16_t is_manual;
	float current_vel;
	int16_t current_pos;
	prog_dscrptr* programms;
	t_devices* dev;
	uint8_t exe_prog;
	uint8_t save_prog;
	uint8_t start_pos_step_engine;
} t_control;

uint8_t write_program(prog_dscrptr* prog, uint16_t num);
void read_program(prog_dscrptr* program, uint8_t num);
void move_SE(t_control* comtrl, uint8_t num);
void load_prog_FLASH(t_control* comtrl);
void munual_mode(t_control* comtrl);
void init_HMI(t_control* comtrl);
void refresh_prog_parameters_FLASH(t_control* comtrl);
void move_start_pos(t_control* comtrl);

void refresh_reg(t_control* comtrl, int* usRegBuf);

void search_home();

void eHMIPoll();

#endif //!HMI_INTERFACE_H
