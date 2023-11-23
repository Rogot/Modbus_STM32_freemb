#ifndef HMI_INTERFACE_H
#define HMI_INTERFACE_H

#include <stm32f405xx.h>

#include <string.h>

/* DEVISES */
#include "bldc_motors.h"
#include "vacum_sens.h"
#include "vacum_pump.h"
#include "step_engine.h"
#include "limit_switch.h"

/* DWIN UTILS */
#include "proc_func.h"
#include "config_DWIN.h"
#include "DWIN_addr_conv.h"

/* LIBS */
#include "flash_cmsis.h"
#include "ring_buf.h"

#define MAX_PRGRMS_NUM							( 10 )
#define BASE_PROGRAM_ADDRESS					0x0800C000
#define STEPS_NUM 								( 3 )

#define MAX_VEL_PROG							500000

#define STAGE_NUM								( 3 )

/* MODBUS DEFINES BEGIN */
#define REG_INPUT_START 						5002
#define REG_INPUT_NREGS 						( 1 )

#define REG_HOLDING_START 						0x01
#define REG_HOLDING_NREGS 						30
/* MODBUS DEFINES END */


/* REGISTERS DEFINES BEGIN */

/* STEP ENGINE */							/* NUM REG */				/* NUM REG HMI */
#define STEP_ENGINE_ON_MC						( 1 )			 		/* 5002 */
#define STEP_ENGINE_VEL_MC						( 2 )			 		/* 5010 */
#define STEP_ENGINE_CHOOSE						( 3 ) 		 			/* 5012 */
#define STEP_ENGINE_START_POS_MS				( 4 )			 		/* 5004 */
#define STEP_ENGINE_MOVE_RIGHT					( 5	)					/* 5056 */
#define STEP_ENGINE_MOVE_LEFT					( 21 )					/* 5054 */

/* PROGRAM */
#define NUM_EXE_PROGRAM							( 6 ) 			 	 	/* 5014 */
#define LAUNCH_PROGRAM							( 7 )			 		/* 5050 */
#define SAVE_PROGRAM							( 8 )			 		/* 5052 */

#define SET_HOME								( 9 )					/* 5016	*/
#define SEARCH_HOME								( 10 )					/* 5018	*/

#define TRANSFUSE_ANGLE							( 18 )		 			/* 5020 */
#define MIXING_TIME								( 19 )		 			/* 5022 */
#define VACUUM_TIME								( 20 )		 			/* 5024 */

#define CURV_MAX_NUM							( 8 )
/* REGISTERS DEFINES END */

typedef enum {
	CURV_IDLE,
	CURV_SEND_DATA,
}t_State_Curv;

typedef struct HMI_CURV {
	uint8_t num;
	float refresh_rate;
	t_State_Curv state;
	TIM_HandleTypeDef* tim;
}t_HMI_curve;

/* STRUCTS */
typedef struct DEVICES {
	t_step_engine* step_engine;
	t_bldc_engine* bldc_engine;
	t_vac_sen* vac_sensor;
	t_vac_pump* vac_pump;
} t_devices;

typedef struct CONTROL {
	uint16_t is_launch;
	uint16_t is_manual;
	float current_vel;
	int16_t current_pos;
	prog_dscrptr* programms;
	t_devices* dev;
	t_HMI_curve HMI_curves[CURV_MAX_NUM];
	uint8_t exe_prog;
	uint8_t save_prog;
	uint8_t start_pos_step_engine;
	t_ring_buf* queue;
} t_control;

/* FUNCTIONS */
typedef void( *peHMISendRequest ) ( uint8_t pucTrnAddress,
									const uint8_t * pucFrame,
									uint16_t * pusLength);

/* Flash */
uint8_t write_program(prog_dscrptr* prog, uint16_t num);
void read_program(prog_dscrptr* program, uint8_t num);
void load_prog_FLASH(t_control* contrl);
void refresh_prog_parameters_FLASH(t_control* contrl);

/* Step Engine */
void move_SE(t_control* contrl, uint8_t num);
void move_SE_to(t_control* contrl, uint8_t num);
void munual_mode(t_control* contrl, uint8_t num);
void move_start_pos(t_control* contrl, uint8_t num);
void search_home();

/* BLDC Engine*/
void control_BLCD(t_control* contrl);

/* Vacuum sensor */
void read_data_vac_sens(t_control* contrl);

void refresh_reg(t_control* contrl, int* usRegBuf);
void send_request(uint8_t ucSlaveAddress, uint8_t* data, uint8_t cmd, uint8_t len);

/* Vacuum pump */
eVacSetPoint is_setpoint(t_vac_pump* vPump, t_vac_sen* vSen);

/* HMI functions */
void eHMIPoll();
void init_HMI(t_control* contrl);
void send_data_curve(uint8_t ucSlaveAddress, uint16_t data, uint8_t curve);
void refresh_curv( t_control* contrl );

#endif //!HMI_INTERFACE_H
