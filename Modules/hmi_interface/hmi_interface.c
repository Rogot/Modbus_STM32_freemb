#include "hmi_interface.h"
#include <stdlib.h>

t_hmi_reg programs[MAX_PRGRMS_NUM];
t_control ctrl;
extern t_step_engine step_engine;
t_hmi_reg* current_program;

/*
* @bref: prog - programm parameters 
				 num - number of saved programs
*/

uint8_t write_program(t_hmi_reg* prog, uint16_t num){

	uint8_t struct_size = sizeof(t_hmi_reg);
	
	uint8_t* data_ptr = malloc(num);
	
	CMSIS_flash_allow_access();
	CMSIS_internal_flash_erase(FLASH_SNB_SEC_3);
	
	data_ptr = (uint8_t*)&(*prog);
	CMSIS_internal_flash_write(data_ptr, BASE_PROGRAM_ADDRESS, num * sizeof(t_hmi_reg));
	
	//t_hmi_reg* temp[2];
	//for (uint8_t i = 0; i < size; i++) {
	//	temp[i] = (t_hmi_reg*)&(*(data_ptr[i])); 
	//}
	//free(temp);
	free(data_ptr);
	
	return 1;
}

/*
* @bref: program - programm parameters;
				 num - number of the program read
*/

void read_program(t_hmi_reg* program, uint8_t num) {
	t_hmi_reg* temp_program;
	uint32_t* temp = malloc(sizeof(t_hmi_reg));
	
	for (uint16_t i = 0; i < sizeof(t_hmi_reg) / sizeof(uint32_t); i++){
		temp[i] = CMSIS_internal_flash_read(BASE_PROGRAM_ADDRESS
											+ (sizeof(t_hmi_reg) * num) + sizeof(uint32_t) * i);
	}
	
	temp_program = (t_hmi_reg*)&(*(temp));
	
	memcpy(program, temp_program, (sizeof(t_hmi_reg)));
}

/*
* @bref: comtrl - all parameters for controlling by HMI
*/
void execute_program(t_control* comtrl) {
	
	#if STEP_ENGINE_ENABLE
		#if STEP_ENGINE_TEST_ENABLE
			TIM2->CCR1=TIM2->CNT+step_engine.dir*step_engine.speedupCNT;
			HAL_DMA_Start_IT(htim3.hdma[TIM_DMA_ID_UPDATE], 
				(uint32_t)step_engine.speedupbuf, 
				(uint32_t)&TIM3->ARR, 
				step_engine.speedupCNT);
		  __HAL_TIM_ENABLE_DMA(&htim3, TIM_DMA_UPDATE);
			step_engine.mode=SPEEDUP;
			HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_3);
		#endif
			
		#if !STEP_ENGINE_TEST_ENABLE
			//read_program(comtrl->programms, comtrl->exe_prog);
			for (uint8_t i = 0; i < STAGE_NUM; i++) {
				move_step_engine(comtrl->dev->step_engine, 
					comtrl->programms[comtrl->exe_prog].moving[i] * 100, (float)(
					(float)comtrl->programms[comtrl->exe_prog].vel[i] / (float)2550));
				while(comtrl->dev->step_engine->mode != STOP);
			}	
		#endif
	#endif
}

/*
* @bref: comtrl - all parameters for controlling by HMI
*/

void refresh_prog_parameters_FLASH(t_control* comtrl) {
	for (uint8_t i = 0; i < MAX_PRGRMS_NUM; i++) {
		write_program(comtrl->programms, i);
	}
	for (uint8_t i = 0; i < MAX_PRGRMS_NUM; i++) {
		read_program(comtrl->programms, i);
	} 
}
	
void munual_mode(t_control* comtrl) {
	#if !STEP_ENGINE_TEST_ENABLE
	//if (comtrl->current_vel >= 0x04 && comtrl->current_pos >= 1) {
		comtrl->dev->step_engine->manual_mode = 0x01;
		if (comtrl->dev->step_engine->manual_move_left) {
			move_step_engine(comtrl->dev->step_engine, 0, (float)(
										(float)comtrl->current_vel / (float)2550));
		}
		else if (comtrl->dev->step_engine->manual_move_right) {
			move_step_engine(comtrl->dev->step_engine, 0, (float)(
										(float)comtrl->current_vel / (float)2550));
		}
	//}
	#endif
}

void move_start_pos(t_control* comtrl) {
	int32_t start_pos = 2147483647;
	int32_t cur_pos = TIM2->CCR1;
	
	int32_t res = (int16_t)start_pos - (int16_t )cur_pos;
	
	if (res > 48 || res < -48) {
	
	move_step_engine(comtrl->dev->step_engine, (int16_t )(res), (float)(
									(float)comtrl->current_vel / (float)2550));
	}
}


void eHMIPoll(t_control* comtrl, int* usRegBuf) {
	refresh_reg(comtrl, usRegBuf);
	if( comtrl->is_manual == 0x01 && comtrl->is_launch == 0x00
				&& comtrl->dev->step_engine->mode == STOP){
		comtrl->is_manual = 0x00;
		comtrl->dev->step_engine->start_pose_mode = 0x00;			
		munual_mode(comtrl);		
	}
	if(comtrl->is_manual == 0x00 && comtrl->is_launch == 0x01 
			&& comtrl->dev->step_engine->mode == STOP){
		execute_program(comtrl);
		//comtrl->is_launch = 0x00; 
		usRegBuf[LAUNCH_PROGRAM] = 0x0;
	}
	if (comtrl->save_prog != 0) {
		refresh_prog_parameters_FLASH(comtrl);
	}
	if (comtrl->start_pos_step_engine == 0x01 
			&& comtrl->dev->step_engine->mode == STOP) {
		comtrl->dev->step_engine->start_pose_mode = 0x01;	
		move_start_pos(comtrl);
		usRegBuf[STEP_ENGINE_START_POS_MS] = 0x0;	
	}
}

/*
* @bref: comtrl - all parameters for controlling by HMI
				 usRegBuf - MODBUS buffer pointer
*/
void refresh_reg(t_control* comtrl, int* usRegBuf) {
	
	if (comtrl->is_launch == 0x01){
		usRegBuf[LAUNCH_PROGRAM] = 0x0;
	}
	uint8_t num_prog = usRegBuf[NUM_EXE_PROGRAM] - 1;
	
	comtrl->exe_prog = num_prog;
	comtrl->save_prog = usRegBuf[SAVE_PROGRAM];
	comtrl->start_pos_step_engine = usRegBuf[STEP_ENGINE_START_POS_MS];
	
	comtrl->is_launch = usRegBuf[LAUNCH_PROGRAM];
	comtrl->is_manual = usRegBuf[STEP_ENGINE_ON_MC];
	
	comtrl->current_vel = usRegBuf[STEP_ENGINE_VEL_MC];
	comtrl->current_pos = usRegBuf[STEP_ENGINE_POS_MC];
	
	comtrl->programms[num_prog].vel[0] = usRegBuf[STAGE_1_VEL];
	comtrl->programms[num_prog].vel[1] = usRegBuf[STAGE_2_VEL];
	comtrl->programms[num_prog].vel[2] = usRegBuf[STAGE_3_VEL];

	comtrl->programms[num_prog].moving[0] = (int16_t)usRegBuf[STAGE_1_POS];
	comtrl->programms[num_prog].moving[1] = (int16_t)usRegBuf[STAGE_2_POS];
	comtrl->programms[num_prog].moving[2] = (int16_t)usRegBuf[STAGE_3_POS];
	
	comtrl->dev->step_engine->manual_move_left = (uint8_t)usRegBuf[STEP_ENGINE_MOVE_LEFT];
	comtrl->dev->step_engine->manual_move_right = (uint8_t)usRegBuf[STEP_ENGINE_MOVE_RIGHT];
	
}

/*
* @bref: comtrl - all parameters for controlling by HMI
*/
void init_HMI(t_control* comtrl) {
	comtrl->programms[0].mixing_speed = 0x33;
	comtrl->programms[0].mixing_time= 0x34;
	comtrl->programms[0].vacuum_time = 0x35;
	comtrl->programms[0].air_pubping_lvl = 0x36;
	comtrl->programms[0].moving[0] = 100;
	comtrl->programms[0].vel[0] = 20;
	comtrl->programms[0].moving[1] = 200;
	comtrl->programms[0].vel[1] = 100;
	comtrl->programms[0].moving[2] = -100;
	comtrl->programms[0].vel[2] = 20;
	
	comtrl->programms[1].mixing_speed = 0x23;
	comtrl->programms[1].mixing_time= 0x24;
	comtrl->programms[1].vacuum_time = 0x25;
	comtrl->programms[1].air_pubping_lvl = 0x26;
	
	//write_program(programs, 1);
	
	read_program(current_program, 0);
}