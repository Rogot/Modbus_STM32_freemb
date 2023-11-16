#include "hmi_interface.h"
#include <stdlib.h>

prog_dscrptr pr_dscr;
t_control ctrl;
extern uint8_t is_start_pos;
extern t_addr_conv PLC_addr[PLC_ADDR_MAX];
prog_dscrptr* current_program;

static uint8_t usLenth = 0;

uint16_t reqiest_data[10];
peHMISendRequest peHMISendRequestCur;


/*
* @bref: inital function for HMI
* @param (contrl) - all parameters for controlling by HMI
*/
void init_HMI(t_control* contrl) {



	for (uint8_t i = 0; i < CURV_MAX_NUM; i++) {
		if (contrl->HMI_curves[i].tim != NULL) {
			contrl->HMI_curves[i].tim->Instance->CR1 &= ~TIM_CR1_CEN;

			contrl->HMI_curves[i].tim->Instance->ARR = PLC_RATE /
						(contrl->HMI_curves[i].tim->Instance->PSC * contrl->HMI_curves[i].refresh_rate);

//			contrl->HMI_curves[i].tim->Instance->ARR = 60000;
			contrl->HMI_curves[i].tim->Instance->CR1 |= TIM_CR1_CEN;

			HAL_TIM_Base_Start_IT(contrl->HMI_curves[i].tim);

			contrl->HMI_curves[i].state = CURV_IDLE;
		}
	}

	contrl->exe_prog = 0x01;

	read_program(current_program, 0);
}

/*
* @bref: send a request to other device
* @param (ucSlaveAddress) - slave address
* @param (data) - data for sending
* @param (cmd) - read/write mode
* @param (len) - data lenth
*/
void send_request(uint8_t ucSlaveAddress, uint8_t* data, uint8_t cmd, uint8_t len) {

	static uint8_t	 ucHIMFrame_TX[50];
	static uint8_t 	 usLength;
	uint8_t addr = data[0] - 1;
	uint8_t i = 0;

	if (len % 2 != 0) {
		len++;
	}
	data[0] = PLC_addr[addr].HMI_addr >> 8;
	data[1] = PLC_addr[addr].HMI_addr & 0xFF;
	usLength = 0;
	ucHIMFrame_TX[usLength++] = 0x5A;
	ucHIMFrame_TX[usLength++] = ucSlaveAddress; 			/* START BIT */
	ucHIMFrame_TX[usLength++] = (uint8_t)(len) + 2; 	    /* REQUEST SIZE (WORDS) */
	ucHIMFrame_TX[usLength++] = cmd;						/* READ/WRITE */
	ucHIMFrame_TX[usLength++] = data[i++];					/* ADDR */
	ucHIMFrame_TX[usLength++] = data[i++];
	ucHIMFrame_TX[usLength++] = (uint8_t)len / 2 - 1;		/* DATA READ (WORDS) */
	ucHIMFrame_TX[usLength++] = data[i++];					/* DATA */
	ucHIMFrame_TX[usLength++] = data[i++];
	//while (i < len) {
	//	ucHIMFrame_TX[usLength++] = data[i++]; 				/* FILL FRAME BY DATA */
	//}

	peHMISendRequestCur(ucSlaveAddress, ucHIMFrame_TX, usLength);
}

/*
* @bref: write prigram from FLASH
* @param (prog) - programm parameters
* @param (num) - number of saved programs
*/

uint8_t write_program(prog_dscrptr* prog, uint16_t num){

	uint8_t struct_size = sizeof(prog_dscrptr);
	
	uint8_t* data_ptr = malloc(num);
	
	CMSIS_flash_allow_access();
	CMSIS_internal_flash_erase(FLASH_SNB_SEC_3);
	
	data_ptr = (uint8_t*)&(*prog);
	CMSIS_internal_flash_write(data_ptr, BASE_PROGRAM_ADDRESS, num * sizeof(prog_dscrptr));
	
	//t_hmi_reg* temp[2];
	//for (uint8_t i = 0; i < size; i++) {
	//	temp[i] = (t_hmi_reg*)&(*(data_ptr[i])); 
	//}
	//free(temp);
	free(data_ptr);
	
	return 1;
}

/*
* @bref: read prigram from FLASH
* @param (program) - programm parameters;
* @param (num) - number of the program read
*/

void read_program(prog_dscrptr* program, uint8_t num) {
	prog_dscrptr* temp_program;
	uint32_t* temp = malloc(sizeof(prog_dscrptr));
	
	for (uint16_t i = 0; i < sizeof(prog_dscrptr) / sizeof(uint32_t); i++){
		temp[i] = CMSIS_internal_flash_read(BASE_PROGRAM_ADDRESS
											+ (sizeof(prog_dscrptr) * num) + sizeof(uint32_t) * i);
	}
	
	temp_program = (prog_dscrptr*)&(*(temp));
	
	memcpy(&program[num], temp_program, (sizeof(prog_dscrptr)));
}

/*
* @bref: execute storage program
* @param (contrl) - all parameters for controlling by HMI
* @param (num) - choose a engine
*/
void move_SE(t_control* contrl, uint8_t num) {
	
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
		if (contrl->is_manual == 0x00 && contrl->is_launch == 0x01
		&& contrl->dev->step_engine->mode == STOP) {
			contrl->programms->state = STATE_BUSY_COMMAND;
			if (contrl->programms->par2 > MAX_VEL_PROG) {
				contrl->programms->par2 = MAX_VEL_PROG;
			}

			move_step_engine(&contrl->dev->step_engine[num],
					calc_steps(contrl->programms->par1) * RATIO_GEARBOX,
					(float) ((float) contrl->programms->par2 * RATIO_GEARBOX / BASE_FREQ
							/ ANFLE_ONE_STEP * 2));
		}
		/* Legacy code */
		/*
		for (uint8_t i = 0; i < STAGE_NUM; i++) {
			if (comtrl->programms[comtrl->exe_prog].vel[i] != 0x00) {
				if (comtrl->programms[comtrl->exe_prog].vel[i] > MAX_VEL_PROG) {
					comtrl->programms[comtrl->exe_prog].vel[i] = MAX_VEL_PROG;
				}
				move_step_engine(&comtrl->dev->step_engine[i],
						calc_steps(comtrl->programms[comtrl->exe_prog].moving[i]),
						(float) ((float) comtrl->programms[comtrl->exe_prog].vel[i]
								/ BASE_FREQ / ANFLE_ONE_STEP * 2));
				while (comtrl->dev->step_engine->mode != STOP)
					;
			}
		}
		*/
		#endif
	#endif
}

/*
* @bref: execute storage program
* @param (contrl) - all parameters for controlling by HMI
* @param (num) - num of engine (choose an engine which control)
*/
void move_SE_to(t_control* contrl, uint8_t num) {
#if !STEP_ENGINE_TEST_ENABLE

	if (contrl->is_manual == 0x00 && contrl->is_launch >= 0x01
			&& contrl->dev->step_engine->mode == STOP) {
		contrl->programms->state = STATE_BUSY_COMMAND;
		if (contrl->programms->par2 > MAX_VEL_PROG) {
			contrl->programms->par2 = MAX_VEL_PROG;
		} else if (contrl->programms->par2 > 0x0) {

			uint32_t start_pos = START_POS_VALUE + calc_steps(contrl->programms->par1) * RATIO_GEARBOX;
			uint32_t cur_pos = contrl->dev->step_engine[num].
							engine_TIM_slave->Instance->CCR1;

			int32_t res = (int16_t)start_pos - (int16_t )cur_pos;

			if (res > 1 || res < -1) {
				move_step_engine(&contrl->dev->step_engine[num], (int16_t )(res), (float)(
									(float)contrl->programms->par2 * RATIO_GEARBOX / BASE_FREQ
									/ ANFLE_ONE_STEP * 2));
			} else {
				contrl->programms->state = STATE_READ_COMMAND;
			}
		} else {
			contrl->programms->state = STATE_READ_COMMAND;
		}
	}
#endif
}

/*
* @bref: control BLDC motor
*	@param (contrl) - all parameters for controlling by HMI
*/
void control_BLCD(t_control* contrl) {
	if (contrl->is_manual == 0x00 && contrl->is_launch >= 0x01) {

		contrl->dev->bldc_engine->power = contrl->programms->par1;

		if (contrl->dev->bldc_engine->power > 0) {
			start_BLDC(contrl->dev->bldc_engine, contrl->dev->bldc_engine->power);
		} else {
			stop_BLDC(contrl->dev->bldc_engine);
		}
		contrl->programms->state = STATE_READ_COMMAND;
	}
}

/*
* @bref: save struct data in FLASH
*	@param (contrl) - all parameters for controlling by HMI
*/

void refresh_prog_parameters_FLASH(t_control* contrl) {
	for (uint8_t i = 0; i < MAX_PRGRMS_NUM; i++) {
		write_program(contrl->programms, i);
	}
	for (uint8_t i = 0; i < MAX_PRGRMS_NUM; i++) {
		read_program(contrl->programms, i);
	} 
}

/*
* @bref: only load program from FLASH without write
*	@param (contrl) - all parameters for controlling by HMI
*/

void load_prog_FLASH(t_control* contrl) {
	for (uint8_t i = 0; i < MAX_PRGRMS_NUM; i++) {
		read_program(contrl->programms, i);
	}
}

/*
* @bref: manual mode control function
* @param (contrl) - all parameters for controlling by HMI
* @param (num) - num of engine (choose an engine which control)
*/
void munual_mode(t_control* contrl, uint8_t num) {
	contrl->dev->step_engine->manual_mode = 0x01;
	if (contrl->current_vel != 0x00) {
		if (contrl->dev->step_engine->manual_move_left) {
			move_step_engine(&contrl->dev->step_engine[num], 2 * RATIO_GEARBOX,
					(float) ((float) contrl->current_vel / BASE_FREQ
					/ ANFLE_ONE_STEP * 2));
		}
		else if (contrl->dev->step_engine->manual_move_right) {
			move_step_engine(&contrl->dev->step_engine[num], -2 * RATIO_GEARBOX,
					(float) ((float) contrl->current_vel / BASE_FREQ
					/ ANFLE_ONE_STEP * 2));
		}
	}
}

/*
* @bref: "home" position of coordinate
* @param (contrl) - all parameters for controlling by HMI
* @param (num) - num of engine (choose an engine which control)
*/
void move_start_pos(t_control* contrl, uint8_t num) {
	int32_t start_pos = START_POS_VALUE;
	int32_t cur_pos = contrl->dev->step_engine->engine_TIM_slave->Instance->CCR1;

	int32_t res = (int16_t)start_pos - (int16_t )cur_pos;

	if ((res > 1 || res < -1)
				&& contrl->current_vel != 0x0) {
		move_step_engine(&contrl->dev->step_engine[num], (int16_t )(res), (float)(
							(float)contrl->current_vel* RATIO_GEARBOX / BASE_FREQ
							/ ANFLE_ONE_STEP * 2));
	} else {

		t_queue_dicr discr;

		contrl->programms->state = STATE_SAND_REQUEST;

		discr.state = HOME_IS_REACHED;
		ringBuf_put(discr, contrl->queue);
	}
}

/*
* 	@bref: main loop for control HMI request
*	@param (contrl) - all parameters for controlling by HMI
*	@param (usRegBuf) - MODBUS array
*/
void eHMIPoll(t_control* contrl, int* usRegBuf) {

	static eProcState pState;

	t_queue_dicr discr;

	refresh_reg(contrl, usRegBuf);
	
#if HAL_ADC_MODULE_ENABLED && VACUUM_SENSOR_ENABLE
	contrl->dev->vac_sensor->cur_pres = get_value(contrl->dev->vac_sensor->adc);

	switch (is_setpoint(contrl->dev->vac_pump, contrl->dev->vac_sensor)) {
	case VAC_SETPOINT_REACHED:

		//TO DO: make upperBoard and lower measurement boundary

		break;
	case VAC_SETPOINT_LOWER:

		//TO DO: pump ON

		break;
	case VAC_SETPOINT_UPPER:

		//TO DO: pump ON

		break;
	}
#endif

	switch (contrl->programms->state) {
	case STATE_IDLE_COMMAND:
		contrl->programms->read_cnt = 0;
		break;

	case STATE_READ_COMMAND:

		pState = proccesing_HMI_request(contrl->programms);

		if (pState == STATE_END_PROGRAMM) {
			contrl->programms->state = STATE_SAND_REQUEST;
			discr.state = END_PROGRAMM;
			ringBuf_put(discr, contrl->queue);
		}
		break;

	case STATE_EXECUTE_COMMAND:
		if (strcmp(contrl->programms->com_dscr.name, "AX1") == 0) {
		#if STEP_ENGINE_ENABLE
			move_SE_to(contrl, 0);
		#endif
		}
		else if (strcmp(contrl->programms->com_dscr.name, "AX2") == 0) {
		#if STEP_ENGINE_ENABLE
			move_SE_to(contrl, 1);
		#endif
		}
		else if (strcmp(contrl->programms->com_dscr.name, "MIX") == 0) {
		#if BLDC_ENGINE_ENABLE
			control_BLCD(contrl);
		#endif
		}
		else if (strcmp(contrl->programms->com_dscr.name, "VS") == 0) {
			//pump_ON(&contrl->is_pump_ON);
		}

		break;

	case STATE_SAND_REQUEST:

		usLenth = 0;

		discr = ringBuf_pop(contrl->queue);

		if (discr.state != NONE_REQUEST) {
			switch(discr.state) {

			case HOME_IS_REACHED:
				contrl->start_pos_step_engine = 0x0;
				usRegBuf[STEP_ENGINE_START_POS_MS] = 0x0;
				discr.reg_num = STEP_ENGINE_START_POS_MS;
				break;
			case END_PROGRAMM:
				contrl->is_launch = 0x0;
				usRegBuf[LAUNCH_PROGRAM] = 0x0;
				discr.reg_num = LAUNCH_PROGRAM;
				break;
			}
		}

		/* Request formation */
		reqiest_data[usLenth++] = discr.reg_num;
		usLenth++;
		reqiest_data[usLenth++] = usRegBuf[discr.reg_num] >> 8;
		reqiest_data[usLenth++] = usRegBuf[discr.reg_num] & 0xFF;

		send_request(0xA5, (uint8_t*)reqiest_data, 0x82, usLenth);

		discr.state = NONE_REQUEST;

		if (discr.state == NONE_REQUEST && contrl->is_launch != 0) {
			contrl->programms->state = STATE_READ_COMMAND;
		} else if (discr.state == NONE_REQUEST && contrl->is_launch == 0) {
			contrl->programms->state = STATE_IDLE_COMMAND;
		}
		break;
	}

	#if STEP_ENGINE_ENABLE
	if( contrl->is_manual == 0x01 && contrl->is_launch == 0x00
				&& contrl->dev->step_engine->mode == STOP){
		contrl->is_manual = 0x00;
		contrl->dev->step_engine->start_pose_mode = 0x00;
		if (usRegBuf[STEP_ENGINE_CHOOSE] > 0) {
			munual_mode(contrl, usRegBuf[STEP_ENGINE_CHOOSE] - 1);
		}
	}
	
	if ((contrl->start_pos_step_engine && 0xFF) == 0x01
			&& contrl->dev->step_engine->mode == STOP) {
		contrl->dev->step_engine->start_pose_mode = 0x01;
		contrl->programms->state = STATE_BUSY_COMMAND;
		if (usRegBuf[STEP_ENGINE_CHOOSE] > 0) {
			move_start_pos(contrl, usRegBuf[STEP_ENGINE_CHOOSE] - 1);
		}
	}
	#endif

	/* Sending current vacuum value */
	if (contrl->HMI_curves[0].state == CURV_SEND_DATA) {
		contrl->HMI_curves[0].state = CURV_IDLE;
		send_data_curve(0xA5, ctrl.dev->vac_sensor->cur_pres, 0x00);
	}

	#if FLASH_ENABLE
	if (contrl->save_prog != 0) {
		usRegBuf[SAVE_PROGRAM] = 0x0;
		refresh_prog_parameters_FLASH(contrl);
	}
	#endif
}

/*
* @bref: function for refresh struct parameters for HMI
* @param (contrl) - all parameters for controlling by HMI
* @param (usRegBuf) - MODBUS buffer pointer
*/
void refresh_reg(t_control* contrl, int* usRegBuf) {
	
	contrl->save_prog = usRegBuf[SAVE_PROGRAM];
	contrl->start_pos_step_engine = usRegBuf[STEP_ENGINE_START_POS_MS];
	
	contrl->is_launch = usRegBuf[LAUNCH_PROGRAM];
	contrl->is_manual = usRegBuf[STEP_ENGINE_ON_MC];
	
	contrl->current_vel = usRegBuf[STEP_ENGINE_VEL_MC];

	contrl->exe_prog = usRegBuf[NUM_EXE_PROGRAM] - 1;
	
	contrl->dev->step_engine->manual_move_left = (uint8_t)usRegBuf[STEP_ENGINE_MOVE_LEFT];
	contrl->dev->step_engine->manual_move_right = (uint8_t)usRegBuf[STEP_ENGINE_MOVE_RIGHT];
}

/*
* @bref: search start position - "0" via end cap
* @param (contrl) - all parameters for controlling by HMI
*/
void search_home(t_control* contrl) {
	while(!is_start_pos) {
		move_step_engine(contrl->dev->step_engine,
					2, (float)((float)50 / (float)2550));
	}
	contrl->dev->step_engine->engine_TIM_slave->Instance->CCR1 = contrl->dev->step_engine->engine_TIM_slave->Instance->CNT
								+ contrl->dev->step_engine->dir * contrl->dev->step_engine->slowdownCNT;
	contrl->dev->step_engine->mode = SLOWDOWN;
				
	HAL_DMA_Start_IT(contrl->dev->step_engine->engine_TIM_master->hdma[TIM_DMA_ID_UPDATE],
				(uint32_t)(contrl->dev->step_engine->slowdownbuf
								+ contrl->dev->step_engine->accel_size - contrl->dev->step_engine->slowdownCNT),
				(uint32_t)&contrl->dev->step_engine->engine_TIM_master->Instance->ARR, contrl->dev->step_engine->slowdownCNT);
				
	__HAL_TIM_ENABLE_DMA(contrl->dev->step_engine->engine_TIM_master, TIM_DMA_UPDATE);
	
}

/*
* @bref: checking whether the setpoint level has been reached
* @param (vPump) - vacuum pump struct
* @param (vSen) - vacuum sensor struct
* @res - result of checking
*/
eVacSetPoint is_setpoint(t_vac_pump* vPump, t_vac_sen* vSen) {

	if (vSen->cur_pres == vPump->setpoint) {
		return VAC_SETPOINT_REACHED;
	} else if (vSen->cur_pres > vPump->setpoint) {
		return VAC_SETPOINT_LOWER;
	} else if (vSen->cur_pres < vPump->setpoint) {
		return VAC_SETPOINT_UPPER;
	}

}


/* ON/OFF Vacuum pump */
void pump_ON( t_vac_pump* v_pump ) {

}
void pump_OFF( t_vac_pump* v_pump ) {

}

extern UART_HandleTypeDef huart1;

void send_data_curve(uint8_t ucSlaveAddress, uint16_t data, uint8_t curve) {
	static uint8_t mdata[14] = { 0x00 };

	uint8_t 	usLength = 0;
	uint8_t 	low_byte = data;
	data = data >> 8;
	uint8_t 	high_byte = data;

	mdata[usLength++] = 0x5A;
	mdata[usLength++] = ucSlaveAddress;
	mdata[usLength++] = 0x0B;
	mdata[usLength++] = 0x82;
	mdata[usLength++] = 0x03;
	mdata[usLength++] = 0x10;
	mdata[usLength++] = 0x5A;
	mdata[usLength++] = 0xA5;
	mdata[usLength++] = 0x01;
	mdata[usLength++] = 0x00;
	mdata[usLength++] = curve;
	mdata[usLength++] = 0x01;
	mdata[usLength++] = high_byte;
	mdata[usLength++] = low_byte;

	peHMISendRequestCur(ucSlaveAddress, mdata, usLength);
}

/****** IRQ Handlers ******/

/**
  * @brief This function handles TIM1 trigger and commutation interrupts and TIM11 global interrupt.
  */
void TIM7_IRQHandler(void)
{
	ctrl.HMI_curves[0].state = CURV_SEND_DATA;
	HAL_TIM_IRQHandler(ctrl.HMI_curves[0].tim);
}

/**
  * @brief This function handles TIM6 global interrupt.
  */
void TIM6_DAC_IRQHandler(void)
{
	ctrl.dev->bldc_engine->dac->tim->SR &= ~TIM_SR_UIF;
	ctrl.dev->bldc_engine->dac->dac_type->DHR12R1 = ctrl.dev->bldc_engine->power;
}


void EXTI9_5_IRQHandler(void){
	//pump_OFF(&ctrl.is_pump_ON);
	EXTI->PR |= EXTI_PR_PR7;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {

	int value;

	if (hadc->Instance == ADC1) {
		ctrl.dev->vac_sensor->cur_pres = HAL_ADC_GetValue(hadc);
		//value = HAL_ADC_GetValue(hadc);
		//dac.dac_type->DHR12R1 = value;
		//ctrl.dev->bldc_engine->power = ctrl.dev->vac_sensor->cur_pres;

		//dac.dac_type->DHR12R1 = ctrl.dev->vac_sensor->cur_pres;
	}
}
