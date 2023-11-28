#include "hmi_interface.h"
#include <stdlib.h>

t_control ctrl;
extern uint8_t is_start_pos;
extern t_addr_conv PLC_addr[PLC_ADDR_MAX];

static uint8_t usLenth = 0;

uint16_t reqiest_data[10];
peHMISendRequest peHMISendRequestCur;


/*
* @bref: inital function for HMI
* @param (contrl) - all parameters for controlling by HMI
* @param (usRegBuf) - registers buffer
*/
void init_HMI(t_control* contrl, int* usRegBuf) {


	/* Initialization timer for "DL" command via HMI */
	RCC->APB2ENR |= RCC_APB2ENR_TIM9EN; /* Enable TIM9 */

	TIM9->CR1 &= ~TIM_CR1_CEN;
	TIM9->SR &= ~TIM_SR_UIF;
	TIM9->CR1 &= ~TIM_CR1_CEN;

	TIM9->DIER |= TIM_DIER_UIE; /* Update interrupt enable */
	TIM9->PSC = DELAY_PSC_BASE; /* 72 MHz / 36000 = 2 kHz */
	TIM9->ARR = DELAY_ARR_BASE /* 2000 Hz / 2 = 1000 Hz = 1 ms */;
	NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);

	for (uint8_t i = 0; i < CURV_MAX_NUM; i++) {
		if (contrl->HMI_curves[i].tim != NULL) {
			contrl->HMI_curves[i].tim->Instance->CR1 &= ~TIM_CR1_CEN;

			contrl->HMI_curves[i].tim->Instance->ARR = PLC_RATE /
						(contrl->HMI_curves[i].tim->Instance->PSC * contrl->HMI_curves[i].refresh_rate);

			contrl->HMI_curves[i].tim->Instance->CR1 |= TIM_CR1_CEN;

			HAL_TIM_Base_Start_IT(contrl->HMI_curves[i].tim);

			contrl->HMI_curves[i].state = CURV_IDLE;
		}
	}


	usRegBuf[NUM_EXE_PROGRAM] = 1;
	contrl->exe_prog = 0x01;

	CMSIS_flash_allow_access();

	load_prog_FLASH(contrl);

	for (uint8_t i = 0; i < MAX_PRGRMS_NUM; i++) {
		contrl->programms[i].state = STATE_IDLE_COMMAND;
	}
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
	
//	uint8_t* data_ptr = malloc(num);
	uint8_t* data_ptr;
	
//	CMSIS_flash_allow_access();
//	CMSIS_internal_flash_erase(FLASH_SNB_SEC_3);
	
	data_ptr = (uint8_t*)&(*prog);
	CMSIS_internal_flash_write(data_ptr, BASE_PROGRAM_ADDRESS, num * sizeof(prog_dscrptr));
	
	//t_hmi_reg* temp[2];
	//for (uint8_t i = 0; i < size; i++) {
	//	temp[i] = (t_hmi_reg*)&(*(data_ptr[i])); 
	//}
	//free(temp);
	
	return 1;
}

/*
* @bref: read prigram from FLASH
* @param (program) - programm parameters;
* @param (num) - number of the program read
*/

void read_program(prog_dscrptr* program, uint8_t num) {
	prog_dscrptr* temp_program;
	uint8_t* temp = malloc(sizeof(prog_dscrptr));
	
	for (uint16_t i = 0; i < sizeof(prog_dscrptr) / sizeof(uint8_t); i++){
		temp[i] = CMSIS_internal_flash_read(BASE_PROGRAM_ADDRESS
											+ (sizeof(prog_dscrptr) * num) + sizeof(uint8_t) * i);
	}
	
	temp_program = (prog_dscrptr*)&(*(temp));
	
	memcpy(&program[num], temp_program, (sizeof(prog_dscrptr)));
	free(temp);
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
			contrl->programms[contrl->exe_prog].state = STATE_BUSY_COMMAND;
			if (contrl->programms[contrl->exe_prog].par2 > MAX_VEL_PROG) {
				contrl->programms[contrl->exe_prog].par2 = MAX_VEL_PROG;
			}

			move_step_engine(&contrl->dev->step_engine[num],
					calc_steps(contrl->programms[contrl->exe_prog].par1) * RATIO_GEARBOX,
					(float) ((float) contrl->programms[contrl->exe_prog].par2 * RATIO_GEARBOX / BASE_FREQ
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
			&& contrl->dev->step_engine[num].mode == STOP) {
		contrl->programms[contrl->exe_prog].state = STATE_BUSY_COMMAND;
		if (contrl->programms[contrl->exe_prog].par2 > MAX_VEL_PROG) {
			contrl->programms[contrl->exe_prog].par2 = MAX_VEL_PROG;
		} else if (contrl->programms[contrl->exe_prog].par2 > 0x0) {

			uint32_t start_pos = START_POS_VALUE + calc_steps(contrl->programms[contrl->exe_prog].par1) * RATIO_GEARBOX;
			uint32_t cur_pos = contrl->dev->step_engine[num].
							engine_TIM_slave->Instance->CCR1;

			int32_t res = (int16_t)start_pos - (int16_t )cur_pos;

			if (res > 1 || res < -1) {
				move_step_engine(&contrl->dev->step_engine[num], (int16_t )(res), (float)(
									(float)contrl->programms[contrl->exe_prog].par2 * RATIO_GEARBOX / BASE_FREQ
									/ ANFLE_ONE_STEP * 2));
			} else {
				contrl->programms[contrl->exe_prog].state = STATE_READ_COMMAND;
			}
		} else {
			contrl->programms[contrl->exe_prog].state = STATE_READ_COMMAND;
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

		contrl->dev->bldc_engine->power = contrl->programms[contrl->exe_prog].par1;

		if (contrl->dev->bldc_engine->power > 0) {
			start_BLDC(contrl->dev->bldc_engine, contrl->dev->bldc_engine->power);
		} else {
			stop_BLDC(contrl->dev->bldc_engine);
		}
		contrl->programms[contrl->exe_prog].state = STATE_READ_COMMAND;
	}
}

/*
* @bref: save struct data in FLASH
*	@param (contrl) - all parameters for controlling by HMI
*/

void refresh_prog_parameters_FLASH(t_control* contrl) {
	CMSIS_internal_flash_erase(FLASH_SNB_SEC_3);
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
	contrl->dev->step_engine[num].manual_mode = 0x01;

	if (contrl->current_vel != 0x00 && contrl->dev->step_engine[num].mode == STOP) {
		if (contrl->dev->step_engine[num].manual_move_left) {
			move_step_engine(&contrl->dev->step_engine[num], BASE_STEP_MANUAL * RATIO_GEARBOX,
					(float) ((float) contrl->current_vel / BASE_FREQ
					/ ANFLE_ONE_STEP * 2));
		}
		else if (contrl->dev->step_engine[num].manual_move_right) {
			move_step_engine(&contrl->dev->step_engine[num], -BASE_STEP_MANUAL * RATIO_GEARBOX,
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

		contrl->programms[contrl->exe_prog].state = STATE_SAND_REQUEST;

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

	if (contrl->dev->vac_pump->state == PUMP_IS_ON) {
		switch (is_setpoint(contrl->dev->vac_pump, contrl->dev->vac_sensor)) {

		case VAC_SETPOINT_LOWER:

			//TO DO: pump ON
			pump_ON();

			break;
		case VAC_SETPOINT_UPPER:

			//TO DO: pump OFF
			pump_OFF();

			break;
		}
	}
#endif

	switch (contrl->programms[contrl->exe_prog].state) {
	case STATE_IDLE_COMMAND:
		contrl->programms[contrl->exe_prog].read_cnt = 0;
		if (contrl->is_launch >= 0x01) {
			contrl->programms[contrl->exe_prog].state = STATE_READ_COMMAND;
		}
		break;

	case STATE_READ_COMMAND:

		pState = proccesing_HMI_request(&contrl->programms[contrl->exe_prog]);

		if (pState == STATE_END_PROGRAMM) {
			contrl->programms[contrl->exe_prog].state = STATE_SAND_REQUEST;
			discr.state = END_PROGRAMM;
			ringBuf_put(discr, contrl->queue);
		}
		break;

	case STATE_EXECUTE_COMMAND:
		if (strcmp(contrl->programms[contrl->exe_prog].com_dscr.name, "AX1") == 0) {
		#if STEP_ENGINE_ENABLE
			move_SE_to(contrl, 0);
		#endif
		}
		else if (strcmp(contrl->programms[contrl->exe_prog].com_dscr.name, "AX2") == 0) {
		#if STEP_ENGINE_ENABLE
			move_SE_to(contrl, 1);
		#endif
		}
		else if (strcmp(contrl->programms[contrl->exe_prog].com_dscr.name, "MIX") == 0) {
		#if BLDC_ENGINE_ENABLE
			control_BLCD(contrl);
		#endif
		}
		else if (strcmp(contrl->programms[contrl->exe_prog].com_dscr.name, "VS") == 0) {
			contrl->dev->vac_pump->setpoint = contrl->programms[contrl->exe_prog].par1;
			set_pump_state(contrl->dev->vac_pump, PUMP_IS_ON);
		}
		else if (strcmp(contrl->programms[contrl->exe_prog].com_dscr.name, "DL") == 0) {
			start_delay(contrl->programms, contrl->exe_prog);
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
			contrl->programms[contrl->exe_prog].state = STATE_READ_COMMAND;
		} else if (discr.state == NONE_REQUEST && contrl->is_launch == 0) {
			contrl->programms[contrl->exe_prog].state = STATE_IDLE_COMMAND;
		}
		break;
	}

	#if STEP_ENGINE_ENABLE
	if( contrl->is_manual >= 0x01 && contrl->is_launch == 0x00 ){
		contrl->is_manual = 0x00;
		//contrl->dev->step_engine[usRegBuf[STEP_ENGINE_CHOOSE] - 1].start_pose_mode = 0x00;
		if (usRegBuf[STEP_ENGINE_CHOOSE] > 0) {
			munual_mode(contrl, usRegBuf[STEP_ENGINE_CHOOSE] - 1);
		}
	}
	
	if ((contrl->start_pos_step_engine && 0xFF) == 0x01) {
		if (usRegBuf[STEP_ENGINE_CHOOSE] > 0
				&& contrl->dev->step_engine[usRegBuf[STEP_ENGINE_CHOOSE] - 1].mode == STOP ) {
			contrl->dev->step_engine[usRegBuf[STEP_ENGINE_CHOOSE] - 1].start_pose_mode = 0x01;
			contrl->programms[contrl->exe_prog].state = STATE_BUSY_COMMAND;
			move_start_pos(contrl, usRegBuf[STEP_ENGINE_CHOOSE] - 1);
		}
	}

	/* Reach limit switch */
	for (uint8_t idx = 0; idx < ENGINE_NUM; idx++) {
		if (contrl->dev->step_engine[idx].is_lim_sw) {
			//contrl->dev->step_engine[idx].is_lim_sw = 0x0;
			reach_lim_switch(&contrl->dev->step_engine[idx]);
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

void reach_lim_switch(t_step_engine* step_eng) {
	uint32_t start_pos = step_eng->limit_switch_coord;
	uint32_t cur_pos = step_eng->engine_TIM_slave->Instance->CNT;

	int32_t res = (uint32_t) start_pos - (uint32_t) cur_pos;

	move_step_engine(step_eng, (int16_t) (res),
			(float) ((float) 5 * RATIO_GEARBOX / BASE_FREQ / ANFLE_ONE_STEP * 2));
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
	

	for (uint8_t i = 0; i < ENGINE_NUM; i++) {
		contrl->dev->step_engine[i].manual_move_left = (uint8_t)usRegBuf[STEP_ENGINE_MOVE_LEFT];
		contrl->dev->step_engine[i].manual_move_right = (uint8_t)usRegBuf[STEP_ENGINE_MOVE_RIGHT];
	}
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
	if (vSen->cur_pres >= vPump->setpoint + vPump->offset) {
		return VAC_SETPOINT_LOWER;
	} else if (vSen->cur_pres < vPump->setpoint - vPump->offset) {
		return VAC_SETPOINT_UPPER;
	}

}

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

void start_delay(prog_dscrptr*  programm, uint8_t num) {
	programm[num].state = STATE_BUSY_COMMAND;
	TIM9->ARR *= programm[num].par1;
	TIM9->CR1 |= TIM_CR1_CEN;
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


/**
  * @brief This function handles PB7 - signal from limit switch.
  */
void EXTI9_5_IRQHandler(void){

	uint32_t cur_coord_1 = ctrl.dev->step_engine[0].engine_TIM_slave->Instance->CNT;
	uint32_t cur_coord_2 = ctrl.dev->step_engine[1].engine_TIM_slave->Instance->CNT;


	if (EXTI->PR & EXTI_PR_PR7) {
		ctrl.dev->step_engine[0].limit_switch_coord = cur_coord_1;
		ctrl.dev->step_engine[0].is_lim_sw = 0x1;
		run_step(&ctrl.dev->step_engine[0]);
		EXTI->PR |= EXTI_PR_PR7;
	}
	if (EXTI->PR & EXTI_PR_PR9) {
		ctrl.dev->step_engine[1].limit_switch_coord = cur_coord_2;
		ctrl.dev->step_engine[1].is_lim_sw = 0x1;
		run_step(&ctrl.dev->step_engine[1]);
		EXTI->PR |= EXTI_PR_PR9;
	}
}

void TIM1_BRK_TIM9_IRQHandler(void) {
	ctrl.programms[ctrl.exe_prog].state = STATE_READ_COMMAND;
	TIM9->SR &= ~TIM_SR_UIF;
	TIM9->CR1 &= ~TIM_CR1_CEN;
	TIM9->ARR = DELAY_ARR_BASE;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {

	if (hadc->Instance == ADC1) {
		ctrl.dev->vac_sensor->cur_pres = HAL_ADC_GetValue(hadc);
	}
}
