#include "step_engine.h"
#include "hmi_interface.h"
/* VARIABLES BEGIN */
t_step_engine step_engines[ENGINE_NUM];
extern t_control ctrl;
extern uint16_t reqiest_data;
uint8_t start=0;


//extern TIM_HandleTypeDef htim2;
//extern TIM_HandleTypeDef htim3;
extern DMA_HandleTypeDef hdma_tim1_ch4_trig_com;

extern uint8_t is_start_pos;
/* VARIABLES END */

/*
* @bref: initial parameters of step engine
* @param (step_eng) - pointer to the step engine which will be controlled
* @param (engine_TIM_master) - pointer to the Master Timer
* @param (engine_TIM_slave) - pointer to the Slave Timer
*/
void init_step_engine(t_step_engine* step_eng,
						TIM_HandleTypeDef* engine_TIM_master,
						TIM_HandleTypeDef* engine_TIM_slave) {

	(*step_eng).mode = STOP;
	(*step_eng).vel = SPEED_MIN;
	(*step_eng).accel_size = 0;
	(*step_eng).dir = 1;
	(*step_eng).is_lim_sw = 0;

	(*step_eng).engine_TIM_master = engine_TIM_master;
	(*step_eng).engine_TIM_slave = engine_TIM_slave;

	(*step_eng).engine_TIM_slave->Instance->CNT = START_POS_VALUE;
	(*step_eng).engine_TIM_slave->Instance->CCR1 = START_POS_VALUE;
	HAL_TIM_OC_Start_IT((*step_eng).engine_TIM_slave, TIM_CHANNEL_1);
	
	int t=0;
	float tnext;
	
	#if STEP_ENGINE_TEST_ENABLE
	(*step_eng).mode = STOP;
	(*step_eng).speedupCNT = 500;
	(*step_eng).slowdownCNT = 500;
	(*step_eng).runCNT = 1000;
	(*step_eng).accel_size = 0;
	(*step_eng).dir = 1;
	(*step_eng).vel = SPEED_MIN;
	#endif
	
	(*step_eng).accel =(SPEED_MAX - SPEED_MIN)/ACCEL_TIME; /* Calculate acceliration */
	(*step_eng).accel_size = (*step_eng).accel * ACCEL_TIME * ACCEL_TIME/2; /* Calculate distant */
	
	/* Signal on step engine in each point of speedup */
	for(int i=0; i < (*step_eng).accel_size; i++){
		tnext = sqrt(2 * (i + 1) / (*step_eng).accel);
		(*step_eng).speedupbuf[i] = tnext - t;
		t = t + (*step_eng).speedupbuf[i];
	}
	for(int i = (*step_eng).accel_size - 1; i >= 0; i--){
		(*step_eng).slowdownbuf[i]=(*step_eng).speedupbuf[(*step_eng).accel_size - 1 - i];
	}
	(*step_eng).speedupCNT = (*step_eng).accel_size;
	(*step_eng).runCNT=1000;
	(*step_eng).slowdownCNT = (*step_eng).accel_size;
}

/*
* @bref: calculate movment steps count
* @param (pos) - angle for movment
* @res - num of steps
*/
int16_t calc_steps(int16_t pos) {
		return pos / ANFLE_ONE_STEP * TOGGLE_DIVIDER;
}

int32_t step_count;
float vel_val;
size_t accel_size;

/*
* @bref: setting the displacement of the motor shaft
* @param (step_eng) - pointer to the step engine which will be controlled
* @param (pos) - moving distance
* @param (vel) - moving velocity
*/

void move_step_engine(t_step_engine* step_eng, int16_t pos, float vel) {
	
	(*step_eng).cnt = (int16_t)(pos);
	//(*step_eng).cnt = (int16_t)(pos - TIM3->CNT);
	step_count = (int16_t)(*step_eng).cnt;
	
	if (vel < SPEED_MIN) { 
		(*step_eng).vel = SPEED_MIN;
	} else {
		(*step_eng).vel = vel;
	}
	vel_val = (*step_eng).vel;
	
	if ((*step_eng).cnt < 0 || (*step_eng).manual_move_right) {
		(*step_eng).cnt = -((*step_eng).cnt);
		(*step_eng).dir = -1;
		(*step_eng).engine_TIM_slave->Instance->CR1|=TIM_CR1_DIR;
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
	} else if ((*step_eng).cnt >= 0 || (*step_eng).manual_move_left) {
		(*step_eng).dir = 1;
		(*step_eng).engine_TIM_slave->Instance->CR1 &= ~(TIM_CR1_DIR);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
	}
	
	
	(*step_eng).cur_accel_size = pow((*step_eng).vel - SPEED_MIN, 2)/
													(2 * (*step_eng).accel);
	/*
	(*step_eng).cur_accel_size = (pow((*step_eng).vel, 2) - pow(SPEED_MIN, 2))/
													(2 * (*step_eng).accel);
	*/
	
	accel_size = (*step_eng).cur_accel_size;
	
	 if (2 * (*step_eng).cur_accel_size < (*step_eng).cnt) {
		 (*step_eng).runCNT = (*step_eng).cnt -
							2 * (*step_eng).cur_accel_size;
		 (*step_eng).speedupCNT = (*step_eng).cur_accel_size;
		 (*step_eng).slowdownCNT = (*step_eng).cur_accel_size;
	 } else {
		 (*step_eng).runCNT = 0;
//		 if (!(*step_eng).manual_move_left && !(*step_eng).manual_move_right) {
			(*step_eng).speedupCNT = (*step_eng).cnt / 2;
			(*step_eng).slowdownCNT = (*step_eng).cnt / 2;
//		 } else {
//			(*step_eng).speedupCNT = (*step_eng).cur_accel_size;
//			(*step_eng).slowdownCNT = (*step_eng).cur_accel_size;
//		 }
	 }
	 
	 if ((*step_eng).speedupCNT != (*step_eng).slowdownCNT) {
		 (*step_eng).speedupCNT++;
		 //(*step_eng).speedupCNT = (uint32_t)((*step_eng).cur_accel_size / 2) + 1;
		 //(*step_eng).slowdownCNT = (*step_eng).cur_accel_size  - (*step_eng).speedupCNT;
	 }
	 
	 if ((*step_eng).speedupCNT == 0x0) {
		 (*step_eng).engine_TIM_master->Instance->ARR = 1 / (*step_eng).vel;
		 (*step_eng).engine_TIM_slave->Instance->CCR1 = (*step_eng).engine_TIM_slave->Instance->CNT
																						+ (*step_eng).dir * (*step_eng).runCNT;

		 (*step_eng).mode = RUN;
	 } else {
		 
		 (*step_eng).engine_TIM_slave->Instance->CCR1 = (*step_eng).engine_TIM_slave->Instance->CNT
													+ (*step_eng).dir * (*step_eng).speedupCNT;

		 
		 HAL_DMA_Start_IT((*step_eng).engine_TIM_master->hdma[TIM_DMA_ID_UPDATE], 
						(uint32_t)(*step_eng).speedupbuf, 
						(uint32_t)&(*step_eng).engine_TIM_master->Instance->ARR, 
			(*step_eng).speedupCNT);
			
		 __HAL_TIM_ENABLE_DMA((*step_eng).engine_TIM_master, TIM_DMA_UPDATE);
			
		 (*step_eng).mode=SPEEDUP;
	 }
//	 (*step_eng).engine_TIM_slave->Instance->CCR1 = (*step_eng).engine_TIM_slave->Instance->CNT
//	 													+ (*step_eng).dir * (*step_eng).speedupCNT;
	 HAL_TIM_OC_Start((*step_eng).engine_TIM_master, TIM_CHANNEL_3);
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2 || htim->Instance == TIM5)
	{
			t_step_engine* step_eng = NULL;
			if (htim->Instance == TIM2) {
				step_eng = &step_engines[0];
//				if ((*step_eng).mode == SLOWDOWN) {
//					t_queue_dicr disc;
//
//					disc.state = STEP_ENGINE_1_STOPPED;
//					ringBuf_put(disc, ctrl.queue);
//				}
			} else if (htim->Instance == TIM5) {
				step_eng = &step_engines[1];
//				if ((*step_eng).mode == SLOWDOWN) {
//					t_queue_dicr disc;
//
//					disc.state = STEP_ENGINE_2_STOPPED;
//					ringBuf_put(disc, ctrl.queue);
//				}
			}
		
    	if((*step_eng).mode == SPEEDUP){
    		speed_up_step(step_eng);
    	}
    	else
    	if((*step_eng).mode == RUN){
    		run_step(step_eng);
		}
    	else
    	if((*step_eng).mode == SLOWDOWN){
    		speed_down_step(step_eng);
    	}
    }
}

/*
* @bref: control step engine before speedUp stage
*	@param (step_eng) - Step engine structure
*/
void speed_up_step(t_step_engine *step_eng) {
	//if (!(*step_eng).manual_mode || (*step_eng).start_pose_mode) {
	if ((*step_eng).start_pose_mode) {
		(*step_eng).engine_TIM_slave->Instance->CCR1 =
				(*step_eng).engine_TIM_slave->Instance->CNT
						+ (*step_eng).dir * (*step_eng).runCNT;

		(*step_eng).mode = RUN;
	} else if (is_start_pos) {
		(*step_eng).engine_TIM_slave->Instance->CCR1 = START_POS_VALUE;
		(*step_eng).mode = RUN;
	} else {
		(*step_eng).engine_TIM_slave->Instance->CCR1 =
				(*step_eng).engine_TIM_slave->Instance->CNT
						+ (*step_eng).dir * (*step_eng).runCNT;

//		if (!(*step_eng).manual_move_left && !(*step_eng).manual_move_right) {
			(*step_eng).mode = RUN;
//		}
	}
	(*step_eng).engine_TIM_master->Instance->ARR = (uint16_t) 1
			/ (*step_eng).vel;
}

/*
* @bref: control step engine before run stage
*	@param (step_eng) - Step engine structure
*/
void run_step(t_step_engine *step_eng) {
	if ((*step_eng).slowdownCNT != 0) {
		(*step_eng).engine_TIM_slave->Instance->CCR1 =
				(*step_eng).engine_TIM_slave->Instance->CNT
						+ (*step_eng).dir * (*step_eng).slowdownCNT;

		(*step_eng).mode = SLOWDOWN;

		HAL_DMA_Start_IT((*step_eng).engine_TIM_master->hdma[TIM_DMA_ID_UPDATE],
				(uint32_t) ((*step_eng).slowdownbuf + (*step_eng).accel_size
						- (*step_eng).slowdownCNT),
				(uint32_t) &(*step_eng).engine_TIM_master->Instance->ARR,
				(*step_eng).slowdownCNT);

		__HAL_TIM_ENABLE_DMA((*step_eng).engine_TIM_master, TIM_DMA_UPDATE);
	} else {
		if (!(*step_eng).manual_mode || (*step_eng).start_pose_mode) {
			ctrl.programms[ctrl.exe_prog].state = STATE_READ_COMMAND;
		}
		if ((*step_eng).is_lim_sw) {
			if (!(*step_eng).manual_mode) {
				t_queue_dicr disc;
				ctrl.programms[ctrl.exe_prog].state = STATE_SAND_REQUEST;
				disc.state = END_PROGRAMM;
				(*step_eng).is_lim_sw = 0;
				ringBuf_put(disc, ctrl.queue);
			}
		}
		HAL_TIM_OC_Stop((*step_eng).engine_TIM_master, TIM_CHANNEL_3);
		(*step_eng).mode = STOP;
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
	}
}

/*
* @bref: control step engine before speedDown stage
*	@param (step_eng) - Step engine structure
*/
void speed_down_step(t_step_engine *step_eng) {
	if (!(*step_eng).manual_mode) {
		ctrl.programms[ctrl.exe_prog].state = STATE_READ_COMMAND;
	}
	if ((*step_eng).start_pose_mode) {
		t_queue_dicr discr;

		//ctrl.programms[ctrl.exe_prog].state = STATE_SAND_REQUEST;

		discr.state = HOME_IS_REACHED;
		ringBuf_put(discr, ctrl.queue);
		(*step_eng).start_pose_mode = 0x0;
	}

	HAL_TIM_OC_Stop((*step_eng).engine_TIM_master, TIM_CHANNEL_3);
	HAL_DMA_Abort_IT((*step_eng).engine_TIM_master->hdma[TIM_DMA_ID_UPDATE]);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
	(*step_eng).mode = STOP;
}
