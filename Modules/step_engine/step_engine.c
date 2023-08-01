#include "step_engine.h"

/* VARIABLES BEGIN */
t_step_engine step_engine;

uint8_t start=0;

eMBEventType    eEvent;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern DMA_HandleTypeDef hdma_tim1_ch4_trig_com;
/* VARIABLES END */

/*
* @bref: step_eng - pointer to the step engine which will be controlled
*/

void init_step_engine(t_step_engine* step_eng) {
	TIM2->CNT = 2147483647;
	HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);
	
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
	
	(*step_eng).accel =(SPEED_MAX - SPEED_MIN)/ACCEL_TIME;
	(*step_eng).accel_size = (*step_eng).accel * ACCEL_TIME * ACCEL_TIME/2;
	
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
* @bref: step_eng - pointer to the step engine which will be controlled
				 pos - moving distance
				 vel - moving velocity
*/

void move_step_engine(t_step_engine* step_eng, int16_t pos, float vel) {
	
	(*step_eng).cnt = (int16_t)(pos - TIM3->CNT);
	(*step_eng).cnt = (int16_t)(pos);
	(*step_eng).vel = vel;
	
	char* temp = (char*)(&((*step_eng).cnt));
	
	if ((*step_eng).cnt < 0) {
		(*step_eng).cnt = -((*step_eng).cnt);
		(*step_eng).dir = -1;
		TIM2->CR1|=TIM_CR1_DIR;
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
	} else {
		(*step_eng).dir = 1;
		TIM2->CR1&=~(TIM_CR1_DIR);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
	}
	
	(*step_eng).cur_accel_size = pow((*step_eng).vel - SPEED_MIN ,2) /
														(2 * (*step_eng).accel);
	
	 if (2 * (*step_eng).cur_accel_size < (*step_eng).cnt) {
		 (*step_eng).runCNT = (*step_eng).cnt -
													2 * (*step_eng).cur_accel_size;
		 (*step_eng).speedupCNT = (*step_eng).cur_accel_size;
		 (*step_eng).slowdownCNT = (*step_eng).cur_accel_size;
	 } else {
		 (*step_eng).runCNT = 0;
	 }
	 
	 if ((*step_eng).speedupCNT != (*step_eng).slowdownCNT) {
		 (*step_eng).speedupCNT = (uint32_t)((*step_eng).cur_accel_size / 2) + 1;
		 (*step_eng).slowdownCNT = (*step_eng).cur_accel_size  - (*step_eng).speedupCNT;
	 }
	 
	 //while (eEvent != EV_FRAME_SENT) {
			//xMBPortEventGet(&eEvent);
	 //}
	 
	 TIM2->CCR1=TIM2->CNT+step_engine.dir*step_engine.speedupCNT;
	 HAL_DMA_Start_IT(htim3.hdma[TIM_DMA_ID_UPDATE], 
					(uint32_t)step_engine.speedupbuf, 
					(uint32_t)&TIM3->ARR, 
		step_engine.speedupCNT);
		__HAL_TIM_ENABLE_DMA(&htim3, TIM_DMA_UPDATE);
		step_engine.mode=SPEEDUP;
		HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_3);
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2)
	{
    	if(step_engine.mode == SPEEDUP){
    		if (!step_engine.manual_mode) {
					TIM2->CCR1 = TIM2->CNT + step_engine.dir * step_engine.runCNT;
					step_engine.mode = RUN;
				} else {
					TIM2->CCR1 = TIM2->CNT + step_engine.dir * 3000;
					if (!step_engine.manual_move_left && !step_engine.manual_move_right) {
						step_engine.mode = RUN;
					}
				}
    	}
    	else
    	if(step_engine.mode == RUN){
    		TIM2->CCR1 = TIM2->CNT + step_engine.dir * step_engine.slowdownCNT;
				step_engine.mode = SLOWDOWN;
				
				HAL_DMA_Start_IT(htim3.hdma[TIM_DMA_ID_UPDATE], 
				(uint32_t)(step_engine.slowdownbuf 
								+ step_engine.accel_size - step_engine.slowdownCNT),
				(uint32_t)&TIM3->ARR, step_engine.slowdownCNT);
				
				__HAL_TIM_ENABLE_DMA(&htim3, TIM_DMA_UPDATE);
			}
    	else
    	if(step_engine.mode == SLOWDOWN){
    		HAL_TIM_OC_Stop(&htim3, TIM_CHANNEL_3);
				HAL_DMA_Abort_IT(htim3.hdma[TIM_DMA_ID_UPDATE]);
    		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
				step_engine.mode = STOP;
				//step_engine.manual_mode = 0x01;
    	}
    }
}