#include "step_engine.h"

/* VARIABLES BEGIN */
t_step_engine step_engine;

uint8_t start=0;
uint8_t print=0;
char msg[80]={'\0'};

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern DMA_HandleTypeDef hdma_tim1_ch4_trig_com;

/* VARIABLES END */


void init_step_engine(t_step_engine* step_eng) {
	TIM2->CNT = 2147483647;
	HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);
	
	int t=0;
	float tnext;
	
	#if STEP_ENGINE_TEST_ENABLE
	step_engine.mode = STOP;
	(*step_eng).speedupCNT = 500;
	(*step_eng).slowdownCNT = 500;
	(*step_eng).runCNT = 1000;
	(*step_eng).accel_size = 0;
	(*step_eng).dir = 1;
	#endif
	
	(*step_eng).vel = SPEED_MIN;
	(*step_eng).accel =(SPEED_MAX - (*step_eng).vel)/ACCEL_TIME;
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

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2)
	{
    	if(step_engine.mode == SPEEDUP){
    		TIM2->CCR1 = TIM2->CNT + step_engine.dir * step_engine.runCNT;
    		step_engine.mode = RUN;
       	print = 1;
    	}
    	else
    	if(step_engine.mode == RUN){
    		TIM2->CCR1 = TIM2->CNT + step_engine.dir * step_engine.slowdownCNT;
    		step_engine.mode = SLOWDOWN;
				
    		HAL_DMA_Start_IT(htim3.hdma[TIM_DMA_ID_UPDATE], 
				(uint32_t)step_engine.slowdownbuf,
				(uint32_t)&TIM3->ARR, step_engine.slowdownCNT);
				
    		__HAL_TIM_ENABLE_DMA(&htim3, TIM_DMA_UPDATE);
  
    		print = 1;
    	}
    	else
    	if(step_engine.mode == SLOWDOWN){
    		HAL_TIM_OC_Stop(&htim3, TIM_CHANNEL_3);
    		step_engine.mode = STOP;
    		print = 1;
    		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
    	}
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
 if((GPIO_Pin== GPIO_PIN_1) && (step_engine.mode == STOP) && (start == 0)) {
	 if(step_engine.dir == 1){
		 step_engine.dir = -1;
		 TIM2->CR1 |= TIM_CR1_DIR;
		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
	 }
	 else {
		 step_engine.dir = 1;
		 TIM2->CR1 &= ~(TIM_CR1_DIR);
		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
	 }
	 start = 1;
   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);

 }
}