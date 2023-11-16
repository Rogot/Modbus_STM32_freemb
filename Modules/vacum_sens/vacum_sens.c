/*
 * vacum_sens.c
 *
 *  Created on: 1 нояб. 2023 г.
 *      Author: maksi
 */

#include "vacum_sens.h"

/*
* 	@bref: init vacuum sensor struct
*	@param (vSen) - struct vacuum sensor
*	@param (adc) - used ADC for recieve input signal
*	@param (setpoint) - start setpoint
*/
void init_vac_sens(t_vac_sen* vSen, ADC_HandleTypeDef* adc) {
	vSen->adc = adc;

	#if 1
		/*TEST*/
		/* PB7 */

		RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;	/* Enable alternative functions */
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; 	/* Enable GPIOB */

		GPIOB->MODER &= ~GPIO_MODER_MODE7; /* Input mode */
		GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR7_1; /* High speed */
		//GPIOB->PUPDR |= GPIO_PUPDR_PUPD7_1;

		SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI7_PB;
		EXTI->PR = EXTI_PR_PR7;
		EXTI->IMR |= EXTI_IMR_MR7; /* Enable interrupts from EXTI7 */
		EXTI->RTSR |= EXTI_RTSR_TR7; /* Rising Trigger enable */
		NVIC_EnableIRQ(EXTI9_5_IRQn); /* Handler for EXTI 5...9 interrupt */
	#endif
}

/*
* @bref: get current value from ADC
* @param (hadc) - using ADC
* @res - current value
*/
uint16_t get_value( ADC_HandleTypeDef* hadc ) {
	return HAL_ADC_GetValue(hadc);
}
