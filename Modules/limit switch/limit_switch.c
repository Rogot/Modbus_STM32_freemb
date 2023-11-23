#include "limit_switch.h"

void limit_siwitch_init(t_LSPort* port, uint8_t used_engine) {


	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; /* Enable alternative functions */
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; /* Enable GPIOB */

	/* PB7 */
	if (port->port_letter == 'B' && port->port_number == 7) {

		GPIOB->MODER &= ~GPIO_MODER_MODE7; /* Input mode */
		GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR7_1; /* High speed */
		//GPIOB->PUPDR |= GPIO_PUPDR_PUPD7_1;

		SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI7_PB;
		EXTI->PR = EXTI_PR_PR7;
		EXTI->IMR |= EXTI_IMR_MR7; /* Enable interrupts from EXTI7 */
		EXTI->RTSR |= EXTI_RTSR_TR7; /* Rising Trigger enable */
	}
	/* PB9 */
	else if (port->port_letter == 'B' && port->port_number == 9) {

		GPIOB->MODER &= ~GPIO_MODER_MODE9; /* Input mode */
		GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR9_1; /* High speed */
		//GPIOB->PUPDR |= GPIO_PUPDR_PUPD7_1;

		SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI9_PB;

		EXTI->PR = EXTI_PR_PR9;
		EXTI->IMR |= EXTI_IMR_MR9; /* Enable interrupts from EXTI7 */
		EXTI->RTSR |= EXTI_RTSR_TR9; /* Rising Trigger enable */
	}
	NVIC_EnableIRQ(EXTI9_5_IRQn); /* Handler for EXTI 5...9 interrupt */
}


