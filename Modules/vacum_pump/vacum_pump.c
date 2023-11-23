/*
 * vacum_pump.c
 *
 *  Created on: 13 нояб. 2023 г.
 *      Author: maksi
 */

#include "vacum_pump.h"

void init_vac_pump(t_vac_pump* vPump, uint16_t offset, uint16_t setpoint) {
	vPump->setpoint = setpoint;
	vPump->offset = offset;
	vPump->state = PUMP_IS_ON;

	/* PB9 - control vacuum ON/OFF */
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	GPIOB->MODER |= GPIO_MODER_MODE9_1; /* Output mode */
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR9_1;
}

/*
* @bref: set setpoint
* @param (vPump) - vacuum pump struct
* @param (value) - setpoint value
*/
void set_setpoint(t_vac_pump* vPump, uint16_t value) {
	vPump->setpoint = value;
}

void set_pump_state(t_vac_pump* vPump, eVacPumpState pmState) {
	vPump->state = pmState;
}

void get_pump_state(t_vac_pump* vPump) {
	return vPump->state;
}

/* ON/OFF Vacuum pump */
void pump_ON( void ) {
	GPIOB->BSRR |= GPIO_BSRR_BS9;
}
void pump_OFF( void ) {
	GPIOB->BSRR |= GPIO_BSRR_BR9;
}
