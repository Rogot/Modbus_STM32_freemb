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
}

/*
* @bref: get current value from ADC
* @param (hadc) - using ADC
* @res - current value
*/
uint16_t get_value( ADC_HandleTypeDef* hadc ) {
	return HAL_ADC_GetValue(hadc);
}
