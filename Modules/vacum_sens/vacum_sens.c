/*
 * vacum_sens.c
 *
 *  Created on: 1 нояб. 2023 г.
 *      Author: maksi
 */

#include "vacum_sens.h"

/*
* @bref: set setpoint
* @param (vSen) - vacuum sensor struct
* @param (value) - setpoint value
*/
void set_setpoint(t_vac_sen* vSen, uint16_t value) {
	vSen->setpoint = value;
}

/*
* @bref: get current value from ADC
* @param (hadc) - using ADC
* @res - current value
*/
uint16_t get_value( ADC_HandleTypeDef* hadc ) {
	return HAL_ADC_GetValue(hadc);
}


/*
* @bref: checking whether the setpoint level has been reached
* @param (vSen) - vacuum sensor struct
* @res - result of checking
*/
eVacSetPoint is_setpoint(t_vac_sen* vSen) {

	if (vSen->cur_pres == vSen->setpoint) {
		return VAC_SETPOINT_REACHED;
	} else if (vSen->cur_pres > vSen->setpoint) {
		return VAC_SETPOINT_LOWER;
	} else if (vSen->cur_pres < vSen->setpoint) {
		return VAC_SETPOINT_UPPER;
	}

}
