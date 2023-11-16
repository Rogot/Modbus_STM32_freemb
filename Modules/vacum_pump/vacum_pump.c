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
}

/*
* @bref: set setpoint
* @param (vPump) - vacuum pump struct
* @param (value) - setpoint value
*/
void set_setpoint(t_vac_pump* vPump, uint16_t value) {
	vPump->setpoint = value;
}
