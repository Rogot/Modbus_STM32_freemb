/*
 * bldc_motors.c
 *
 *  Created on: 16 окт. 2023 г.
 *      Author: maksi
 */
#include "bldc_motors.h"

/*
* @bref: start BLDC engine in set value
*	@param (bldc) - struct BLDC motor
*	@param (power) - value of power
*/
void start_BLDC(t_bldc_engine* bldc, uint16_t power) {
	bldc->dac->dac_type->DHR12R1= power;
}


/*
* @bref: stop BLDC engine
*	@param (bldc) - struct BLDC motor
*/
void stop_BLDC(t_bldc_engine* bldc) {
	bldc->dac->dac_type->DHR12R1 = 0;
}
