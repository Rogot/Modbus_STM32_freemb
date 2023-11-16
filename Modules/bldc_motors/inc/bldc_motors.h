/*
 * bldc_motors.h
 *
 *  Created on: 16 окт. 2023 г.
 *      Author: maksi
 */

#ifndef BLDC_MOTORS_H
#define BLDC_MOTORS_H

#include "dac_cmsis.h"

typedef struct BLDC_ENGINE {
uint8_t manual_mode;
t_dac* dac; 					/* ptr for using DAC */
uint16_t power;
}t_bldc_engine;

void init_BLDC(t_bldc_engine* bldc, t_dac* dac);
void start_BLDC(t_bldc_engine* bldc, uint16_t power);
void stop_BLDC(t_bldc_engine* bldc);

#endif /* !BLDC_MOTORS_H */
