/*
 * vacum_sens.h
 *
 *  Created on: 1 нояб. 2023 г.
 *      Author: maksi
 */

#ifndef VACUM_SENS_H
#define VACUM_SENS_H

#include <stm32f405xx.h>
#include "stm32f4xx_hal.h"

typedef struct VACUM_SEN {
uint16_t cur_pres;
ADC_HandleTypeDef* adc;
}t_vac_sen;

void init_vac_sens(t_vac_sen* vSen, ADC_HandleTypeDef* adc);
void set_ref(t_vac_sen* vSen);
uint16_t get_value( ADC_HandleTypeDef* hadc );

#endif // !VACUM_SENS_H
