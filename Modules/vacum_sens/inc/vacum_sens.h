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

typedef enum
{
    VAC_SETPOINT_REACHED,         /*!< Setpoint is reached. */
    VAC_SETPOINT_UPPER,           /*!< Current value is upper setpoint. */
    VAC_SETPOINT_LOWER,                 /*!< Current value is lower setpoint. */
} eVacSetPoint;

typedef struct VACUM_SEN {
uint16_t cur_pres;
uint16_t setpoint;
ADC_HandleTypeDef* adc;
}t_vac_sen;

void set_ref(t_vac_sen* vSen);
uint16_t get_value( ADC_HandleTypeDef* hadc );
eVacSetPoint is_setpoint(t_vac_sen* vSen);

#endif // !VACUM_SENS_H
