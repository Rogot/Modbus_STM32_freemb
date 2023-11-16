/*
 * vacum_pump.h
 *
 *  Created on: 13 нояб. 2023 г.
 *      Author: maksi
 */

#ifndef VACUM_PUMP_H
#define VACUM_PUMP_H

#include <stm32f405xx.h>

typedef enum {
    VAC_SETPOINT_REACHED,         /*!< Setpoint is reached. */
    VAC_SETPOINT_UPPER,           /*!< Current value is upper setpoint. */
    VAC_SETPOINT_LOWER,           /*!< Current value is lower setpoint. */
} eVacSetPoint;

typedef enum {
	PUMP_IS_ON,
	PUMP_IS_OFF,
}eVacPumpState;

typedef struct VACUM_PUMP {
	eVacPumpState state;
	uint16_t offset;			/*!< Offset for regulate upper/lower measurement boundary */
	uint16_t setpoint;
}t_vac_pump;

void init_vac_pump(t_vac_pump* vPump, uint16_t offset, uint16_t setpoint);
void set_setpoint(t_vac_pump* vPump, uint16_t value);

#endif // !VACUM_PUMP_H
