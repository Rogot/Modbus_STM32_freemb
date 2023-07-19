#ifndef HMI_INTERFACE_H
#define HMI_INTERFACE_H

#include <stm32f405xx.h>

typedef struct HMI_REGISTERS {
	uint16_t current_temp;
	uint16_t setpoint_temp;
	uint16_t mixing_speed;
	uint16_t mixing_time;
	uint16_t vacuum_time;
	uint16_t air_pubping_lvl;
}t_hmi_reg;

#endif //!HMI_INTERFACE_H