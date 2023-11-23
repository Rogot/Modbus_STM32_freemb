#ifndef LIMIT_SWITCH_H
#define LIMIT_SWITCH_H

#include <stm32f405xx.h>

typedef struct LSPort {
	char port_letter;
	uint8_t port_number;
}t_LSPort;

void limit_siwitch_init(t_LSPort* port, uint8_t used_engine);

#endif /*LIMIT_SWITCH_H*/
