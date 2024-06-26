#ifndef DWIN_ADDR_CONV_H
#define DWIN_ADDR_CONV_H

#include <stm32f405xx.h>
#include "hmi_interface.h"

#define CURRENT_USED_REG_CNT			( 14 )
#define PLC_ADDR_MAX					( 256 )

typedef struct ADDR_CONV {
	uint16_t PLC_addr;
	uint16_t HMI_addr;
}t_addr_conv;

USHORT conv_addr(t_addr_conv* addr_conv, USHORT hmi_addr);
USHORT conv_addr_to_HMI(t_addr_conv* addr_conv, USHORT reg_addr);

#endif //!DWIN_ADDR_CONV_H
