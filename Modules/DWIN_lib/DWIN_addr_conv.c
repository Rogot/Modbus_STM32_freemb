#include "DWIN_addr_conv.h"

t_addr_conv PLC_addr[PLC_ADDR_MAX] = {

			{  STEP_ENGINE_ON_MC, 			 		0x5002	},
			{  STEP_ENGINE_VEL_MC,  				0x5010	},
			{  STEP_ENGINE_CHOOSE,					0x5012	},
			{  STEP_ENGINE_START_POS_MS,  			0x5004	},
			{  STEP_ENGINE_MOVE_RIGHT,  			0x5056	},
			{  NUM_EXE_PROGRAM,  					0x5014	},
			{  LAUNCH_PROGRAM,  					0x5050	},
			{  SAVE_PROGRAM,  						0x5052	},
			{  TRANSFUSE_ANGLE,  					0x5020	},
			{  MIXING_TIME,  						0x5022	},
			{  VACUUM_TIME,  						0x5024	},
			{  STEP_ENGINE_MOVE_LEFT,  				0x5054	},
			{  SET_HOME,							0x5016	},
			{  SEARCH_HOME,							0x5018	},
			{  VACUUM_VALUE_REG,					0x5026	},
			{  X_COORD,								0x5028	},
			{  Y_COORD,								0x5030	},
			{  VELOCITY_MIX,						0x5032	},
};

USHORT conv_addr_to_HMI(t_addr_conv* addr_conv, USHORT reg_addr) {
	uint16_t i = 0;
	uint8_t is_find = 0;

	while (!is_find) {
		if (addr_conv[i++].PLC_addr == reg_addr) {
			is_find = 1;
		}
		if (i > PLC_ADDR_MAX) {
			return 404;
		}
	}

	return addr_conv[i - 1].HMI_addr;
}

USHORT conv_addr(t_addr_conv* addr_conv, USHORT hmi_addr) {
	
	uint16_t i = 0;
	uint8_t is_find = 0;
	
	while (!is_find) {
		if (addr_conv[i++].HMI_addr == hmi_addr){
			is_find = 1;
		}
		if (i > PLC_ADDR_MAX) {
			return 404;
		}
	}
	
	return addr_conv[i - 1].PLC_addr;

}
