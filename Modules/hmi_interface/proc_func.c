#include "proc_func.h"

void proc_funct_init(prog_dscrptr* pd) {
	pd->state = STATE_IDLE_COMMAND;
	pd->read_cnt = 0;
}

/*
* @bref: proccesing program grom HMI
*	@param (pd) - programm descriptor
*/
void proccesing_HMI_request(prog_dscrptr* pd) {

	pd->state = parser(pd->usDataRx, &pd->com_dscr, &pd->read_cnt, pd->size);

	/* TEST strcmp() */
	/*
	int rc = strcmp(pd->com_dscr.name, "#PR");
	const char *rel = rc < 0 ? "precedes" : rc > 0 ? "follows" : "equals";
	printf("[%s] %s [%s]\n", pd->com_dscr.name, rel, "#PR");
	 */

	if (pd->state != STATE_IDLE_COMMAND) {
		if(strcmp(pd->com_dscr.name, "#PR") == 0) {
			copy(pd->com_dscr.p1, pd->prog_name, MAX_SIZE_PARAMETR);
			pd->state = STATE_READ_COMMAND;
		} else if (strcmp(pd->com_dscr.name, "AX1") == 0) {
			pd->par1 = atoi(pd->com_dscr.p1);
			pd->par2 = atoi(pd->com_dscr.p2);
		} else if (strcmp(pd->com_dscr.name, "AX2") == 0) {
			pd->par1 = atoi(pd->com_dscr.p1);
			pd->par2 = atoi(pd->com_dscr.p2);
		} else if (strcmp(pd->com_dscr.name, "MIX") == 0) {
			pd->par1 = atoi(pd->com_dscr.p1);
		}
	}
}

/*
* @bref: split str to words and forming a single command
*	@param (src) - array with symbolic description of program
*	@param (cmd) - array for store full command
*	@param (sign) - sign for split "sentenses"
*	@param (start_pos) - start of read array
*	@param (usLen) - lenth of "src"
*/
#if !TWO_BYTE_CNTRL_SIGN
void split(const uint8_t* src, uint8_t* cmd, uint8_t sign, uint8_t* start_pos, uint8_t usLen) {
	uint8_t i = 0;
	
	while (src[i] != sign) {
		cmd[i] = src[*start_pos + i];
		i++;
	}
	*start_pos = i + 1;
}
#endif

/*
* @bref: split str to words and forming a single command (use 2 byte cntrol symbol)
*	@param (src) - array with symbolic description of program
*	@param (cmd) - array for store full command
*	@param (sign1) - sign for split "sentenses" (byte 1)
*	@param (sign2) - sign for split "sentenses" (byte 2)
*	@param (start_pos) - start of read array
*	@param (usLen) - lenth of "src"
*/
#if TWO_BYTE_CNTRL_SIGN
void split_d(const uint8_t* src, uint8_t* cmd, uint8_t sign1, uint8_t sign2, uint8_t* start_pos, uint8_t usLen) {
	uint8_t i = 0;
	
	while (src[*start_pos + i] != sign1 && src[*start_pos + i + 1] != sign2) {
		cmd[i] = src[*start_pos + i];
		i++;
	}
	*start_pos += i + 2;
}
#endif

/*
* @bref: parser for program
*	@param (program) - array with symbolic description of program
*	@param (com_dscr) - struvture for descript command
*	@param (start_pos) - start of read array
*	@param (usLen) - symbolic program lenth
*/
eProcState parser(const uint8_t* program, command_dscrptr* com_dscr,
					uint8_t* start_pos, uint8_t usLen) {
		uint8_t cmd[MAX_SIZE_COMMAND];
		clear(cmd, MAX_SIZE_COMMAND);
		int par_cnt = 0;

		#if !TWO_BYTE_CNTRL_SIGN
		split(program, cmd, RET_CAR, start_pos, usLen);
		#endif
		#if TWO_BYTE_CNTRL_SIGN
		if ( program[*start_pos + 1] == 'E' && program[*start_pos] == '#' 
			&& program[*start_pos + 2] == 'N' && program[*start_pos + 3] == 'D') {
			return STATE_IDLE_COMMAND;
		} else {
			split_d(program, cmd, NEW_LINE, RET_CAR, start_pos, usLen);
			
			par_cnt = sscanf(cmd, "%s %s %s %s", com_dscr->name, &com_dscr->p1, &com_dscr->p2, &com_dscr->p3);

			if (par_cnt == 0) return STATE_IDLE_COMMAND;
			else if (par_cnt > 0) return STATE_EXECUTE_COMMAND;
		}
		#endif
}

/*
* @bref: copy program from serial port array
*	@param (src) - array with data from serial port
*	@param (dst) - array with executable program
*	@param (start_data_pos) - start of read array
*	@param (usLen) - data lenth
* @return (uint8_t) - payload size
*/
uint8_t copy_to(const uint8_t* src, uint8_t* dst, uint8_t start_data_pos, uint8_t usLen) {
	
	uint8_t real_size = 0;
	
	for (uint8_t i = 0; i < usLen; i++) {
		dst[i] = src[start_data_pos + i];
		
		if (src[start_data_pos + i] == '#') {
			if (src[start_data_pos + i + 1] == 'E'
			&& src[start_data_pos + i + 2] == 'N' 
			&& src[start_data_pos + i + 3] == 'D') {
				real_size = i;
				dst[++i] = 'E';
				dst[++i] = 'N';
				dst[++i] = 'D';
				real_size += 3;
				i = usLen;
			}
		}
	}
	return real_size;
}
