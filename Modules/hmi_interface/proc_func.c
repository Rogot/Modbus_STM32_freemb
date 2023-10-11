#include "proc_func.h"

prog_dscrptr pr_dscr;
//uint8_t usDataRx[DATA_RX_MAX_SIZE];

/*
* @bref: proccesing program grom HMI
*	@param (usRegBuf) - array with data from serial port
*	@param (start_data_pos) - start executing programm position
*	@param (usLen) - data lenth
*/
void proccesing_HMI_request(const uint8_t* usRegBuf, uint8_t usLen) {
	
	//copy_to(&usRegBuf[start_data_pos], usDataRx, usLen);
	parser(pr_dscr.usDataRx, usLen);
}

/*
* @bref: split str to words
*	@param (src) - array with symbolic description of program
*	@param (sign) - sign for split "sentenses"
*	@param (usLen) - lenth of "src"
*/

void split(const uint8_t* src, uint8_t sign, uint8_t* start_pos, uint8_t usLen) {
	
	
	while (src[*start_pos] != sign) {
		
	}
}


/*
* @bref: parser for program
*	@param (program) - array with symbolic description of program
*	@param (usLen) - symbolic program lenth
*/
void parser(const uint8_t* program, uint8_t usLen) {
	
	for (uint8_t i = 0; i < usLen; i++) {
		split(program, RET_CAR, &i, usLen);
	}
}

/*
* @bref: copy program from serial port array
*	@param (src) - array with data from serial port
*	@param (dst) - array with executable program
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
