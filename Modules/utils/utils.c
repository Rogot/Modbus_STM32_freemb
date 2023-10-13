#include "utils.h"

/*
* 	@bref: clear array of char - change slots to "space"
*	@param (arr) - array for clear
*	@param (size) - size of array
*/
void clear(uint8_t* arr, uint8_t size) {
	for (uint8_t i = 0; i < size; i++) {
		arr[i] = ' ';
	}
}

/*
* 	@bref: copy dat from array "src" to "dst"
*	@param (src) - copied array
*	@param (dst) - output array
*	@param (size) - count of copy symbols
*/
void copy(uint8_t* src, uint8_t* dst, uint8_t cnt) {
	for (uint8_t i = 0; i < cnt; i++) {
		dst[i] = src[i];
	}
}

