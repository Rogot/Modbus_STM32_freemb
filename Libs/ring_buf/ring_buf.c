/*
 * ring_buf.c
 *
 *  Created on: 3 нояб. 2023 г.
 *      Author: maksi
 */

/*
* @bref: put symbol (value) to ring buffer
* @param (symbol) - element for putting
* @param (buf) - the array to be filled in
*/

#include "ring_buf.h"

void ringBuf_put(t_queue_dicr disc, t_ring_buf* buf) {
    buf->que_disc[buf->idxIn].state = disc.state;
    buf->que_disc[(buf->idxIn)++].reg_num = disc.reg_num;
    if (buf->idxIn >= buf->size) {
        buf->idxIn = 0;
    }
}


/*
* @bref: get symbol (value) from ring buffer. Delete reading element
* @param (buf) - ring buffer struct
* @ref - element of array
*/
t_queue_dicr ringBuf_pop(t_ring_buf* buf) {
	t_queue_dicr retVal = buf->que_disc[(buf->idxOut)++];
    if (buf->idxOut >= buf->size) {
        buf->idxOut = 0;
    }
    return retVal;
}

/*
* @bref: get number of useful data
* @param (buf) - ring buffer struct
* @ref - number of useful data
*/
uint16_t ringBuf_getCount(t_ring_buf* buf) {
    uint16_t retVal = 0;
    if (buf->idxIn < buf->idxOut) {
        retVal = buf->size + buf->idxIn - buf->idxOut;
    }
    else {
        retVal = buf->idxIn - buf->idxOut;
    }

    return retVal;
}

/*
* @bref: get symbol (value) from ring buffer. Don't delete reading element
* @param (symbolNum) - offset relative to the beginning of the buffer (read point).
* @param (buf) - ring buffer struct
* @ref - number of useful data
*/
//int32_t ringBuf_showSymmol(uint16_t symbolNum, t_ring_buf* buf) {
//    uint32_t pointer = buf->idxOut + symbolNum;
//    int32_t retVal = -1;
//    if (symbolNum < ringBuf_getCount(buf)) {
//        if (pointer > buf->size) {
//            pointer -= buf->size;
//        }
//        retVal = buf->que_disc[pointer];
//    }
//
//    return retVal;
//}

/*
* @bref: clear ring buffer
* @param (buf) - ring buffer struct
*/
void ringBuf_clear(t_ring_buf* buf) {
    buf->idxIn = 0;
    buf->idxOut = 0;
}

/*
* @bref: init ring buffer
* @param (ring) - ring buffer struct
* @param (buf) - buffer for filling
* @param (size) - size of buffer
* @ret - error status
*/
t_RING_BUF_ErrorStatus ringBuf_init(t_ring_buf* ring, t_queue_dicr* buf, uint16_t size) {
    ring->size = size;
    ring->que_disc = buf;
    ringBuf_clear(ring);

    return (ring->que_disc ? RING_BUFFER_SUCCESS : RING_BUFFER_ERROR);
}
