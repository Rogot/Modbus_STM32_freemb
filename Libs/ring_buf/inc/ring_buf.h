/*
 * ring_buf.h
 *
 *  Created on: 3 нояб. 2023 г.
 *      Author: maksi
 */

#ifndef RING_BUF_H
#define RING_BUF_H

#include <stm32f405xx.h>

/* SPECIALS STATUSES FOR REQUESTS */

typedef enum SPECIAL_STATUSES_REQUSET{
NONE_REQUEST = 0,
STEP_ENGINE_1_STOPPED = 1,
STEP_ENGINE_2_STOPPED = 2,
BLDC_ENGINE_STOPPED = 3,
HOME_IS_REACHED = 4,
END_PROGRAMM = 5,
}t_spec_stat_request;

typedef enum RING_BUFFER_ERROR_STATUS{
    RING_BUFFER_SUCCESS,
    RING_BUFFER_ERROR,
}t_RING_BUF_ErrorStatus;

typedef struct QUEUE_DISCRIPTOR {
	t_spec_stat_request state;			/*!< Special status for request */
	uint8_t 			reg_num;		/*!< Modified register */
}t_queue_dicr;

typedef struct RING_BUFFER {
	t_queue_dicr* que_disc;
    uint16_t idxIn;
    uint16_t idxOut;
    uint16_t size;
}t_ring_buf;

void ringBuf_put(t_queue_dicr disc, t_ring_buf* buf);
t_queue_dicr ringBuf_pop(t_ring_buf* buf);
uint16_t ringBuf_getCount(t_ring_buf* buf);
int32_t ringBuf_showSymmol(uint16_t symbolNum, t_ring_buf* buf);
void ringBuf_clear(t_ring_buf* buf);
t_RING_BUF_ErrorStatus ringBuf_init(t_ring_buf* ring, t_queue_dicr* buf, uint16_t size);

#endif /* !RING_BUF_H */
