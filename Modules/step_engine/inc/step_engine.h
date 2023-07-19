#ifndef STEP_ENGINE_H
#define STEP_ENGINE_H

#include <stm32f405xx.h>
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include "math.h"

/* DEFINE BEGIN */
#define STEP_ENGINE_ENABLE		 			( 1 )
#define STEP_ENGINE_TEST_ENABLE		  ( 1 )

#define BASE_FREQ 1e+5
#define ACCEL_TIME_SEC 1.0
#define DECEL_TIME_SEC 1.0
#define ACCEL_TIME (ACCEL_TIME_SEC*BASE_FREQ)
#define DECEL_TIME DECEL_TIME_SEC*BASE_FREQ
#define ACCEL 0.000000001	
#define SPEED_MIN 10.0/65535
#define SPEED_MAX 0.1

#define STOP 1
#define SPEEDUP 2
#define RUN 3
#define SLOWDOWN 4

/* DEFINE END */

typedef struct step_engine {
	float vel;
	float accel;
	size_t accel_size;
	uint8_t mode;
	int dir;
	uint16_t speedupbuf[10240];
	uint16_t slowdownbuf[10240];
	uint32_t speedupCNT;
	uint32_t runCNT;
	uint32_t slowdownCNT;
} t_step_engine;

void init_step_engine(t_step_engine* step_eng);
void led_init(); /* LED-pin init */

inline float mysqrt(float value);


#endif //!STEP_ENGINE_H