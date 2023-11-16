#ifndef CONFIG_DWIN_H
#define CONFIG_DWIN_H

#include <stm32f405xx.h>

#define PLC_RATE									72000000

#define DAC_ENABLE									( 1 )

#define DMA_ENABLE									( 1 )

#define SEND_CURV_TEST								( 0 )

#define CURV_ENABLE									( 1 )

#define MODBUS_ENABLE								( 0 ) 

#define DWIN_SERIAL_PORT_ENABLE						( 1 )

#define STEP_ENGINE_ENABLE		 					( 1 )

#define STEP_ENGINE_TEST_ENABLE						( 0 )

#define PEREPH_ENABLE								( 1 )

#define HAL_ADC_MODULE_ENABLED						( 1 )

#define VACUUM_SENSOR_ENABLE						( 1 )

#define BLDC_ENGINE_ENABLE							( 1 )

typedef uint8_t BOOL;

typedef unsigned char UCHAR;
typedef char CHAR;

typedef uint16_t USHORT;
typedef int16_t SHORT;

typedef uint32_t ULONG;
typedef int32_t LONG;

#endif //!CONFIG_DWIN_H
