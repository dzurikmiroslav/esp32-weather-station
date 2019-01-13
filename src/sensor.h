#ifndef SENSOR_H_
#define SENSOR_H_

#include <stdint.h>

typedef struct
{
   float pressure;
   float temperature;
   float humidity;
   uint8_t battery;
} __attribute__((packed)) espnow_sensor_data_t;

#define SENSOR_COUNT 2

typedef enum
{
   SENSOR_ID_INTERNAL,
   SENSOR_ID_EXTERNAL
} sensor_id_t;

#define HISTORY_SIZE 72        /* 3px each hour */
#define HISTORY_PERIOD 1200000 /* 20min */

#endif /* SENSOR_H_ */