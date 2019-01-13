#ifndef DISPLAY_H_
#define DISPLAY_H_

#include "sensor.h"

#define LCD_SCREEN_TYPE_COUNT 7

typedef enum
{
    LCD_SCREEN_TYPE_ACTUAL,
    LCD_SCREEN_TYPE_TEMPERATURE_MIN_MAX,
    LCD_SCREEN_TYPE_TEMPERATURE_GRAPH,
    LCD_SCREEN_TYPE_HUMIDITY_MIN_MAX,
    LCD_SCREEN_TYPE_HUMIDITY_GRAPH,
    LCD_SCREEN_TYPE_PRESSURE_MIN_MAX,
    LCD_SCREEN_TYPE_PRESSURE_GRAPH
} lcd_screen_type_t;

typedef enum
{
    DISPLAY_VALUE_HUMIDITY,
    DISPLAY_VALUE_PRESSURE,
    DISPLAY_VALUE_TEMPERATURE
} display_value_t;

void display_init();

void display_clear();

void display_sync();

void display_print_value(sensor_id_t sensor_id, float pressure, float temperature, float humidity);

void display_print_min_max(sensor_id_t sensor_id, display_value_t value, float data[HISTORY_SIZE]);

void display_print_graph(sensor_id_t sensor_id, display_value_t value, float data[HISTORY_SIZE]);

#endif /* DISPLAY_H_ */