#ifndef DISPLAY_H_
#define DISPLAY_H_

#include <esp_err.h>
#include <pcd8544.h>
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
    LCD_EVT_VALUE,
    LCD_EVT_ROTATE,
    LCD_EVT_BTN_0,
    LCD_EVT_BTN_1
} lcd_evt_type_t;

typedef struct
{
    float pressure;
    float temperature;
    float humidity;
} lcd_evt_data_t;

typedef struct
{
    lcd_evt_type_t type;
    sensor_id_t id;
    lcd_evt_data_t data;
} lcd_evt_t;

void display_init();

void display_print_value(sensor_id_t sensor_id, float pressure, float temperature, float humidity);

void display_print_min_max(sensor_id_t sensor_id, char *parameter, float data[HISTORY_SIZE], char *unit);

void display_print_graph(sensor_id_t sensor_id, char *parameter, float data[HISTORY_SIZE], uint16_t unit_size);

#endif /* DISPLAY_H_ */