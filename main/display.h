#ifndef DISPLAY_H_
#define DISPLAY_H_

#include <stddef.h>

#define HISTORY_SIZE 72        /* 3px each hour */

typedef enum
{
    SENSOR_ID_INTERNAL, SENSOR_ID_EXTERNAL
} sensor_id_t;

typedef enum
{
    LCD_SCREEN_TYPE_INT_MAIN,
    LCD_SCREEN_TYPE_INT_TEMPERATURE_MIN_MAX,
    LCD_SCREEN_TYPE_INT_TEMPERATURE_GRAPH,
    LCD_SCREEN_TYPE_INT_HUMIDITY_MIN_MAX,
    LCD_SCREEN_TYPE_INT_HUMIDITY_GRAPH,
    LCD_SCREEN_TYPE_INT_IAQ_MIN_MAX,
    LCD_SCREEN_TYPE_INT_IAQ_GRAPH,
    LCD_SCREEN_TYPE_EXT_MAIN,
    LCD_SCREEN_TYPE_EXT_TEMPERATURE_MIN_MAX,
    LCD_SCREEN_TYPE_EXT_TEMPERATURE_GRAPH,
    LCD_SCREEN_TYPE_EXT_HUMIDITY_MIN_MAX,
    LCD_SCREEN_TYPE_EXT_HUMIDITY_GRAPH,
    LCD_SCREEN_TYPE_EXT_PRESSURE_MIN_MAX,
    LCD_SCREEN_TYPE_EXT_PRESSURE_GRAPH
} lcd_screen_type_t;

#define LCD_SCREEN_INT_FIRST LCD_SCREEN_TYPE_INT_MAIN
#define LCD_SCREEN_INT_LAST LCD_SCREEN_TYPE_INT_IAQ_GRAPH
#define LCD_SCREEN_EXT_FIRST LCD_SCREEN_TYPE_EXT_MAIN
#define LCD_SCREEN_EXT_LAST LCD_SCREEN_TYPE_EXT_PRESSURE_GRAPH

#define LCD_SCREEN_IS_INT(screen) (screen >= LCD_SCREEN_INT_FIRST && screen <= LCD_SCREEN_INT_LAST)
#define LCD_SCREEN_IS_EXT(screen) (screen >= LCD_SCREEN_EXT_FIRST && screen <= LCD_SCREEN_EXT_LAST)

typedef enum
{
    DISPLAY_VALUE_HUMIDITY, DISPLAY_VALUE_PRESSURE, DISPLAY_VALUE_TEMPERATURE, DISPLAY_VALUE_IAQ
} display_value_t;

typedef enum
{
    DISPLAY_ICON_NONE, DISPLAY_ICON_BLUETOOH
} display_icon_t;

void display_init();

void display_clear();

void display_sync();

void display_print_int_value(float temperature, float humidity, float iaq, display_icon_t icon);

void display_print_ext_value(float pressure, float temperature, float humidity, display_icon_t icon);

void display_print_min_max(sensor_id_t sensor_id, display_value_t value, float data[HISTORY_SIZE]);

void display_print_graph(sensor_id_t sensor_id, display_value_t value, float data[HISTORY_SIZE]);

void display_print_pairing(uint32_t passkey, bool blink);

void display_print_paired_result(bool successfully);

#endif /* DISPLAY_H_ */
