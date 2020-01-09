#ifndef DISPLAY_H_
#define DISPLAY_H_

#include <stddef.h>

#define HISTORY_SIZE        96          /* 4px each hour */
#define HISTORY_PERIOD      900000      /* 15min */

typedef enum
{
    DISPLAY_VALUE_TEMPERATURE,
    DISPLAY_VALUE_HUMIDITY,
    DISPLAY_VALUE_PRESSURE,
    DISPLAY_VALUE_IAQ,
    DISPLAY_VALUE_OUT_TEMPERATURE,
    DISPLAY_VALUE_OUT_HUMIDITY,
    DISPLAY_VALUE_NB
} display_value_t;

typedef enum
{
    DISPLAY_SETTING_TIME,
    DISPLAY_SETTING_BT_START_PAIRING,
    DISPLAY_SETTING_BT_REMOVE_PAIRED_DEVICES,
    DISPLAY_SETTING_BACK,
    DISPLAY_SETTING_NB
} display_setting_t;

typedef enum
{
    DISPLAY_TIME_HOURS,
    DISPLAY_TIME_MINUTES,
    DISPLAY_TIME_SECONDS,
    DISPLAY_TIME_SET,
    DISPLAY_TIME_BACK,
    DISPLAY_TIME_NB
} display_time_t;

void display_init();

void display_clear();

void display_sync();

void display_print_main(float temperature, float humidity, float iaq, float pressure, float out_temperature,
        float out_humidity, bool bt_connection, display_value_t selected_value);

void display_print_graph(display_value_t value, float data[HISTORY_SIZE]);

void display_settins(display_setting_t selected);

void display_time(uint8_t hours_offset, uint8_t minutes_offset, uint8_t seconds_offset, display_time_t selected);

void display_print_pairing(uint32_t passkey, bool blink);

void display_print_paired_result(bool successfully);

#endif /* DISPLAY_H_ */
