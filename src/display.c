#include <driver/gpio.h>
#include <math.h>
#include "display.h"

#define LOG_TAG "DISPLAY"

#define GRAPH_START_X 6
#define GRAPH_START_Y 8
#define GRAPH_WIDTH HISTORY_SIZE
#define GRAPH_HEIGHT 37
#define GRAPH_END_X GRAPH_START_X + GRAPH_WIDTH
#define GRAPH_END_Y GRAPH_START_Y + GRAPH_HEIGHT

pcd8544_spi_pin_config_t spi_pin_config = {
    //.miso_io_num = 0,
    .mosi_io_num = 26, //DN
    .sclk_io_num = 27,
    .spics_io_num = 33,
};

pcd8544_control_pin_config_t control_pin_config = {
    .reset_io_num = 32,
    .dc_io_num = 25,
    .backlight_io_num = 16,
};

pcd8544_config_t config = {
    .spi_host = HSPI_HOST,
    .is_backlight_common_anode = false,
};

void display_init()
{
    config.spi_pin = &spi_pin_config;
    config.control_pin = &control_pin_config;

    pcd8544_init(&config);
}

void get_min_max(float *min_val, float *max_val, float data[HISTORY_SIZE])
{
    *max_val = NAN;
    *min_val = NAN;
    for (uint8_t i = 0; i < HISTORY_SIZE; i++)
    {
        *max_val = fmaxf(*max_val, data[i]);
        *min_val = fminf(*min_val, data[i]);
    }
}

void display_print_value(sensor_id_t sensor_id, float pressure, float temperature, float humidity)
{
    if (sensor_id == SENSOR_ID_INTERNAL)
    {
        pcd8544_puts("IN");
    }
    else
    {
        pcd8544_puts("OUT");
    }

    pcd8544_set_pos(0, 1);
    pcd8544_printf("%.2f 'C", temperature);

    pcd8544_set_pos(0, 2);
    pcd8544_printf("%.2f %%", humidity);

    pcd8544_set_pos(0, 3);
    pcd8544_printf("%.2f hPa", pressure);
}

void display_print_graph(sensor_id_t sensor_id, char *parameter, float data[HISTORY_SIZE], uint16_t unit_size)
{
    float min_val;
    float max_val;
    get_min_max(&min_val, &max_val, data);
    int16_t min_bound = (int16_t)floorf(min_val);
    int16_t max_bound = (int16_t)ceilf(max_val);
    min_bound -= min_bound > 0 ? min_bound % unit_size : unit_size - abs(min_bound) % unit_size;
    max_bound += max_bound < 0 ? abs(max_bound) % unit_size : unit_size - max_bound % unit_size;
    int16_t range = max_bound - min_bound;

    for (uint8_t x = 0; x < HISTORY_SIZE - 1; x++)
    {
        if (!isnan(data[x]) && !isnan(data[x + 1]))
            pcd8544_draw_line(GRAPH_START_X + x + 0, GRAPH_END_Y - ((data[x] - min_bound) / range) * GRAPH_HEIGHT,
                              GRAPH_START_X + x + 1, GRAPH_END_Y - ((data[x + 1] - min_bound) / range) * GRAPH_HEIGHT);
    }

    /* value axis */
    pcd8544_draw_line(GRAPH_END_X, GRAPH_START_Y, GRAPH_END_X, GRAPH_END_Y + 1);
    for (uint8_t i = 0; i <= range / unit_size; i++)
    {
        uint8_t y = GRAPH_END_Y - ((float)(i * unit_size) / range) * GRAPH_HEIGHT;
        pcd8544_draw_line(GRAPH_END_X + 1, y, GRAPH_END_X + 3, y);
    }

    /* time axis */
    pcd8544_draw_line(GRAPH_START_X, GRAPH_END_Y + 1, GRAPH_END_X, GRAPH_END_Y + 1);
    for (uint8_t x = GRAPH_START_X + 3; x < GRAPH_END_X; x += 3)
    {
        pcd8544_draw_line(x, GRAPH_END_Y + 2, x, GRAPH_END_Y + 3);
    }

    pcd8544_finalize_frame_buf();

    pcd8544_set_pos(0, 0);
    if (sensor_id == SENSOR_ID_INTERNAL)
    {
        pcd8544_printf("IN %s 24h", parameter);
    }
    else
    {
        pcd8544_printf("OUT %s 24h", parameter);
    }
}

void display_print_min_max(sensor_id_t sensor_id, char *parameter, float data[HISTORY_SIZE], char *unit)
{
    if (sensor_id == SENSOR_ID_INTERNAL)
    {
        pcd8544_printf("IN %s", parameter);
    }
    else
    {
        pcd8544_printf("OUT %s", parameter);
    }

    float max_val;
    float min_val;
    get_min_max(&min_val, &max_val, data);

    pcd8544_set_pos(0, 1);
    pcd8544_puts("MAX");
    pcd8544_set_pos(0, 2);
    pcd8544_printf("%.2f %s", max_val, unit);

    pcd8544_set_pos(0, 3);
    pcd8544_puts("MIN");
    pcd8544_set_pos(0, 4);
    pcd8544_printf("%.2f %s", min_val, unit);
}
