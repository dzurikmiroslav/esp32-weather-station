#include "driver/gpio.h"
#include "esp_log.h"
#include "math.h"
#include "string.h"
#include "pcd8544.h"

#include "display.h"

#define LOG_TAG "DISPLAY"

#define SPI_RST     32
#define SPI_CE      33
#define SPI_DC      25
#define SPI_DIN     23
#define SPI_CLK     18
#define PCD8544_BL  16

#define GRAPH_START_X 9
#define GRAPH_START_Y 8
#define GRAPH_WIDTH   HISTORY_SIZE
#define GRAPH_HEIGHT  39
#define GRAPH_END_X GRAPH_START_X + GRAPH_WIDTH
#define GRAPH_END_Y GRAPH_START_Y + GRAPH_HEIGHT

// @formatter:off
const uint8_t degree_grade_glyph[] = {
      0x00, 0x06, 0x09, 0x09, 0x06
};

const uint8_t tilde_glyph[] = {
        0x08, 0x04, 0x08, 0x10, 0x08
};

const uint8_t bluetooth_glyph[] =   {
        0x22, 0x14, 0x7F, 0x49, 0x36
};

const uint8_t ind_icon[] = {
        0xFF, 0xFF, 0xFF, 0x81, 0xFF, 0xFF,
        0xFF, 0xFF, 0x81, 0xFB, 0xF7, 0xEF,
        0x81, 0xFF, 0xFF, 0x81, 0xBD, 0xBD,
        0xBD, 0xC3, 0xFF, 0x00, 0x3C, 0x42,
        0x42, 0x42, 0x3C, 0x00, 0x00, 0x3E,
        0x40, 0x40, 0x40, 0x3E, 0x00, 0x00,
        0x02, 0x02, 0x7E, 0x02, 0x02, 0x00
};

const uint8_t out_icon[] = {
        0x00, 0x00, 0x00, 0x7E, 0x00, 0x00,
        0x00, 0x00, 0x7E, 0x04, 0x08, 0x10,
        0x7E, 0x00, 0x00, 0x7E, 0x42, 0x42,
        0x42, 0x3C, 0x00, 0xFF, 0xC3, 0xBD,
        0xBD, 0xBD, 0xC3, 0xFF, 0xFF, 0xC1,
        0xBF, 0xBF, 0xBF, 0xC1, 0xFF, 0xFF,
        0xFD, 0xFD, 0x81, 0xFD, 0xFD, 0xFF
};

pcd8544_spi_pin_config_t spi_pin_config = {
        .mosi_io_num = SPI_DIN,
        .sclk_io_num = SPI_CLK,
        .spics_io_num = SPI_CE,
};

pcd8544_control_pin_config_t control_pin_config = {
        .reset_io_num = SPI_RST,
        .dc_io_num = SPI_DC,
        .backlight_io_num =PCD8544_BL,
};

pcd8544_config_t config = {
        .spi_host = HSPI_HOST,
        .is_backlight_common_anode = false,
};
// @formatter:on

void display_init()
{
    config.spi_pin = &spi_pin_config;
    config.control_pin = &control_pin_config;

    pcd8544_init(&config);
    pcd8544_set_backlight(false);
    pcd8544_create_char('\1', degree_grade_glyph);
    pcd8544_create_char('\2', tilde_glyph);
    pcd8544_create_char('\3', bluetooth_glyph);
}

void display_clear()
{
    pcd8544_clear_display();
    pcd8544_finalize_frame_buf();
}

void display_sync()
{
    pcd8544_sync_and_gc();
}

void get_min_max(float *min_val, float *max_val, float data[HISTORY_SIZE])
{
    *max_val = NAN;
    *min_val = NAN;
    for (uint8_t i = 0; i < HISTORY_SIZE; i++) {
        *max_val = fmaxf(*max_val, data[i]);
        *min_val = fminf(*min_val, data[i]);
    }
}

float get_middle(float data[HISTORY_SIZE])
{
    float sum = 0.0f;
    uint8_t count = 0;
    for (uint8_t i = 0; i < HISTORY_SIZE; i++) {
        if (!isnan(data[i])) {
            sum += data[i];
            count++;
        }
    }
    return sum / count;
}

char *value_unit(display_value_t value)
{
    switch (value) {
        case DISPLAY_VALUE_HUMIDITY:
            return "%";
        case DISPLAY_VALUE_PRESSURE:
            return "hPa";
        case DISPLAY_VALUE_TEMPERATURE:
            return "\1C";
        case DISPLAY_VALUE_CO2:
            return "ppm";
        case DISPLAY_VALUE_TVOC:
            return "ppb";
        default:
            return NULL;
    }
}

uint16_t value_unit_size(display_value_t value)
{
    switch (value) {
        case DISPLAY_VALUE_HUMIDITY:
            return 10;
        case DISPLAY_VALUE_PRESSURE:
            return 10;
        case DISPLAY_VALUE_TEMPERATURE:
            return 5;
        case DISPLAY_VALUE_CO2:
            return 1000;
        case DISPLAY_VALUE_TVOC:
            return 100;
        default:
            return 0;
    }
}

void display_print_int_value(float temperature, float humidity, uint16_t co2, uint16_t tvoc, display_icon_t icon)
{
    pcd8544_set_pos(0, 0);
    pcd8544_draw_bitmap(ind_icon, 7, 6, false);
    pcd8544_finalize_frame_buf();

    pcd8544_set_pos(12, 0);
    pcd8544_printf("%.2f%\1C", temperature);

    pcd8544_set_pos(12, 1);
    pcd8544_printf("%.2f%%", humidity);

    pcd8544_set_pos(12, 2);
    pcd8544_printf("%dppm", co2);

    pcd8544_set_pos(12, 3);
    pcd8544_printf("%dppb", tvoc);

    pcd8544_set_pos(78, 0);
    switch (icon) {
        case DISPLAY_ICON_BLUETOOH:
            pcd8544_puts("\3");
            break;
        default:
            break;
    }
}

void display_print_ext_value(float pressure, float temperature, float humidity, display_icon_t icon)
{
    pcd8544_set_pos(0, 0);
    pcd8544_draw_bitmap(out_icon, 7, 6, false);
    pcd8544_finalize_frame_buf();

    pcd8544_set_pos(12, 0);
    pcd8544_printf("%.2f\1C", temperature);

    pcd8544_set_pos(12, 1);
    pcd8544_printf("%.2f%%", humidity);

    pcd8544_set_pos(12, 2);
    pcd8544_printf("%.2fhPa", pressure);

    pcd8544_set_pos(78, 0);
    switch (icon) {
        case DISPLAY_ICON_BLUETOOH:
            pcd8544_puts("\3");
            break;
        default:
            break;
    }
}

void display_print_graph(sensor_id_t sensor_id, display_value_t value, float data[HISTORY_SIZE])
{
    uint16_t unit_size = value_unit_size(value);
    float min_val;
    float max_val;
    get_min_max(&min_val, &max_val, data);
    int16_t min_bound = (int16_t) floorf(min_val);
    int16_t max_bound = (int16_t) ceilf(max_val);
    min_bound -= min_bound > 0 ? min_bound % unit_size : unit_size - abs(min_bound) % unit_size;
    max_bound += max_bound < 0 ? abs(max_bound) % unit_size : unit_size - max_bound % unit_size;
    int16_t range = max_bound - min_bound;
    if (range == unit_size) {
        float middle = get_middle(data);
        if (max_bound - middle > middle - min_bound) {
            min_bound -= unit_size;
        } else {
            max_bound += unit_size;
        }
        range = 2 * unit_size;
    }

    for (uint8_t x = 0; x < HISTORY_SIZE - 1; x++) {
        if (!isnan(data[x]) && !isnan(data[x + 1])) pcd8544_draw_line(GRAPH_START_X + x + 0,
        GRAPH_END_Y - ((data[x] - min_bound) / range) * GRAPH_HEIGHT,
        GRAPH_START_X + x + 1,
        GRAPH_END_Y - ((data[x + 1] - min_bound) / range) * GRAPH_HEIGHT);
    }

    /* value axis */
    pcd8544_draw_line(GRAPH_END_X, GRAPH_START_Y, GRAPH_END_X, GRAPH_END_Y);
    for (uint8_t i = 1; i < range / unit_size; i++) {
        uint8_t y = GRAPH_END_Y - ((float) (i * unit_size) / range) * GRAPH_HEIGHT;
        pcd8544_draw_line(GRAPH_END_X + 1, y, GRAPH_END_X + 3, y);
    }

    /* time axis */
    for (uint8_t i = 1; i < GRAPH_WIDTH / 3; i++) {
        uint8_t x = GRAPH_START_X + i * 3;
        pcd8544_draw_line(x, GRAPH_END_Y, x + 2, GRAPH_END_Y);
    }

    pcd8544_set_pos(0, 0);
    pcd8544_draw_bitmap(sensor_id == SENSOR_ID_INTERNAL ? ind_icon : out_icon, 7, 6, false);
    pcd8544_finalize_frame_buf();

    pcd8544_set_pos(12, 0);
    pcd8544_printf("%d\2%d%s", min_bound, max_bound, value_unit(value));
}

void display_print_min_max(sensor_id_t sensor_id, display_value_t value, float data[HISTORY_SIZE])
{
    pcd8544_set_pos(0, 0);
    pcd8544_draw_bitmap(sensor_id == SENSOR_ID_INTERNAL ? ind_icon : out_icon, 7, 6, false);
    pcd8544_finalize_frame_buf();

    float max_val;
    float min_val;
    get_min_max(&min_val, &max_val, data);

    char *unit = value_unit(value);

    pcd8544_set_pos(12, 0);
    pcd8544_puts("MAX");
    pcd8544_set_pos(12, 1);
    pcd8544_printf("%.2f%s", max_val, unit);

    pcd8544_set_pos(12, 3);
    pcd8544_puts("MIN");
    pcd8544_set_pos(12, 4);
    pcd8544_printf("%.2f%s", min_val, unit);
}

void display_print_pairing(uint32_t passkey, bool blink)
{
    pcd8544_set_pos(0, 0);
    pcd8544_puts("Passkey");

    pcd8544_set_pos(0, 1);
    pcd8544_printf("%06d", passkey);

    if (blink) {
        pcd8544_set_pos(78, 0);
        pcd8544_puts("\3");
    }
}

void display_print_paired_result(bool successfully)
{
    if (successfully) {
        pcd8544_set_pos(0, 0);
        pcd8544_puts("Device");
        pcd8544_set_pos(0, 1);
        pcd8544_puts("successfully");
        pcd8544_set_pos(0, 2);
        pcd8544_puts("paired");
    } else {
        pcd8544_set_pos(0, 0);
        pcd8544_puts("No device");
        pcd8544_set_pos(0, 1);
        pcd8544_puts("paired");
    }
}
