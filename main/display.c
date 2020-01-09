#include <math.h>
#include <string.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "u8g2.h"

#include "display.h"

#define SPI_MOSI        13
#define SPI_SCLK        14
#define SPI_CS          27
#define SPI_RESET       25

#define GRAPH_START_X 10
#define GRAPH_START_Y 8
#define GRAPH_WIDTH   HISTORY_SIZE
#define GRAPH_HEIGHT    48
#define GRAPH_END_X GRAPH_START_X + GRAPH_WIDTH
#define GRAPH_END_Y GRAPH_START_Y + GRAPH_HEIGHT
#define GRAPH_DELTA_TIME    (24 * 3600) //sec

const char *TAG = "display";

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

// @formatter:on

static u8g2_t u8g2;

uint8_t u8g2_esp32_spi_byte_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
uint8_t u8g2_esp32_gpio_and_delay_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);

static spi_device_handle_t spi_handle;

void display_init()
{
    u8g2_Setup_st7920_s_128x64_f(&u8g2, U8G2_R0, u8g2_esp32_spi_byte_cb, u8g2_esp32_gpio_and_delay_cb);

    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0);
}

void display_clear()
{
    u8g2_ClearBuffer(&u8g2);
}

void display_sync()
{
    u8g2_SendBuffer(&u8g2);
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

uint16_t value_unit_size(display_value_t value)
{
    switch (value) {
        case DISPLAY_VALUE_TEMPERATURE:
        case DISPLAY_VALUE_OUT_TEMPERATURE:
            return 5;
        case DISPLAY_VALUE_HUMIDITY:
        case DISPLAY_VALUE_OUT_HUMIDITY:
            return 10;
        case DISPLAY_VALUE_PRESSURE:
            return 10;
        case DISPLAY_VALUE_IAQ:
            return 20;
        default:
            return 0;
    }
}

void draw_frame(uint8_t x, uint8_t y, uint8_t w, uint8_t h, const char *title)
{
    uint8_t title_w = strlen(title) * 6;

    u8g2_DrawRFrame(&u8g2, x, y + 2, w, h - 2, 3);
    u8g2_DrawRFrame(&u8g2, x + 2, y + 4, w - 4, h - 6, 2);

    u8g2_SetDrawColor(&u8g2, 0);
    u8g2_DrawBox(&u8g2, x + w / 2 - title_w / 2 - 3, y + 2, title_w + 5, 3);
    u8g2_SetDrawColor(&u8g2, 1);

    u8g2_DrawStr(&u8g2, x + w / 2 - title_w / 2, y + 7, title);
}

void print_string(uint8_t x, uint8_t y, uint8_t w, char *value, bool selected)
{
    if (selected) {
        u8g2_DrawBox(&u8g2, x - 1, y - 8, w + 1, 9);
        u8g2_SetDrawColor(&u8g2, 2);
    }

    u8g2_DrawStr(&u8g2, x, y, value);

    if (selected) {
        u8g2_SetDrawColor(&u8g2, 1);
    }
}

size_t utf8len(const char *s)
{
    size_t len = 0;
    while (*s)
        len += (*(s++) & 0xC0) != 0x80;
    return len;
}

void print_string_2(uint8_t x, uint8_t y, uint8_t w, char *value, char *unit, bool selected)
{
    if (selected) {
        u8g2_DrawBox(&u8g2, x - 1, y - 8, w + 1, 9);
        u8g2_SetDrawColor(&u8g2, 2);
    }

    u8g2_DrawStr(&u8g2, x, y, value);
    u8g2_DrawUTF8(&u8g2, x + w - (6 * utf8len(unit)), y, unit);

    if (selected) {
        u8g2_SetDrawColor(&u8g2, 1);
    }
}

void display_print_main(float temperature, float humidity, float iaq, float pressure, float out_temperature,
        float out_humidity, bool bt_connection, display_value_t selected_value)
{
    if (bt_connection) {
        u8g2_SetFont(&u8g2, u8g2_font_open_iconic_embedded_1x_t);
        u8g2_DrawGlyph(&u8g2, 120, 8, 0x4a);
    }

    u8g2_SetFont(&u8g2, u8g2_font_6x12_tf);
    char str[32];

    time_t t = time(NULL);
    struct tm tm = *localtime(&t);

    sprintf(str, "%02d:%02d:%02d", tm.tm_hour, tm.tm_min, tm.tm_sec);
    u8g2_DrawStr(&u8g2, 0, 7, str);

    draw_frame(0, 10, 63, 54, "IN");
    draw_frame(65, 10, 63, 54, "OUT");

    u8g2_SetFontMode(&u8g2, 1);

    sprintf(str, "%.2f", temperature);
    print_string_2(5, 27, 54, str, "°C", selected_value == DISPLAY_VALUE_TEMPERATURE);

    sprintf(str, "%.2f", humidity);
    print_string_2(5, 37, 54, str, "%", selected_value == DISPLAY_VALUE_HUMIDITY);

    sprintf(str, "%.0f", roundf(pressure));
    print_string_2(5, 47, 54, str, "hPa", selected_value == DISPLAY_VALUE_PRESSURE);

    sprintf(str, "%.0f", roundf(iaq));
    print_string_2(5, 57, 54, str, "IAQ", selected_value == DISPLAY_VALUE_IAQ);

    sprintf(str, "%.2f", out_temperature);
    print_string_2(70, 27, 54, str, "°C", selected_value == DISPLAY_VALUE_OUT_TEMPERATURE);

    sprintf(str, "%.2f", out_humidity);
    print_string_2(70, 37, 54, str, "%", selected_value == DISPLAY_VALUE_OUT_HUMIDITY);

}

void display_print_graph(display_value_t value, float data[HISTORY_SIZE])
{
    char str[32];
    uint16_t unit_size = 10; //value_unit_size(value);
    float min_val;
    float max_val;
    get_min_max(&min_val, &max_val, data);
    int min_bound = (int) floorf(min_val);
    int max_bound = (int) ceilf(max_val);

    if (isnan(min_val)) {
        u8g2_SetFont(&u8g2, u8g2_font_6x12_tf);
        u8g2_DrawUTF8(&u8g2, 4, 35, "No data recorded yet");
    } else {
        min_bound -= min_bound >= 0 ? min_bound % unit_size : (unit_size - abs(min_bound) % unit_size) % unit_size;
        max_bound += max_bound < 0 ? abs(max_bound) % unit_size : (unit_size - max_bound % unit_size) % unit_size;
        int range = max_bound - min_bound;
        if (range == 0) {
            max_bound += unit_size;
            range = unit_size;
        }

        for (uint8_t x = 0; x < HISTORY_SIZE - 1; x++) {
            if (!isnan(data[x]) && !isnan(data[x + 1])) u8g2_DrawLine(&u8g2, GRAPH_START_X + x + 0,
            GRAPH_END_Y - ((data[x] - min_bound) / range) * GRAPH_HEIGHT,
            GRAPH_START_X + x + 1,
            GRAPH_END_Y - ((data[x + 1] - min_bound) / range) * GRAPH_HEIGHT);
        }

        u8g2_SetFont(&u8g2, u8g2_font_5x7_mf);
        switch (value) {
            case DISPLAY_VALUE_TEMPERATURE:
            case DISPLAY_VALUE_OUT_TEMPERATURE:
                sprintf(str, "24H [%.2f %.2f]°C", min_val, max_val);
                break;
            case DISPLAY_VALUE_HUMIDITY:
            case DISPLAY_VALUE_OUT_HUMIDITY:
                sprintf(str, "24H [%.2f %.2f]%%", min_val, max_val);
                break;
            case DISPLAY_VALUE_PRESSURE:
                sprintf(str, "24H [%.0f %.0f]hPa", roundf(min_val), roundf(max_val));
                break;
            default:  //DISPLAY_VALUE_IAQ:
                sprintf(str, "24H [%.0f %.0f]", roundf(min_val), roundf(max_val));
                break;
        }
        u8g2_DrawUTF8(&u8g2, 0, 6, str);

        uint8_t vert_parts = 6;  //max 5 vertical lines
        while (range % vert_parts > 0 || (range / vert_parts) % 5 > 0) {
            vert_parts--;
            if (vert_parts == 2) break;
        }

        for (uint8_t i = 0; i <= vert_parts; i++) {
            uint8_t y = GRAPH_END_Y - (i * (float) (range / vert_parts) / range) * GRAPH_HEIGHT;

            sprintf(str, "%d", min_bound + i * (range / vert_parts));
            u8g2_DrawStr(&u8g2, GRAPH_END_X + 7, y + 3, str);

            for (uint8_t x = GRAPH_START_X; x < GRAPH_END_X - 1; x += 3) {
                u8g2_DrawPixel(&u8g2, x, y);
            }
        }

        time_t end_time = time(NULL);
        time_t start_time = end_time - GRAPH_DELTA_TIME;

        struct tm tm = *localtime(&start_time);
        tm.tm_hour -= tm.tm_hour % 6;
        tm.tm_min = 0;
        tm.tm_sec = 0;

        time_t time = mktime(&tm);

        if (time < start_time) {
            time += 6 * 3600;
        }
        while (time < end_time) {
            uint8_t x = GRAPH_START_X + ((time - start_time) / (float) GRAPH_DELTA_TIME) * GRAPH_WIDTH;
            for (uint8_t y = GRAPH_START_Y + 3; y < GRAPH_END_Y - 1; y += 3) {
                u8g2_DrawPixel(&u8g2, x, y);
            }

            tm = *localtime(&time);
            sprintf(str, "%02d", tm.tm_hour);
            u8g2_DrawStr(&u8g2, x - 4, 64, str);

            time += 6 * 3600;
        }
    }
}

void display_settins(display_setting_t selected)
{
    u8g2_SetFont(&u8g2, u8g2_font_6x12_tf);

    draw_frame(0, 00, 128, 64, "SETTINGS");

    print_string(5, 17, 123, "Set time", selected == DISPLAY_SETTING_TIME);

    print_string(5, 27, 123, "BLE start pairing", selected == DISPLAY_SETTING_BT_START_PAIRING);

    print_string(5, 37, 123, "BLE remove paired", selected == DISPLAY_SETTING_BT_REMOVE_PAIRED_DEVICES);

    print_string(5, 47, 123, "Back", selected == DISPLAY_SETTING_BACK);
}

void display_time(uint8_t hours_offset, uint8_t minutes_offset, uint8_t seconds_offset, display_time_t selected)
{
    u8g2_SetFont(&u8g2, u8g2_font_6x12_tf);

    draw_frame(0, 00, 128, 64, "TIME");

    time_t t = time(NULL);
    struct tm tm = *localtime(&t);

    char str[8];

    sprintf(str, "%02d", (tm.tm_hour + hours_offset) % 24);
    print_string(5, 17, 13, str, selected == DISPLAY_TIME_HOURS);

    u8g2_DrawStr(&u8g2, 18, 17, ":");

    sprintf(str, "%02d", (tm.tm_min + minutes_offset) % 60);
    print_string(24, 17, 13, str, selected == DISPLAY_TIME_MINUTES);

    u8g2_DrawStr(&u8g2, 36, 17, ":");

    sprintf(str, "%02d", (tm.tm_sec + seconds_offset) % 60);
    print_string(43, 17, 13, str, selected == DISPLAY_TIME_SECONDS);

    print_string(5, 27, 123, "Set", selected == DISPLAY_TIME_SET);

    print_string(5, 37, 123, "Back", selected == DISPLAY_TIME_BACK);
}

void display_print_pairing(uint32_t passkey, bool blink)
{
    u8g2_SetFont(&u8g2, u8g2_font_6x12_tf);

    draw_frame(0, 00, 128, 64, "BLE PAIRING");

    u8g2_DrawStr(&u8g2, 5, 17, "Passkey");

    char str[8];
    sprintf(str, "%06d", passkey);
    u8g2_DrawStr(&u8g2, 5, 27, str);
}

void display_print_paired_result(bool successfully)
{
    u8g2_SetFont(&u8g2, u8g2_font_6x12_tf);

    draw_frame(0, 00, 128, 64, "BLE PAIRING");

    if (successfully) {
        u8g2_DrawStr(&u8g2, 5, 17, "Device successfully");
        u8g2_DrawStr(&u8g2, 5, 27, "paired");
    } else {
        u8g2_DrawStr(&u8g2, 5, 17, "No devices paired");
    }
//    if (successfully) {
//        pcd8544_set_pos(0, 0);
//        pcd8544_puts("Device");
//        pcd8544_set_pos(0, 1);
//        pcd8544_puts("successfully");
//        pcd8544_set_pos(0, 2);
//        pcd8544_puts("paired");
//    } else {
//        pcd8544_set_pos(0, 0);
//        pcd8544_puts("No device");
//        pcd8544_set_pos(0, 1);
//        pcd8544_puts("paired");
//    }
}

uint8_t u8g2_esp32_spi_byte_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    switch (msg) {
        case U8X8_MSG_BYTE_INIT: {
            spi_bus_config_t bus_config = { 0 };
            bus_config.sclk_io_num = SPI_SCLK;
            bus_config.mosi_io_num = SPI_MOSI;
            bus_config.miso_io_num = -1;
            bus_config.quadwp_io_num = -1;
            bus_config.quadhd_io_num = -1;
            bus_config.flags = (SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_SCLK | SPICOMMON_BUSFLAG_MOSI);
            ESP_ERROR_CHECK(spi_bus_initialize(VSPI_HOST, &bus_config, 1));

            spi_device_interface_config_t dev_config = { 0 };
            dev_config.clock_speed_hz = 1 * 1000 * 1000;
            dev_config.spics_io_num = SPI_CS;
            dev_config.flags = SPI_DEVICE_POSITIVE_CS;
            dev_config.queue_size = 1;
            ESP_ERROR_CHECK(spi_bus_add_device(VSPI_HOST, &dev_config, &spi_handle));
            break;
        }
        case U8X8_MSG_BYTE_SEND: {
            spi_transaction_t trans_desc;
            trans_desc.addr = 0;
            trans_desc.cmd = 0;
            trans_desc.flags = 0;
            trans_desc.length = 8 * arg_int;
            trans_desc.rxlength = 0;
            trans_desc.tx_buffer = arg_ptr;
            trans_desc.rx_buffer = NULL;
            ESP_ERROR_CHECK(spi_device_transmit(spi_handle, &trans_desc));
            break;
        }
    }
    return 1;
}

uint8_t u8g2_esp32_gpio_and_delay_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    switch (msg) {
        case U8X8_MSG_GPIO_AND_DELAY_INIT: {
            gpio_config_t gpioConfig;
            gpioConfig.pin_bit_mask = 1ULL << SPI_RESET;
            gpioConfig.mode = GPIO_MODE_OUTPUT;
            gpioConfig.pull_up_en = GPIO_PULLUP_DISABLE;
            gpioConfig.pull_down_en = GPIO_PULLDOWN_ENABLE;
            gpioConfig.intr_type = GPIO_INTR_DISABLE;
            gpio_config(&gpioConfig);
            break;
        }
        case U8X8_MSG_GPIO_RESET:
            gpio_set_level(SPI_RESET, arg_int);
            break;
        case U8X8_MSG_DELAY_MILLI:
            vTaskDelay(arg_int / portTICK_PERIOD_MS);
            //ets_delay_us(arg_int * 1000);
            break;
    }
    return 1;
}
