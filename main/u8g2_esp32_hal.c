#include <stdio.h>
#include <string.h>

#include "sdkconfig.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "u8g2_esp32_hal.h"

static const char *TAG = "u8g2_hal";

static spi_device_handle_t handle_spi;      // SPI handle.
static u8g2_esp32_hal_t u8g2_esp32_hal;  // HAL state data.

//#undef ESP_ERROR_CHECK
//#define ESP_ERROR_CHECK(x)   do { esp_err_t rc = (x); if (rc != ESP_OK) { ESP_LOGE("err", "esp_err_t = %d", rc); assert(0 && #x);} } while(0);

/*
 * Initialze the ESP32 HAL.
 */
void u8g2_esp32_hal_init(u8g2_esp32_hal_t u8g2_esp32_hal_param)
{
    u8g2_esp32_hal = u8g2_esp32_hal_param;
} // u8g2_esp32_hal_init

/*
 * HAL callback function as prescribed by the U8G2 library.  This callback is invoked
 * to handle SPI communications.
 */
uint8_t u8g2_esp32_spi_byte_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    ESP_LOGD(TAG, "spi_byte_cb: Received a msg: %d, arg_int: %d, arg_ptr: %p", msg, arg_int, arg_ptr);
    switch (msg) {
        case U8X8_MSG_BYTE_SET_DC:
            ESP_LOGI(TAG, "set DC %d", arg_int);
            if (u8g2_esp32_hal.dc != U8G2_ESP32_HAL_UNDEFINED) {
                gpio_set_level(u8g2_esp32_hal.dc, arg_int);
            }
            break;

        case U8X8_MSG_BYTE_INIT: {
            if (u8g2_esp32_hal.clk == U8G2_ESP32_HAL_UNDEFINED || u8g2_esp32_hal.mosi == U8G2_ESP32_HAL_UNDEFINED
                    || u8g2_esp32_hal.cs == U8G2_ESP32_HAL_UNDEFINED) {
                ESP_LOGI(TAG, "... Fuck. %d %d %d", u8g2_esp32_hal.clk, u8g2_esp32_hal.mosi, u8g2_esp32_hal.cs);
                break;
            }

            u8x8_gpio_SetCS(u8x8, u8x8->display_info->chip_disable_level);

            spi_bus_config_t bus_config = { 0 };
            bus_config.sclk_io_num = u8g2_esp32_hal.clk; // CLK
            bus_config.mosi_io_num = u8g2_esp32_hal.mosi; // MOSI
            bus_config.miso_io_num = -1; // MISO
            bus_config.quadwp_io_num = -1; // Not used
            bus_config.quadhd_io_num = -1; // Not used
            bus_config.flags = (SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_SCLK | SPICOMMON_BUSFLAG_MOSI);
            ESP_ERROR_CHECK(spi_bus_initialize(VSPI_HOST, &bus_config, 1));

            spi_device_interface_config_t dev_config = { 0 };
            dev_config.clock_speed_hz = 1*1000*1000;
            dev_config.spics_io_num = u8g2_esp32_hal.cs;
            dev_config.flags        = SPI_DEVICE_POSITIVE_CS;
            dev_config.queue_size = 1;

            ESP_ERROR_CHECK(spi_bus_add_device(VSPI_HOST, &dev_config, &handle_spi));
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
            ESP_ERROR_CHECK(spi_device_transmit(handle_spi, &trans_desc));
            break;
        }
    }
    return 1;
}

/*
 * HAL callback function as prescribed by the U8G2 library.  This callback is invoked
 * to handle callbacks for GPIO and delay functions.
 */
uint8_t u8g2_esp32_gpio_and_delay_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    ESP_LOGD(TAG, "gpio_and_delay_cb: Received a msg: %d, arg_int: %d, arg_ptr: %p", msg, arg_int, arg_ptr);

    switch (msg) {
        // Initialize the GPIO and DELAY HAL functions.  If the pins for DC and RESET have been
        // specified then we define those pins as GPIO outputs.
        case U8X8_MSG_GPIO_AND_DELAY_INIT: {
            uint64_t bitmask = 0;
            if (u8g2_esp32_hal.dc != U8G2_ESP32_HAL_UNDEFINED) {
                bitmask = bitmask | (1ull << u8g2_esp32_hal.dc);
            }
            if (u8g2_esp32_hal.reset != U8G2_ESP32_HAL_UNDEFINED) {
                bitmask = bitmask | (1ull << u8g2_esp32_hal.reset);
            }
            if (u8g2_esp32_hal.cs != U8G2_ESP32_HAL_UNDEFINED) {
                bitmask = bitmask | (1ull << u8g2_esp32_hal.cs);
            }
            if (u8g2_esp32_hal.clk != U8G2_ESP32_HAL_UNDEFINED) {
                bitmask = bitmask | (1ull << u8g2_esp32_hal.clk);
            }
            if (u8g2_esp32_hal.mosi != U8G2_ESP32_HAL_UNDEFINED) {
                bitmask = bitmask | (1ull << u8g2_esp32_hal.mosi);
            }

            if (bitmask == 0) {
                break;
            }
            gpio_config_t gpioConfig;
            gpioConfig.pin_bit_mask = bitmask;
            gpioConfig.mode = GPIO_MODE_OUTPUT;
            gpioConfig.pull_up_en = GPIO_PULLUP_DISABLE;
            gpioConfig.pull_down_en = GPIO_PULLDOWN_ENABLE;
            gpioConfig.intr_type = GPIO_INTR_DISABLE;
            gpio_config(&gpioConfig);
            break;
        }

        case U8X8_MSG_GPIO_DC:
            ESP_LOGI(TAG, "set DC %d", arg_int);
            if (u8g2_esp32_hal.dc != U8G2_ESP32_HAL_UNDEFINED) {
                gpio_set_level(u8g2_esp32_hal.reset, arg_int);
            }
            break;

            // Set the GPIO reset pin to the value passed in through arg_int.
        case U8X8_MSG_GPIO_RESET:
            ESP_LOGI(TAG, "set RST %d", arg_int);
            if (u8g2_esp32_hal.reset != U8G2_ESP32_HAL_UNDEFINED) {
                gpio_set_level(u8g2_esp32_hal.reset, arg_int);
            }
            break;
            // Set the GPIO client select pin to the value passed in through arg_int.
        case U8X8_MSG_GPIO_CS:
            ESP_LOGI(TAG, "set CS %d", arg_int);
            if (u8g2_esp32_hal.cs != U8G2_ESP32_HAL_UNDEFINED) {
                gpio_set_level(u8g2_esp32_hal.cs, arg_int);
            }
            break;
            // Set the Software I²C pin to the value passed in through arg_int.

        case U8X8_MSG_GPIO_SPI_CLOCK:
            if (u8g2_esp32_hal.clk != U8G2_ESP32_HAL_UNDEFINED) {
                // printf("C%d ", arg_int);
                gpio_set_level(u8g2_esp32_hal.clk, arg_int);
            }
            break;
        case U8X8_MSG_GPIO_SPI_DATA:
            if (u8g2_esp32_hal.mosi != U8G2_ESP32_HAL_UNDEFINED) {
                //printf("D%d ", arg_int);
                gpio_set_level(u8g2_esp32_hal.mosi, arg_int);
            }
            break;
            // Delay for the number of milliseconds passed in through arg_int.
        case U8X8_MSG_DELAY_MILLI:
            //vTaskDelay(arg_int/portTICK_PERIOD_MS);
            ets_delay_us(arg_int * 1000);
            break;
        case U8X8_MSG_DELAY_NANO:
      //      ESP_LOGI(TAG, "U8X8_MSG_DELAY_NANO %d", arg_int);
            ets_delay_us(arg_int);
            break;
        default:
            ESP_LOGI(TAG, "not handled ev %d", msg);
    }
    return 1;
} // u8g2_esp32_gpio_and_delay_cb
