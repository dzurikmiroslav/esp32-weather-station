#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "rom/ets_sys.h"
#include "bme280.h"

#include "bme.h"
#include "i2c.h"

#define LOG_TAG "BME"

struct bme280_dev bme280;

static int8_t bme_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    return i2c_write(dev_id, reg_addr, reg_data, len);
}

static int8_t bme_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    return i2c_read(dev_id, reg_addr, reg_data, len);
}

static void bme_delay_ms(uint32_t period)
{
    ets_delay_us(period * 1000);
}

void bme_init()
{
    bme280.write = bme_i2c_write;
    bme280.read = bme_i2c_read;
    bme280.delay_ms = bme_delay_ms;
    bme280.intf = BME280_I2C_INTF;
    bme280.dev_id = BME280_I2C_ADDR_PRIM;
    ESP_ERROR_CHECK(bme280_init(&bme280));

    bme280.settings.osr_h = BME280_OVERSAMPLING_1X;
    bme280.settings.osr_t = BME280_OVERSAMPLING_2X;
    bme280.settings.filter = BME280_FILTER_COEFF_16;
    uint8_t settings_sel = BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;
    ESP_ERROR_CHECK(bme280_set_sensor_settings(settings_sel, &bme280));
}

bme_data_t bme_read()
{
    bme_data_t ret;
    struct bme280_data data;

    ESP_ERROR_CHECK(bme280_set_sensor_mode(BME280_FORCED_MODE, &bme280));
    vTaskDelay(70 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(bme280_get_sensor_data(BME280_TEMP | BME280_HUM, &data, &bme280));

    ret.humidity = (data.humidity / 1024.0f) * 100;
    ret.temperature = data.temperature;

    ESP_LOGD(LOG_TAG, "Temperature %d *0.01 °C", data.temperature);
    ESP_LOGD(LOG_TAG, "Humidity %d *0.01 %%", data.humidity);

    return ret;
}
