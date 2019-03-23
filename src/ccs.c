#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

#include "ccs.h"
#include "i2c.h"

#define LOG_TAG "CCS"

#define I2C_MASTER_NUM 1
#define WRITE_BIT I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ   /*!< I2C master read */
#define ACK_CHECK_EN 0x1           /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0          /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                /*!< I2C ack value */
#define NACK_VAL 0x1               /*!< I2C nack value */

// Default address of the device is 0x5B
#define CCS811_ADDRESS 0x5A

/* CCS811 register addresses */
#define CCS811_REG_STATUS 0x00
#define CCS811_REG_MEAS_MODE 0x01
#define CCS811_REG_ALG_RESULT_DATA 0x02
#define CCS811_REG_RAW_DATA 0x03
#define CCS811_REG_ENV_DATA 0x05
#define CCS811_REG_NTC 0x06
#define CCS811_REG_THRESHOLDS 0x10
#define CCS811_REG_BASELINE 0x11

#define CCS811_REG_HW_ID 0x20
#define CCS811_REG_HW_VER 0x21
#define CCS811_REG_FW_BOOT_VER 0x23
#define CCS811_REG_FW_APP_VER 0x24

#define CCS811_REG_ERROR_ID 0xe0

#define CCS811_REG_APP_ERASE 0xf1
#define CCS811_REG_APP_DATA 0xf2
#define CCS811_REG_APP_VERIFY 0xf3
#define CCS811_REG_APP_START 0xf4
#define CCS811_REG_SW_RESET 0xff

// status register bits
#define CCS811_STATUS_ERROR 0x01     // error, details in CCS811_REG_ERROR
#define CCS811_STATUS_DATA_RDY 0x08  // new data sample in ALG_RESULT_DATA
#define CCS811_STATUS_APP_VALID 0x10 // valid application firmware loaded
#define CCS811_STATUS_FW_MODE 0x80   // firmware is in application mode

// error register bits
#define CCS811_ERR_WRITE_REG_INV 0x01  // invalid register address on write
#define CCS811_ERR_READ_REG_INV 0x02   // invalid register address on read
#define CCS811_ERR_MEASMODE_INV 0x04   // invalid requested measurement mode
#define CCS811_ERR_MAX_RESISTANCE 0x08 // maximum sensor resistance exceeded
#define CCS811_ERR_HEATER_FAULT 0x10   // heater current not in range
#define CCS811_ERR_HEATER_SUPPLY 0x20  // heater voltage not applied correctly

// modes
#define CCS811_MODE_IDLE 0 // Idle, low current mode
#define CCS811_MODE_1S 1   // Constant Power mode, IAQ values every 1 s
#define CCS811_MODE_10S 2  // Pulse Heating mode, IAQ values every 10 s
#define CCS811_MODE_60S 3  // Low Power Pulse Heating, IAQ values every 60 s
#define CCS811_MODE_250MS 4

#define CCS811_HW_ID_CODE 0x81

void ccs_init()
{
    const static uint8_t sw_reset[4] = {0x11, 0xe5, 0x72, 0x8a};
    ESP_ERROR_CHECK(i2c_write(CCS811_ADDRESS, CCS811_REG_SW_RESET, (uint8_t *)sw_reset, 4));

    vTaskDelay(100 / portTICK_PERIOD_MS);

    uint8_t hw_id;
    ESP_ERROR_CHECK(i2c_read(CCS811_ADDRESS, CCS811_REG_HW_ID, &hw_id, 1));
    ESP_LOGD(LOG_TAG, "HW id %04x", hw_id);
    ESP_ERROR_CHECK(hw_id != 0x81);

    uint8_t hw_version;
    ESP_ERROR_CHECK(i2c_read(CCS811_ADDRESS, CCS811_REG_HW_VER, &hw_version, 1));
    ESP_LOGD(LOG_TAG, "HW version %04x", hw_version);
    ESP_ERROR_CHECK((hw_version & 0xF0) != 0x10);

    uint8_t fw_boot_version[2];
    ESP_ERROR_CHECK(i2c_read(CCS811_ADDRESS, CCS811_REG_FW_BOOT_VER, fw_boot_version, 2));
    ESP_LOGD(LOG_TAG, "Bootloader version %04x", fw_boot_version[0] * 256 + fw_boot_version[1]);

    uint8_t fw_app_version[2];
    ESP_ERROR_CHECK(i2c_read(CCS811_ADDRESS, CCS811_REG_FW_APP_VER, fw_app_version, 2));
    ESP_LOGD(LOG_TAG, "Application version %04x", fw_app_version[0] * 256 + fw_app_version[1]);

    uint8_t status;
    ESP_ERROR_CHECK(i2c_read(CCS811_ADDRESS, CCS811_REG_STATUS, &status, 1));
    ESP_LOGD(LOG_TAG, "Status %d", status);

    if (!(status & CCS811_STATUS_FW_MODE))
    {
        ESP_LOGD(LOG_TAG, "Switch to application mode");
        ESP_ERROR_CHECK(!(status & CCS811_STATUS_APP_VALID));

        uint8_t empty[0] = {};
        ESP_ERROR_CHECK(i2c_write(CCS811_ADDRESS, CCS811_REG_APP_START, empty, 0));
        vTaskDelay(100 / portTICK_PERIOD_MS);

        ESP_ERROR_CHECK(i2c_read(CCS811_ADDRESS, CCS811_REG_STATUS, &status, 1));
        ESP_LOGD(LOG_TAG, "Status %d", status);
        ESP_ERROR_CHECK(!(status & CCS811_STATUS_FW_MODE));
    }

    uint8_t meas_mode = (CCS811_MODE_10S << 4) | (1 << 3) | (0 << 2);
    ESP_ERROR_CHECK(i2c_write(CCS811_ADDRESS, CCS811_REG_MEAS_MODE, &meas_mode, 1));  
}

void ccs_set_env_data(float humidity, float temperature)
{
    // if ((humidity < 0) || (humidity > 100))
    // {
    //     ESP_LOGD(LOG_TAG, "Humidity out of range");
    //     return;
    // }

    // if ((temperature < -25) || (temperature > 50))
    // {
    //     ESP_LOGD(LOG_TAG, "Temperature out of range");
    //     return;
    // }

    // uint32_t rH = humidity * 1000;
    // uint32_t temp = temperature * 1000;

    // uint8_t data[4];

    // //Split value into 7-bit integer and 9-bit fractional
    // data[0] = ((rH % 1000) / 100) > 7 ? (rH / 1000 + 1) << 1 : (rH / 1000) << 1;
    // data[1] = 0; //CCS811 only supports increments of 0.5 so bits 7-0 will always be zero
    // if (((rH % 1000) / 100) > 2 && (((rH % 1000) / 100) < 8))
    // {
    //     data[0] |= 1; //Set 9th bit of fractional to indicate 0.5%
    // }

    // temp += 25000; //Add the 25C offset
    // //Split value into 7-bit integer and 9-bit fractional
    // data[2] = ((temp % 1000) / 100) > 7 ? (temp / 1000 + 1) << 1 : (temp / 1000) << 1;
    // data[3] = 0;
    // if (((temp % 1000) / 100) > 2 && (((temp % 1000) / 100) < 8))
    // {
    //     data[2] |= 1; //Set 9th bit of fractional to indicate 0.5C
    // }

    uint16_t temp = (temperature + 25) * 512; // -25 °C maps to 0
    uint16_t hum = humidity * 512;

    uint8_t data[4] = {temp >> 8, temp & 0xff,
                       hum >> 8, hum & 0xff};

    ESP_ERROR_CHECK(i2c_write(CCS811_ADDRESS, CCS811_REG_ENV_DATA, data, 4));
}

ccs_data_t ccs_read()
{
    ccs_data_t ret;

    uint8_t data[8];

    ESP_ERROR_CHECK(i2c_read(CCS811_ADDRESS, CCS811_REG_ALG_RESULT_DATA, data, 8));

    ret.co2 = (uint16_t)(data[0]) << 8 | data[1];
    ret.tvoc = (uint16_t)(data[2]) << 8 | data[3];

    ESP_LOGD(LOG_TAG, "eCO2 %d ppm", ret.co2);
    ESP_LOGD(LOG_TAG, "TVOC %d ppb", ret.tvoc);

    return ret;
}