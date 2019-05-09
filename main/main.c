#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_event_loop.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "driver/gpio.h"

#include "display.h"
#include "i2c.h"
#include "bme.h"
#include "ccs.h"
#include "ble.h"

#define LOG_TAG "MAIN"

#define ESPNOW_CHANNEL CONFIG_ESPNOW_CHANNEL
#define ESPNOW_SENSOR_MAC CONFIG_ESPNOW_SENSOR_MAC
#define ESPNOW_PMK CONFIG_ESPNOW_PMK
#define ESPNOW_LMK CONFIG_ESPNOW_LMK
#define ENABLE_BLE_SECURITY CONFIG_ENABLE_BLE_SECURITY

#define BUTTON_0            15
#define BUTTON_1            13
#define CCS811_INT          22
#define SCREN_TIMEOUT       10000
#define BUTTON_MIN_TRESHOLD 250
#define EXT_SENSOR_TIMEUT   6000000 /* 10min */
#define HISTORY_PERIOD      1200000 /* 20min */

typedef struct
{
    uint32_t pressure;
    int16_t temperature;
    uint16_t humidity;
    uint8_t battery;
}__attribute__((packed)) espnow_sensor_data_t;

typedef enum
{
    LCD_EVT_INT_VALUE, LCD_EVT_EXT_VALUE, LCD_EVT_CHANGE_SENSOR, LCD_EVT_CHANGE_SCREEN
} lcd_evt_t;

typedef struct
{
    uint16_t data[HISTORY_SIZE];
    uint8_t size;
    uint16_t shift;
    uint8_t scale;
    uint32_t avg_value;
    uint8_t avg_counter;
} history_array_t;

typedef struct
{
    float humidity;
    float temperature;
    uint16_t co2;
    uint16_t tvoc;
    struct
    {
        float humidity[HISTORY_SIZE];
        float temperature[HISTORY_SIZE];
        float co2[HISTORY_SIZE];
        float tvoc[HISTORY_SIZE];
        TickType_t write_time;
        float avg_humidity;
        float avg_temperature;
        float avg_co2;
        float avg_tvoc;
        uint16_t avg_counter;
    } history;
} int_data_t;

typedef struct
{
    float humidity;
    float pressure;
    float temperature;
    TickType_t write_time;
    struct
    {
        float humidity[HISTORY_SIZE];
        float pressure[HISTORY_SIZE];
        float temperature[HISTORY_SIZE];
        TickType_t write_time;
        float avg_humidity;
        float avg_pressure;
        float avg_temperature;
        uint16_t avg_counter;
    } history;
} ext_data_t;

static int_data_t int_data;
static ext_data_t ext_data;

static TaskHandle_t lcd_print_sensor_task;
static QueueHandle_t lcd_print_sensor_queue;

static TaskHandle_t ccs_read_task;
static QueueHandle_t ble_connection_queue;

#if ENABLE_BLE_SECURITY
static TaskHandle_t ble_reset_task;
static SemaphoreHandle_t ble_reset_mutex;
#endif /* ENABLE_BLE_SECURITY */

static esp_now_peer_info_t sensor_peer;

typedef struct
{
    TickType_t click_time;
    uint8_t button;
} button_handler_data_t;

// @formatter:off
static button_handler_data_t button_handler_data[2] = {
        { .button = BUTTON_0, },
        { .button = BUTTON_1, }
};
// @formatter:on

static void ext_data_push_history(ext_data_t *data)
{
    data->history.avg_humidity += data->humidity;
    data->history.avg_pressure += data->pressure;
    data->history.avg_temperature += data->temperature;
    data->history.avg_counter++;

    TickType_t now = xTaskGetTickCount();

    if (now >= (data->history.write_time + HISTORY_PERIOD / portTICK_PERIOD_MS)) {
        /* rotate history */
        for (int i = 0; i < HISTORY_SIZE - 1; i++) {
            data->history.humidity[i] = data->history.humidity[i + 1];
            data->history.pressure[i] = data->history.pressure[i + 1];
            data->history.temperature[i] = data->history.temperature[i + 1];
        }

        data->history.humidity[HISTORY_SIZE - 1] = data->history.avg_humidity / data->history.avg_counter;
        data->history.pressure[HISTORY_SIZE - 1] = data->history.avg_pressure / data->history.avg_counter;
        data->history.temperature[HISTORY_SIZE - 1] = data->history.avg_temperature / data->history.avg_counter;

        data->history.write_time = now;
        data->history.avg_humidity = 0.0f;
        data->history.avg_pressure = 0.0f;
        data->history.avg_temperature = 0.0f;
        data->history.avg_counter = 0;
    }
}

static void int_data_push_history(int_data_t *data)
{
    data->history.avg_humidity += data->humidity;
    data->history.avg_temperature += data->temperature;
    data->history.avg_co2 += data->co2;
    data->history.avg_tvoc += data->tvoc;
    data->history.avg_counter++;

    TickType_t now = xTaskGetTickCount();

    if (now >= (data->history.write_time + HISTORY_PERIOD / portTICK_PERIOD_MS)) {
        /* rotate history */
        for (int i = 0; i < HISTORY_SIZE - 1; i++) {
            data->history.humidity[i] = data->history.humidity[i + 1];
            data->history.temperature[i] = data->history.temperature[i + 1];
            data->history.co2[i] = data->history.co2[i + 1];
            data->history.tvoc[i] = data->history.tvoc[i + 1];
        }

        data->history.humidity[HISTORY_SIZE - 1] = data->history.avg_humidity / data->history.avg_counter;
        data->history.temperature[HISTORY_SIZE - 1] = data->history.avg_temperature / data->history.avg_counter;
        data->history.co2[HISTORY_SIZE - 1] = data->history.avg_co2 / data->history.avg_counter;
        data->history.tvoc[HISTORY_SIZE - 1] = data->history.avg_tvoc / data->history.avg_counter;

        data->history.write_time = now;
        data->history.avg_humidity = 0.0f;
        data->history.avg_temperature = 0.0f;
        data->history.avg_co2 = 0.0f;
        data->history.avg_tvoc = 0.0f;
        data->history.avg_counter = 0;
    }
}

static void lcd_print_sensor_task_func(void *param)
{
    lcd_screen_type_t screen = LCD_SCREEN_INT_FIRST;
    bool ble_connection = false;
    TickType_t sensor_change_time = 0;

    QueueSetHandle_t queue_set = xQueueCreateSet(20);
    xQueueAddToSet(lcd_print_sensor_queue, queue_set);
    xQueueAddToSet(ble_connection_queue, queue_set);

    while (true) {
        QueueSetMemberHandle_t queue_member = xQueueSelectFromSet(queue_set,
        portMAX_DELAY);

        bool refresh = false;
        if (queue_member == lcd_print_sensor_queue) {
            lcd_evt_t evt;
            xQueueReceive(lcd_print_sensor_queue, &evt, 0);
            TickType_t now = xTaskGetTickCount();
            bool change_sensor = now >= (sensor_change_time + SCREN_TIMEOUT / portTICK_PERIOD_MS);
            bool change_screen = false;
            bool new_data = false;

            if (now >= ext_data.write_time + EXT_SENSOR_TIMEUT / portTICK_PERIOD_MS) {
                ext_data.temperature = NAN;
                ext_data.humidity = NAN;
                ext_data.pressure = NAN;
                ext_data.write_time = now;
            }

            switch (evt) {
                case LCD_EVT_INT_VALUE:
                    new_data = LCD_SCREEN_IS_INT(screen);
                    break;
                case LCD_EVT_EXT_VALUE:
                    new_data = LCD_SCREEN_IS_EXT(screen);
                    break;
                case LCD_EVT_CHANGE_SENSOR:
                    change_sensor = true;
                    break;
                case LCD_EVT_CHANGE_SCREEN:
                    change_screen = true;
                    change_sensor = false;
                    break;
            }

            if (change_sensor) {
                if (LCD_SCREEN_IS_INT(screen)) {
                    screen = LCD_SCREEN_EXT_FIRST;
                } else {
                    screen = LCD_SCREEN_INT_FIRST;
                }
                sensor_change_time = now;
            }
            if (change_screen) {
                screen++;
                if (screen == LCD_SCREEN_INT_LAST + 1) {
                    screen = LCD_SCREEN_INT_FIRST;
                }
                if (screen == LCD_SCREEN_EXT_LAST + 1) {
                    screen = LCD_SCREEN_EXT_FIRST;
                }
                sensor_change_time = now;
            }

            refresh = change_sensor || change_screen || new_data;
        } else if (queue_member == ble_connection_queue) {
            xQueueReceive(ble_connection_queue, &ble_connection, 0);
            refresh = true;
        }

        if (refresh) {
            display_clear();
            switch (screen) {
                case LCD_SCREEN_TYPE_INT_MAIN:
                    display_print_int_value(int_data.temperature, int_data.humidity, int_data.co2, int_data.tvoc,
                            ble_connection);
                    break;
                case LCD_SCREEN_TYPE_INT_HUMIDITY_MIN_MAX:
                    display_print_min_max(SENSOR_ID_INTERNAL, DISPLAY_VALUE_HUMIDITY, int_data.history.humidity);
                    break;
                case LCD_SCREEN_TYPE_INT_TEMPERATURE_MIN_MAX:
                    display_print_min_max(SENSOR_ID_INTERNAL, DISPLAY_VALUE_TEMPERATURE, int_data.history.temperature);
                    break;
                case LCD_SCREEN_TYPE_INT_CO2_MIN_MAX:
                    display_print_min_max(SENSOR_ID_INTERNAL, DISPLAY_VALUE_CO2, int_data.history.co2);
                    break;
                case LCD_SCREEN_TYPE_INT_TVOC_MIN_MAX:
                    display_print_min_max(SENSOR_ID_INTERNAL, DISPLAY_VALUE_TVOC, int_data.history.tvoc);
                    break;
                case LCD_SCREEN_TYPE_INT_HUMIDITY_GRAPH:
                    display_print_graph(SENSOR_ID_INTERNAL, DISPLAY_VALUE_HUMIDITY, int_data.history.humidity);
                    break;
                case LCD_SCREEN_TYPE_INT_TEMPERATURE_GRAPH:
                    display_print_graph(SENSOR_ID_INTERNAL, DISPLAY_VALUE_TEMPERATURE, int_data.history.temperature);
                    break;
                case LCD_SCREEN_TYPE_INT_CO2_GRAPH:
                    display_print_graph(SENSOR_ID_INTERNAL, DISPLAY_VALUE_CO2, int_data.history.co2);
                    break;
                case LCD_SCREEN_TYPE_INT_TVOC_GRAPH:
                    display_print_graph(SENSOR_ID_INTERNAL, DISPLAY_VALUE_TVOC, int_data.history.tvoc);
                    break;
                case LCD_SCREEN_TYPE_EXT_MAIN:
                    display_print_ext_value(ext_data.pressure, ext_data.temperature, ext_data.humidity, ble_connection);
                    break;
                case LCD_SCREEN_TYPE_EXT_HUMIDITY_MIN_MAX:
                    display_print_min_max(SENSOR_ID_EXTERNAL, DISPLAY_VALUE_HUMIDITY, ext_data.history.humidity);
                    break;
                case LCD_SCREEN_TYPE_EXT_PRESSURE_MIN_MAX:
                    display_print_min_max(SENSOR_ID_EXTERNAL, DISPLAY_VALUE_PRESSURE, ext_data.history.pressure);
                    break;
                case LCD_SCREEN_TYPE_EXT_TEMPERATURE_MIN_MAX:
                    display_print_min_max(SENSOR_ID_EXTERNAL, DISPLAY_VALUE_TEMPERATURE, ext_data.history.temperature);
                    break;
                case LCD_SCREEN_TYPE_EXT_HUMIDITY_GRAPH:
                    display_print_graph(SENSOR_ID_EXTERNAL, DISPLAY_VALUE_HUMIDITY, ext_data.history.humidity);
                    break;
                case LCD_SCREEN_TYPE_EXT_PRESSURE_GRAPH:
                    display_print_graph(SENSOR_ID_EXTERNAL, DISPLAY_VALUE_PRESSURE, ext_data.history.pressure);
                    break;
                case LCD_SCREEN_TYPE_EXT_TEMPERATURE_GRAPH:
                    display_print_graph(SENSOR_ID_EXTERNAL, DISPLAY_VALUE_TEMPERATURE, ext_data.history.temperature);
                    break;
            }
            display_sync();
        }
    }
}

static void bme_read_task_func(void *param)
{
    lcd_evt_t evt = LCD_EVT_INT_VALUE;
    bme_data_t bme_data;

    while (true) {
        bme_data = bme_read();

        int_data.humidity = bme_data.humidity / 100.0f;
        int_data.temperature = bme_data.temperature / 100.0f;
        ble_set_int_humidity_temperature(int_data.humidity, int_data.temperature);
        int_data_push_history(&int_data);

        xQueueSend(lcd_print_sensor_queue, &evt, portMAX_DELAY);

        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

static void ccs_read_task_func(void *param)
{
    lcd_evt_t evt = LCD_EVT_INT_VALUE;
    ccs_data_t ccs_data;

    while (true) {
        vTaskSuspend(NULL);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        ccs_set_env_data(int_data.humidity, int_data.temperature);
        ccs_data = ccs_read();

        int_data.co2 = ccs_data.co2;
        int_data.tvoc = ccs_data.tvoc;
        ble_set_int_co2_tvoc(int_data.co2, int_data.tvoc);
        int_data_push_history(&int_data);

        xQueueSend(lcd_print_sensor_queue, &evt, portMAX_DELAY);
    }
}

#if ENABLE_BLE_SECURITY
static void ble_reset_task_func(void *param)
{
    while (true) {
        vTaskSuspend(NULL);

        if (!xSemaphoreTake(ble_reset_mutex, 3000 / portTICK_PERIOD_MS)) {
            ESP_LOGI(LOG_TAG, "BLE start pairing");
            vTaskSuspend(lcd_print_sensor_task);

            uint32_t passkey = (esp_random() / (float) UINT32_MAX) * 999999;
            ble_remove_paired_device();
            ble_enable_pairing(passkey);

            bool connection;
            bool paired;
            uint8_t countdown = 60;
            do {
                display_clear();
                display_print_pairing(passkey, countdown % 2);
                display_sync();
            } while (!(paired = xQueuePeek(ble_connection_queue, &connection, 1000 / portTICK_PERIOD_MS)) && countdown--);

            ESP_LOGI(LOG_TAG, "BLE stop pairing");
            ble_disable_pairing();

            display_clear();
            display_print_paired_result(paired);
            display_sync();
            vTaskDelay(3000 / portTICK_PERIOD_MS);

            xQueueReset(lcd_print_sensor_queue);
            vTaskResume(lcd_print_sensor_task);
        }
    }
}
#endif /* ENABLE_BLE_SECURITY */

static void IRAM_ATTR button_isr_handler(void *arg)
{
#if ENABLE_BLE_SECURITY
    if (!gpio_get_level(BUTTON_0) && !gpio_get_level(BUTTON_1)) {
        xSemaphoreTakeFromISR(ble_reset_mutex, 0);
        xTaskResumeFromISR(ble_reset_task);
    } else {
        xSemaphoreGiveFromISR(ble_reset_mutex, NULL);
#endif /* ENABLE_BLE_SECURITY */
        TickType_t now = xTaskGetTickCount();
        button_handler_data_t *handler_data = (button_handler_data_t *) arg;
        if (!gpio_get_level(handler_data->button)) {
            if (now >= handler_data->click_time + BUTTON_MIN_TRESHOLD / portTICK_PERIOD_MS) {
                lcd_evt_t evt = handler_data->button == BUTTON_0 ? LCD_EVT_CHANGE_SENSOR : LCD_EVT_CHANGE_SCREEN;
                xQueueSendFromISR(lcd_print_sensor_queue, &evt, NULL);
                handler_data->click_time = now;
            }
        }
#if ENABLE_BLE_SECURITY
    }
#endif /* ENABLE_BLE_SECURITY */
}

static void IRAM_ATTR ccs_int_isr_halder(void *arg)
{
    xTaskResumeFromISR(ccs_read_task);
}

static void espnow_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len)
{
    lcd_evt_t evt = LCD_EVT_EXT_VALUE;
    espnow_sensor_data_t *sensor_data = (espnow_sensor_data_t *) data;

    ext_data.write_time = xTaskGetTickCount();
    ext_data.humidity = sensor_data->humidity / 100.0f;
    ext_data.pressure = sensor_data->pressure / 10000.0f;
    ext_data.temperature = sensor_data->temperature / 100.0f;
    ble_set_ext(ext_data.humidity, ext_data.temperature, ext_data.pressure, sensor_data->battery);
    ext_data_push_history(&ext_data);

    xQueueSend(lcd_print_sensor_queue, &evt, portMAX_DELAY);
}

static void espnow_init()
{
    tcpip_adapter_init();

    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT()
    ;
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_CHANNEL, 0));

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));
    ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t *)ESPNOW_PMK));

    sensor_peer.channel = ESPNOW_CHANNEL;
    sensor_peer.ifidx = ESP_IF_WIFI_STA;
    sensor_peer.encrypt = true;
    memcpy(sensor_peer.lmk, ESPNOW_LMK, ESP_NOW_KEY_LEN);
    const char *mac = ESPNOW_SENSOR_MAC;
    for (uint8_t i = 0; i < ESP_NOW_ETH_ALEN; i++) {
        sensor_peer.peer_addr[i] = (uint8_t) strtol(mac + (3 * i), (char **) NULL, 16);
    }
    ESP_LOGI(LOG_TAG, "Sensor MAC %02x:%02x:%02x:%02x:%02x:%02x", sensor_peer.peer_addr[0], sensor_peer.peer_addr[1],
            sensor_peer.peer_addr[2], sensor_peer.peer_addr[3], sensor_peer.peer_addr[4], sensor_peer.peer_addr[5]);

    ESP_ERROR_CHECK(esp_now_add_peer(&sensor_peer));
}

static void gpio_interupt_init()
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.pin_bit_mask = (1ULL << BUTTON_0) | (1ULL << BUTTON_1);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.pin_bit_mask = 1ULL << CCS811_INT;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(BUTTON_0, button_isr_handler, &button_handler_data[0]));
    ESP_ERROR_CHECK(gpio_isr_handler_add(BUTTON_1, button_isr_handler, &button_handler_data[1]));
    ESP_ERROR_CHECK(gpio_isr_handler_add(CCS811_INT, ccs_int_isr_halder, NULL));

    if (!gpio_get_level(CCS811_INT)) {
        ccs_read();
    }
}

void app_main()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    for (uint8_t i = 0; i < HISTORY_SIZE; i++) {
        int_data.history.humidity[i] = NAN;
        int_data.history.temperature[i] = NAN;
        int_data.history.co2[i] = NAN;
        int_data.history.tvoc[i] = NAN;
        ext_data.history.humidity[i] = NAN;
        ext_data.history.pressure[i] = NAN;
        ext_data.history.temperature[i] = NAN;
    }
    ext_data.humidity = NAN;
    ext_data.pressure = NAN;
    ext_data.temperature = NAN;

    lcd_print_sensor_queue = xQueueCreate(10, sizeof(lcd_evt_t));
    ble_connection_queue = xQueueCreate(10, sizeof(bool));
#if ENABLE_BLE_SECURITY
    ble_reset_mutex = xSemaphoreCreateMutex();
#endif /* ENABLE_BLE_SECURITY */

    i2c_init();
    bme_init();
    ccs_init();
    espnow_init();
    ble_init(ble_connection_queue);
    display_init();
    gpio_interupt_init();

    xTaskCreate(bme_read_task_func, "bme_read_task", 4 * 1024, NULL, 5, NULL);
    xTaskCreate(ccs_read_task_func, "ccs_read_task", 4 * 1024, NULL, 5, &ccs_read_task);
    xTaskCreate(lcd_print_sensor_task_func, "lcd_print_sensor_task", 4 * 1024, NULL, 5, &lcd_print_sensor_task);
#if ENABLE_BLE_SECURITY
    xTaskCreate(ble_reset_task_func, "ble_reset_task", 4 * 1024, NULL, 5, &ble_reset_task);
#endif /* ENABLE_BLE_SECURITY */
}
