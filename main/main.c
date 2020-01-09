#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
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
#include "bsec_integration.h"
#include "ble.h"

static const char* TAG = "main";

#define ESPNOW_CHANNEL CONFIG_ESPNOW_CHANNEL
#define ESPNOW_SENSOR_MAC CONFIG_ESPNOW_SENSOR_MAC
#define ESPNOW_PMK CONFIG_ESPNOW_PMK
#define ESPNOW_LMK CONFIG_ESPNOW_LMK

#define BTN_NEXT            15
#define BTN_PREV            4
#define BTN_SEL             18
#define LCD_BL              33

#define SCREN_TIMEOUT       60000
#define BUTTON_MIN_TRESHOLD 250
#define EXT_SENSOR_TIMEUT   6000000 /* 10min */

typedef struct
{
    uint32_t pressure;
    int16_t temperature;
    uint16_t humidity;
    uint8_t battery;
}__attribute__((packed)) espnow_sensor_data_t;

typedef struct
{
    float humidity;
    float temperature;
    float pressure;
    float iaq;
    struct
    {
        float humidity[HISTORY_SIZE];
        float temperature[HISTORY_SIZE];
        float pressure[HISTORY_SIZE];
        float iaq[HISTORY_SIZE];
        TickType_t write_time;
        float avg_humidity;
        float avg_temperature;
        float avg_pressure;
        float avg_iaq;
        uint16_t avg_counter;
    } history;
} int_data_t;

typedef struct
{
    float humidity;
    float temperature;
    TickType_t write_time;
    struct
    {
        float humidity[HISTORY_SIZE];
        float temperature[HISTORY_SIZE];
        TickType_t write_time;
        float avg_humidity;
        float avg_temperature;
        uint16_t avg_counter;
    } history;
} ext_data_t;

static int_data_t int_data;
static ext_data_t ext_data;

typedef enum
{
    SCREEN_MAIN, SCREEN_DETAIL, SCREEN_SETTINGS, SCREEN_TIME, SCREEN_BLE_PAIRING, SCREEN_BLE_PAIRED_RESULT
} screen_t;

screen_t screen = SCREEN_MAIN;
int screen_selection = -1;

static TickType_t screen_time = 0;

static uint8_t hours_offset = 0;
static uint8_t minutes_offset = 0;
static uint8_t seconds_offset = 0;

static esp_now_peer_info_t sensor_peer;

static uint32_t ble_passkey;

typedef enum
{
    BUTTON_PREVIOUS, BUTTON_NEXT, BUTTON_SELECT, BUTTON_NB
} button_t;

static TickType_t button_push_time[BUTTON_NB];
static TickType_t button_release_time[BUTTON_NB];
static gpio_num_t button_gpio[BUTTON_NB] = { BTN_PREV, BTN_NEXT, BTN_SEL };

static TaskHandle_t screen_refresh_task;

static QueueHandle_t button_queue;
static QueueHandle_t ble_connection_queue;

static void ext_data_push_history(ext_data_t *data)
{
    data->history.avg_humidity += data->humidity;
    data->history.avg_temperature += data->temperature;
    data->history.avg_counter++;

    TickType_t now = xTaskGetTickCount();

    if (now >= (data->history.write_time + HISTORY_PERIOD / portTICK_PERIOD_MS)) {
        /* rotate history */
        for (int i = 0; i < HISTORY_SIZE - 1; i++) {
            data->history.humidity[i] = data->history.humidity[i + 1];
            data->history.temperature[i] = data->history.temperature[i + 1];
        }

        data->history.humidity[HISTORY_SIZE - 1] = data->history.avg_humidity / data->history.avg_counter;
        data->history.temperature[HISTORY_SIZE - 1] = data->history.avg_temperature / data->history.avg_counter;

        data->history.write_time = now;
        data->history.avg_humidity = 0.0f;
        data->history.avg_temperature = 0.0f;
        data->history.avg_counter = 0;
    }
}

static void int_data_push_history(int_data_t *data)
{
    data->history.avg_humidity += data->humidity;
    data->history.avg_temperature += data->temperature;
    data->history.avg_pressure += data->pressure;
    data->history.avg_iaq += data->iaq;
    data->history.avg_counter++;

    TickType_t now = xTaskGetTickCount();

    if (now >= (data->history.write_time + HISTORY_PERIOD / portTICK_PERIOD_MS)) {
        /* rotate history */
        for (int i = 0; i < HISTORY_SIZE - 1; i++) {
            data->history.humidity[i] = data->history.humidity[i + 1];
            data->history.temperature[i] = data->history.temperature[i + 1];
            data->history.pressure[i] = data->history.pressure[i + 1];
            data->history.iaq[i] = data->history.iaq[i + 1];
        }

        data->history.humidity[HISTORY_SIZE - 1] = data->history.avg_humidity / data->history.avg_counter;
        data->history.temperature[HISTORY_SIZE - 1] = data->history.avg_temperature / data->history.avg_counter;
        data->history.pressure[HISTORY_SIZE - 1] = data->history.avg_pressure / data->history.avg_counter;
        data->history.iaq[HISTORY_SIZE - 1] = data->history.avg_iaq / data->history.avg_counter;

        data->history.write_time = now;
        data->history.avg_humidity = 0.0f;
        data->history.avg_temperature = 0.0f;
        data->history.avg_pressure = 0.0f;
        data->history.avg_iaq = 0.0f;
        data->history.avg_counter = 0;
    }
}

static void set_time()
{
    struct timeval timeval;
    gettimeofday(&timeval, NULL);
    struct tm tm = *localtime(&timeval.tv_sec);
    tm.tm_hour = (tm.tm_hour + hours_offset) % 24;
    tm.tm_min = (tm.tm_min + minutes_offset) % 60;
    tm.tm_sec = (tm.tm_sec + seconds_offset) % 60;
    timeval.tv_sec = mktime(&tm);
    settimeofday(&timeval, NULL);
    hours_offset = minutes_offset = seconds_offset = 0;
}

static int screen_get_selection_num(screen_t screen)
{
    switch (screen) {
        case SCREEN_MAIN:
            return DISPLAY_VALUE_NB;
        case SCREEN_SETTINGS:
            return DISPLAY_SETTING_NB;
        case SCREEN_TIME:
            return DISPLAY_TIME_NB;
        default:
            return -1;
    }
}

static TickType_t screen_get_time(screen_t screen)
{
    switch (screen) {
//        case SCREEN_MAIN:
//            return portMAX_DELAY ;
        case SCREEN_BLE_PAIRED_RESULT:
            return 5000 / portTICK_PERIOD_MS;
        default:
            return 60000 / portTICK_PERIOD_MS;
    }
}

static void screen_prolongate_tiomeout()
{
    screen_time = xTaskGetTickCount() + screen_get_time(screen);
}

static void screen_navigate(screen_t new_screen, int new_selection)
{
    ESP_LOGI(TAG, "screen_navigate %d", new_screen);
    screen = new_screen;
    screen_selection = new_selection;
    //screen_time = xTaskGetTickCount() + screen_get_time(screen);
    screen_prolongate_tiomeout();
}

static void screen_handle_button(button_t button)
{
    if ((button == BUTTON_PREVIOUS || button == BUTTON_NEXT)
            && button_push_time[BUTTON_PREVIOUS] > button_release_time[BUTTON_PREVIOUS]
            && button_push_time[BUTTON_NEXT] > button_release_time[BUTTON_NEXT]) {
        if (screen == SCREEN_MAIN) {
            screen_navigate(SCREEN_SETTINGS, -1);
        }
    } else if (button == BUTTON_PREVIOUS) {
        screen_navigate(screen, screen_selection <= 0 ? screen_get_selection_num(screen) - 1 : screen_selection - 1);
    } else if (button == BUTTON_NEXT) {
        screen_navigate(screen, (screen_selection + 1) % screen_get_selection_num(screen));
    } else { //BUTTON_SELECT
        switch (screen) {
            case SCREEN_MAIN:
                if (screen_selection != -1) {
                    screen_navigate(SCREEN_DETAIL, screen_selection);
                }
                break;
            case SCREEN_DETAIL:
                screen_navigate(SCREEN_MAIN, screen_selection);
                break;
            case SCREEN_SETTINGS:
                switch (screen_selection) {
                    case DISPLAY_SETTING_TIME:
                        screen_navigate(SCREEN_TIME, -1);
                        break;
                    case DISPLAY_SETTING_BT_START_PAIRING:
                        ble_passkey = (esp_random() / (float) UINT32_MAX) * 999999;
                        ble_enable_pairing(ble_passkey);
                        screen_navigate(SCREEN_BLE_PAIRING, -1);
                        break;
                    case DISPLAY_SETTING_BT_REMOVE_PAIRED_DEVICES:
                        ble_remove_paired_device();
                        screen_navigate(SCREEN_MAIN, -1);
                        break;
                    case DISPLAY_SETTING_BACK:
                        screen_navigate(SCREEN_MAIN, -1);
                        break;
                    default:
                        break;
                }
                break;
            case SCREEN_TIME:
                switch (screen_selection) {
                    case DISPLAY_TIME_HOURS:
                        hours_offset++;
                        screen_prolongate_tiomeout();
                        break;
                    case DISPLAY_TIME_MINUTES:
                        minutes_offset++;
                        screen_prolongate_tiomeout();
                        break;
                    case DISPLAY_TIME_SECONDS:
                        seconds_offset++;
                        screen_prolongate_tiomeout();
                        break;
                    case DISPLAY_TIME_SET:
                        set_time();
                        screen_navigate(SCREEN_SETTINGS, -1);
                        break;
                    case DISPLAY_TIME_BACK:
                        screen_navigate(SCREEN_SETTINGS, -1);
                        break;
                    default:
                        break;
                }
                break;
            default:
                break;
        }
    }
}

static void screen_refresh_task_func(void *param)
{
    QueueSetHandle_t queue_set = xQueueCreateSet(20);
    xQueueAddToSet(button_queue, queue_set);
    xQueueAddToSet(ble_connection_queue, queue_set);

    static bool ble_connection = false;

    while (true) {
        QueueSetMemberHandle_t queue_member = xQueueSelectFromSet(queue_set, 1000 / portTICK_PERIOD_MS);
        TickType_t now = xTaskGetTickCount();
        bool refresh = false;

        if (now >= ext_data.write_time + EXT_SENSOR_TIMEUT / portTICK_PERIOD_MS) {
            ext_data.temperature = NAN;
            ext_data.humidity = NAN;
            ext_data.write_time = now;
        }

        if (now >= screen_time) {
            ESP_LOGI(TAG, "Screen %d timeout...", screen);
            if (screen == SCREEN_BLE_PAIRING) {
                ble_disable_pairing();
                screen_navigate(SCREEN_BLE_PAIRED_RESULT, -1);
                refresh = true;
            } else {
                refresh |= screen != SCREEN_MAIN || screen_selection != -1;
                screen_navigate(SCREEN_MAIN, -1);
            }
        }

        if (queue_member == button_queue) {
            button_t button;
            xQueueReceive(button_queue, &button, 0);
            if (screen != SCREEN_BLE_PAIRING && screen != SCREEN_BLE_PAIRED_RESULT) {
                screen_handle_button(button);
                refresh = true;
            }
        } else if (queue_member == ble_connection_queue) {
            xQueueReceive(ble_connection_queue, &ble_connection, 0);

            if (screen == SCREEN_BLE_PAIRING) {
                screen_navigate(SCREEN_BLE_PAIRED_RESULT, -1);
                refresh = true;
            } else {
                refresh |= screen == SCREEN_MAIN;
            }
        } else {
            //1sec wait timeout
            refresh |= screen == SCREEN_MAIN || screen == SCREEN_TIME; // refresh displaying time

            if (screen == SCREEN_TIME) {
                //hold button
                if (now >= button_push_time[BUTTON_SELECT] + 500 / portTICK_PERIOD_MS
                        && button_push_time[BUTTON_SELECT] > button_release_time[BUTTON_SELECT]) {
                    if (screen_selection == DISPLAY_TIME_HOURS) {
                        hours_offset += 10;
                        screen_prolongate_tiomeout();
                    } else if (screen_selection == DISPLAY_TIME_MINUTES) {
                        minutes_offset += 10;
                        screen_prolongate_tiomeout();
                    } else if (screen_selection == DISPLAY_TIME_SECONDS) {
                        seconds_offset += 10;
                        screen_prolongate_tiomeout();
                    }
                }
            }
        }

        if (refresh) {
            display_clear();

            switch (screen) {
                case SCREEN_MAIN:
                    display_print_main(int_data.temperature, int_data.humidity, int_data.iaq, int_data.pressure,
                            ext_data.temperature, ext_data.humidity, ble_connection, screen_selection);
                    break;
                case SCREEN_DETAIL:
                    switch (screen_selection) {
                        case DISPLAY_VALUE_TEMPERATURE:
                            display_print_graph(screen_selection, int_data.history.temperature);
                            break;
                        case DISPLAY_VALUE_HUMIDITY:
                            display_print_graph(screen_selection, int_data.history.humidity);
                            break;
                        case DISPLAY_VALUE_PRESSURE:
                            display_print_graph(screen_selection, int_data.history.pressure);
                            break;
                        case DISPLAY_VALUE_IAQ:
                            display_print_graph(screen_selection, int_data.history.iaq);
                            break;
                        case DISPLAY_VALUE_OUT_TEMPERATURE:
                            display_print_graph(screen_selection, ext_data.history.temperature);
                            break;
                        case DISPLAY_VALUE_OUT_HUMIDITY:
                            display_print_graph(screen_selection, ext_data.history.humidity);
                            break;
                        default:
                            break;
                    }
                    break;
                case SCREEN_SETTINGS:
                    display_settins(screen_selection);
                    break;
                case SCREEN_TIME:
                    display_time(hours_offset, minutes_offset, seconds_offset, screen_selection);
                    break;
                case SCREEN_BLE_PAIRING:
                    display_print_pairing(ble_passkey, false);
                    break;
                case SCREEN_BLE_PAIRED_RESULT:
                    display_print_paired_result(ble_connection);
                    break;
            }

            display_sync();
        }
    }
}

static void IRAM_ATTR button_isr_handler(void *arg)
{
    button_t button = (button_t) arg;

    TickType_t now = xTaskGetTickCount();

    if (!gpio_get_level(button_gpio[button])) {
        if (now >= button_push_time[button] + BUTTON_MIN_TRESHOLD / portTICK_PERIOD_MS) {
            button_push_time[button] = now;

            xQueueSendFromISR(button_queue, &button, NULL);
        }
    } else {
        button_release_time[button] = now;
    }
}

static void espnow_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len)
{
    espnow_sensor_data_t *sensor_data = (espnow_sensor_data_t *) data;

    ext_data.write_time = xTaskGetTickCount();
    ext_data.humidity = sensor_data->humidity / 100.0f;
    ext_data.temperature = sensor_data->temperature / 100.0f;
    ble_set_out_telemetry(ext_data.humidity, ext_data.temperature, sensor_data->battery);
    ext_data_push_history(&ext_data);
}

static void espnow_init()
{
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_init(NULL, NULL));

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
    ESP_LOGI(TAG, "Sensor MAC %02x:%02x:%02x:%02x:%02x:%02x", sensor_peer.peer_addr[0], sensor_peer.peer_addr[1],
            sensor_peer.peer_addr[2], sensor_peer.peer_addr[3], sensor_peer.peer_addr[4], sensor_peer.peer_addr[5]);

    ESP_ERROR_CHECK(esp_now_add_peer(&sensor_peer));
}

static void gpio_interupt_init()
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.pin_bit_mask = (1ULL << BTN_PREV) | (1ULL << BTN_NEXT) | (1ULL << BTN_SEL);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_IRAM));

    ESP_ERROR_CHECK(gpio_isr_handler_add(button_gpio[BUTTON_PREVIOUS], button_isr_handler, (void* )BUTTON_PREVIOUS));
    ESP_ERROR_CHECK(gpio_isr_handler_add(button_gpio[BUTTON_NEXT], button_isr_handler, (void* )BUTTON_NEXT));
    ESP_ERROR_CHECK(gpio_isr_handler_add(button_gpio[BUTTON_SELECT], button_isr_handler, (void* )BUTTON_SELECT));
}

static void sensor_delay(uint32_t period)
{
    vTaskDelay(period / portTICK_PERIOD_MS);
}

int64_t sensor_get_timestamp_us()
{
    return esp_timer_get_time();
}

static uint32_t sensor_state_load(uint8_t *state_buffer, uint32_t n_buffer)
{
    return 0;
}

static void sensor_state_save(const uint8_t *state_buffer, uint32_t length)
{
}

static uint32_t sensor_config_load(uint8_t *config_buffer, uint32_t n_buffer)
{
    return 0;
}

static void sensor_output_ready(int64_t timestamp, float iaq, uint8_t iaq_accuracy, float temperature, float humidity,
        float pressure, float raw_temperature, float raw_humidity, float gas, bsec_library_return_t bsec_status,
        float static_iaq, float co2_equivalent, float breath_voc_equivalent)
{
    ESP_LOGD(TAG, "Temperature %f °C", temperature);
    ESP_LOGD(TAG, "Humidity %f %%", humidity);
    ESP_LOGD(TAG, "Pressure %f Pa", pressure);
    ESP_LOGD(TAG, "IAQ %f", iaq);
    ESP_LOGD(TAG, "CO2 equivalent %f ppm VOC equivalent %f ppm", co2_equivalent, breath_voc_equivalent);

    int_data.humidity = humidity;
    int_data.temperature = temperature;
    int_data.pressure = pressure / 100; //hPa
    int_data.iaq = iaq;

    ble_set_telemetry(humidity, temperature, pressure, iaq, co2_equivalent, breath_voc_equivalent);
    int_data_push_history(&int_data);
}

static void sensor_init()
{
    return_values_init ret = bsec_iot_init(BSEC_SAMPLE_RATE_LP, 0.0f, i2c_write, i2c_read, sensor_delay,
            sensor_state_load, sensor_config_load);
    ESP_ERROR_CHECK(ret.bme680_status);
    ESP_ERROR_CHECK(ret.bsec_status);
}

static void senosor_read_task_func(void *param)
{
    bsec_iot_loop(sensor_delay, sensor_get_timestamp_us, sensor_output_ready, sensor_state_save, UINT32_MAX);
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
        int_data.history.pressure[i] = NAN;
        int_data.history.iaq[i] = NAN;
        ext_data.history.humidity[i] = NAN;
        ext_data.history.temperature[i] = NAN;
    }

    ext_data.humidity = NAN;
    ext_data.temperature = NAN;

    button_queue = xQueueCreate(5, sizeof(button_t));
    ble_connection_queue = xQueueCreate(5, sizeof(bool));

    /*TODO blacklight */
    gpio_config_t io_conf;
    io_conf.pin_bit_mask = 1ULL << LCD_BL;
    io_conf.mode = GPIO_MODE_OUTPUT;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    gpio_set_level(LCD_BL, 1);

    i2c_init();
    sensor_init();
    espnow_init();
    ble_init(ble_connection_queue);
    display_init();
    gpio_interupt_init();

    xTaskCreate(screen_refresh_task_func, "screen_refresh_task", 4 * 1024, NULL, 5, &screen_refresh_task);
    xTaskCreate(senosor_read_task_func, "senosor_read_task", 4 * 1024, NULL, 5, NULL);
}
