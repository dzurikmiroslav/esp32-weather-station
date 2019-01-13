#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <esp_event_loop.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <driver/gpio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "sdkconfig.h"
#include "sensor.h"
#include "display.h"
#include "bme.h"

#define LOG_TAG "MAIN"

#define BUTTON_0 CONFIG_BUTTON_0
#define BUTTON_1 CONFIG_BUTTON_1
#define WIFI_CHANNEL CONFIG_WIFI_CHANNEL

#define SCREN_TIMEOUT 10000
#define INTERNAK_SENSOR_READ_PERIOD 5000
#define BUTTON_MIN_TRESHOLD 250

typedef enum
{
   LCD_EVT_VALUE,
   LCD_EVT_CHANGE_SENSOR,
   LCD_EVT_CHANGE_SCREEN
} lcd_evt_type_t;

typedef struct
{
   float pressure;
   float temperature;
   float humidity;
} lcd_evt_data_t;

typedef struct
{
   lcd_evt_type_t type;
   sensor_id_t id;
   lcd_evt_data_t data;
} lcd_evt_t;

static uint8_t station_mac[ESP_NOW_ETH_ALEN] = CONFIG_STATION_MAC;
static uint8_t sensor_mac[ESP_NOW_ETH_ALEN] = CONFIG_SENSOR_MAC;

static QueueHandle_t lcd_queue;

static float act_humidity[SENSOR_COUNT];
static float act_pressure[SENSOR_COUNT];
static float act_temperature[SENSOR_COUNT];

static float hst_humidity[SENSOR_COUNT][HISTORY_SIZE];
static float hst_pressure[SENSOR_COUNT][HISTORY_SIZE];
static float hst_temperature[SENSOR_COUNT][HISTORY_SIZE];

static TickType_t hst_write_time[SENSOR_COUNT];
static float avg_humidity[SENSOR_COUNT];
static float avg_pressure[SENSOR_COUNT];
static float avg_temperature[SENSOR_COUNT];
static uint16_t avg_counter[SENSOR_COUNT];

static TickType_t button_click_time = 0;

static void add_history(sensor_id_t sensor_id, float humidity, float pressure, float temperature)
{
   avg_humidity[sensor_id] += humidity;
   avg_pressure[sensor_id] += pressure;
   avg_temperature[sensor_id] += temperature;
   avg_counter[sensor_id]++;

   TickType_t now = xTaskGetTickCount();
   for (uint8_t s = 0; s < SENSOR_COUNT; s++)
   {
      if (now >= (hst_write_time[s] + HISTORY_PERIOD / portTICK_PERIOD_MS))
      {
         /* rotate history */
         for (int i = 0; i < HISTORY_SIZE - 1; i++)
         {
            hst_humidity[s][i] = hst_humidity[s][i + 1];
            hst_pressure[s][i] = hst_pressure[s][i + 1];
            hst_temperature[s][i] = hst_temperature[s][i + 1];
         }

         hst_humidity[s][HISTORY_SIZE - 1] = avg_humidity[s] / avg_counter[s];
         hst_pressure[s][HISTORY_SIZE - 1] = avg_pressure[s] / avg_counter[s];
         hst_temperature[s][HISTORY_SIZE - 1] = avg_temperature[s] / avg_counter[s];

         hst_write_time[s] = now;
         avg_humidity[s] = 0.0f;
         avg_pressure[s] = 0.0f;
         avg_temperature[s] = 0.0f;
         avg_counter[s] = 0;
      }
   }
}

static void lcd_task(void *param)
{
   lcd_screen_type_t screen_type = LCD_SCREEN_TYPE_ACTUAL;
   sensor_id_t sensor_id = SENSOR_ID_INTERNAL;
   TickType_t sensor_change_time = 0;

   lcd_evt_t evt;
   while (true)
   {
      if (xQueueReceive(lcd_queue, &evt, portMAX_DELAY))
      {
         TickType_t now = xTaskGetTickCount();
         bool change_sensor = now >= (sensor_change_time + SCREN_TIMEOUT / portTICK_PERIOD_MS);
         bool change_screen = false;
         bool new_data = false;

         switch (evt.type)
         {
         case LCD_EVT_VALUE:
            act_humidity[evt.id] = evt.data.humidity;
            act_pressure[evt.id] = evt.data.pressure;
            act_temperature[evt.id] = evt.data.temperature;
            add_history(evt.id, evt.data.humidity, evt.data.pressure, evt.data.temperature);
            new_data = sensor_id == evt.id;
            break;
         case LCD_EVT_CHANGE_SENSOR:
            change_sensor = true;
            break;
         case LCD_EVT_CHANGE_SCREEN:
            change_screen = true;
            change_sensor = false;
            break;
         }

         if (change_sensor)
         {
            sensor_id++;
            if (sensor_id == SENSOR_COUNT)
            {
               sensor_id = 0;
            }
            screen_type = LCD_SCREEN_TYPE_ACTUAL;
            sensor_change_time = now;
         }
         if (change_screen)
         {
            screen_type++;
            if (screen_type == LCD_SCREEN_TYPE_COUNT)
            {
               screen_type = 0;
            }
            sensor_change_time = now;
         }

         if (change_sensor || change_screen || new_data)
         {
            display_clear();
            switch (screen_type)
            {
            case LCD_SCREEN_TYPE_ACTUAL:
               display_print_value(sensor_id, act_pressure[sensor_id], act_temperature[sensor_id], act_humidity[sensor_id]);
               break;
            case LCD_SCREEN_TYPE_HUMIDITY_MIN_MAX:
               display_print_min_max(sensor_id, DISPLAY_VALUE_HUMIDITY, hst_humidity[sensor_id]);
               break;
            case LCD_SCREEN_TYPE_PRESSURE_MIN_MAX:
               display_print_min_max(sensor_id, DISPLAY_VALUE_PRESSURE, hst_pressure[sensor_id]);
               break;
            case LCD_SCREEN_TYPE_TEMPERATURE_MIN_MAX:
               display_print_min_max(sensor_id, DISPLAY_VALUE_TEMPERATURE, hst_temperature[sensor_id]);
               break;
            case LCD_SCREEN_TYPE_HUMIDITY_GRAPH:
               display_print_graph(sensor_id, DISPLAY_VALUE_HUMIDITY, hst_humidity[sensor_id]);
               break;
            case LCD_SCREEN_TYPE_PRESSURE_GRAPH:
               display_print_graph(sensor_id, DISPLAY_VALUE_PRESSURE, hst_pressure[sensor_id]);
               break;
            case LCD_SCREEN_TYPE_TEMPERATURE_GRAPH:
               display_print_graph(sensor_id, DISPLAY_VALUE_TEMPERATURE, hst_temperature[sensor_id]);
               break;
            }
            display_sync();
         }
      }
   }
}

static void read_internal_sensor_task(void *param)
{
   bme_data_t data;
   lcd_evt_t lcd_evt;

   while (true)
   {
      data = bme_read();

      lcd_evt.type = LCD_EVT_VALUE;
      lcd_evt.id = SENSOR_ID_INTERNAL;
      lcd_evt.data.humidity = data.humidity;
      lcd_evt.data.pressure = data.pressure;
      lcd_evt.data.temperature = data.temperature;
      xQueueSend(lcd_queue, &lcd_evt, portMAX_DELAY);

      vTaskDelay(INTERNAK_SENSOR_READ_PERIOD / portTICK_PERIOD_MS);
   }
}

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
   TickType_t now = xTaskGetTickCount();
   if (now >= button_click_time + BUTTON_MIN_TRESHOLD / portTICK_PERIOD_MS)
   {
      lcd_evt_t lcd_evt;
      lcd_evt.type = (lcd_evt_type_t)arg;
      xQueueSendFromISR(lcd_queue, &lcd_evt, NULL);
      button_click_time = now;
   }
}

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
   return ESP_OK;
}

static void wifi_init()
{
   esp_err_t ret = nvs_flash_init();
   if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
   {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
   }
   ESP_ERROR_CHECK(ret);

   tcpip_adapter_init();

   ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

   wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
   ESP_ERROR_CHECK(esp_wifi_init(&cfg));
   ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
   ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
   ESP_ERROR_CHECK(esp_wifi_set_mac(ESP_IF_WIFI_STA, station_mac));
   ESP_ERROR_CHECK(esp_wifi_start());

   ESP_ERROR_CHECK(esp_wifi_set_channel(WIFI_CHANNEL, 0));

#if CONFIG_ENABLE_LONG_RANGE
   ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_MODE_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR));
#endif
}

static void espnow_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len)
{
   espnow_sensor_data_t *sensor_data = (espnow_sensor_data_t *)data;

   lcd_evt_t lcd_evt;
   lcd_evt.type = LCD_EVT_VALUE;
   lcd_evt.id = SENSOR_ID_EXTERNAL;
   lcd_evt.data.humidity = sensor_data->humidity;
   lcd_evt.data.pressure = sensor_data->pressure;
   lcd_evt.data.temperature = sensor_data->temperature;
   xQueueSend(lcd_queue, &lcd_evt, portMAX_DELAY);
}

static void espnow_init()
{
   ESP_ERROR_CHECK(esp_now_init());
   ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

   esp_now_peer_info_t peer;
   peer.channel = WIFI_CHANNEL;
   peer.ifidx = ESP_IF_WIFI_STA;
   peer.encrypt = false;
   memcpy(peer.peer_addr, sensor_mac, ESP_NOW_ETH_ALEN);
   ESP_ERROR_CHECK(esp_now_add_peer(&peer));
}

static void buttons_init()
{
   gpio_config_t io_conf;
   io_conf.intr_type = GPIO_INTR_NEGEDGE;
   io_conf.pin_bit_mask = ((1ULL << BUTTON_0) | (1ULL << BUTTON_1));
   io_conf.mode = GPIO_MODE_INPUT;
   io_conf.pull_down_en = 0;
   io_conf.pull_up_en = 1;
   ESP_ERROR_CHECK(gpio_config(&io_conf));
   ESP_ERROR_CHECK(gpio_install_isr_service(0));
   ESP_ERROR_CHECK(gpio_isr_handler_add(BUTTON_0, gpio_isr_handler, (void *)LCD_EVT_CHANGE_SENSOR));
   ESP_ERROR_CHECK(gpio_isr_handler_add(BUTTON_1, gpio_isr_handler, (void *)LCD_EVT_CHANGE_SCREEN));
}

void app_main()
{
   for (uint8_t s = 0; s < SENSOR_COUNT; s++)
   {
      hst_write_time[s] = 0;
      act_humidity[s] = NAN;
      act_pressure[s] = NAN;
      act_temperature[s] = NAN;
      avg_humidity[s] = 0.0f;
      avg_pressure[s] = 0.0f;
      avg_temperature[s] = 0.0f;
      for (uint8_t i = 0; i < HISTORY_SIZE; i++)
      {
         hst_humidity[s][i] = NAN;
         hst_pressure[s][i] = NAN;
         hst_temperature[s][i] = NAN;
      }
   }
   lcd_queue = xQueueCreate(10, sizeof(lcd_evt_t));

   bme_init();
   wifi_init();
   espnow_init();
   display_init();
   buttons_init();

   xTaskCreate(read_internal_sensor_task, "read_internal_sensor_task", 4 * 1024, NULL, 5, NULL);
   xTaskCreate(lcd_task, "lcd_task", 4 * 1024, NULL, 5, NULL);
}
