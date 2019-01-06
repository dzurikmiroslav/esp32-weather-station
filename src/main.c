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

#define BUTTON_0 GPIO_NUM_15
#define BUTTON_1 GPIO_NUM_13
#define WIFI_CHANNEL CONFIG_WIFI_CHANNEL

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

static void compute_history()
{
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
   lcd_screen_type_t screen_type = LCD_SCREEN_TYPE_TEMPERATURE_GRAPH;
   sensor_id_t sensor_id = SENSOR_ID_INTERNAL;

   lcd_evt_t evt;
   while (true)
   {
      if (xQueueReceive(lcd_queue, &evt, portMAX_DELAY))
      {
         switch (evt.type)
         {
         case LCD_EVT_VALUE:
            act_humidity[evt.id] = evt.data.humidity;
            act_pressure[evt.id] = evt.data.pressure;
            act_temperature[evt.id] = evt.data.temperature;
            avg_humidity[evt.id] += evt.data.humidity;
            avg_pressure[evt.id] += evt.data.pressure;
            avg_temperature[evt.id] += evt.data.temperature;
            avg_counter[evt.id]++;
            compute_history();
            break;
         case LCD_EVT_ROTATE:
         case LCD_EVT_BTN_0:
            screen_type = LCD_SCREEN_TYPE_ACTUAL;
            sensor_id++;
            if (sensor_id == SENSOR_COUNT)
            {
               sensor_id = 0;
            }
            break;
         case LCD_EVT_BTN_1:
            screen_type++;
            if (screen_type == LCD_SCREEN_TYPE_COUNT)
               screen_type = 0;
            break;
         }

         pcd8544_clear_display();
         pcd8544_finalize_frame_buf();

         switch (screen_type)
         {
         case LCD_SCREEN_TYPE_ACTUAL:
            display_print_value(sensor_id, act_pressure[sensor_id], act_temperature[sensor_id], act_humidity[sensor_id]);
            break;
         case LCD_SCREEN_TYPE_HUMIDITY_MIN_MAX:
            display_print_min_max(sensor_id, "hum", hst_humidity[sensor_id], "%");
            break;
         case LCD_SCREEN_TYPE_PRESSURE_MIN_MAX:
            display_print_min_max(sensor_id, "pres", hst_pressure[sensor_id], "kPa");
            break;
         case LCD_SCREEN_TYPE_TEMPERATURE_MIN_MAX:
            display_print_min_max(sensor_id, "temp", hst_temperature[sensor_id], "'C");
            break;
         case LCD_SCREEN_TYPE_HUMIDITY_GRAPH:
            display_print_graph(sensor_id, "hum", hst_humidity[sensor_id], 10);
            break;
         case LCD_SCREEN_TYPE_PRESSURE_GRAPH:
            display_print_graph(sensor_id, "pres", hst_pressure[sensor_id], 100);
            break;
         case LCD_SCREEN_TYPE_TEMPERATURE_GRAPH:
            display_print_graph(sensor_id, "temp", hst_temperature[sensor_id], 5);
            break;
         }

         pcd8544_sync_and_gc();
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

      vTaskDelay(1000 / portTICK_PERIOD_MS);
   }
}

static void lcd_rotate_task(void *param)
{
   lcd_evt_t lcd_evt;
   lcd_evt.type = LCD_EVT_ROTATE;
   while (true)
   {
      xQueueSend(lcd_queue, &lcd_evt, portMAX_DELAY);

      vTaskDelay(2000 / portTICK_PERIOD_MS);
   }
}

static void read_buttons_task(void *param)
{
   gpio_set_direction(BUTTON_0, GPIO_MODE_INPUT);
   gpio_set_direction(BUTTON_1, GPIO_MODE_INPUT);
   gpio_set_pull_mode(BUTTON_0, GPIO_PULLUP_ONLY);
   gpio_set_pull_mode(BUTTON_1, GPIO_PULLUP_ONLY);

   lcd_evt_t lcd_evt;
   while (true)
   {
      if (gpio_get_level(BUTTON_0) == 0)
      {
         lcd_evt.type = LCD_EVT_BTN_0;
         xQueueSend(lcd_queue, &lcd_evt, portMAX_DELAY);
      }
      if (gpio_get_level(BUTTON_1) == 0)
      {
         lcd_evt.type = LCD_EVT_BTN_1;
         xQueueSend(lcd_queue, &lcd_evt, portMAX_DELAY);
      }

      vTaskDelay(200 / portTICK_PERIOD_MS);
   }
}

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
   lcd_evt_t lcd_evt;
   lcd_evt.type = (lcd_evt_type_t)arg;
   xQueueSendFromISR(lcd_queue, &lcd_evt, NULL);
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
   lcd_evt.id = SENSOR_ID_EXTERNAL_0;
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

void app_main()
{
   for (uint8_t s = 0; s < SENSOR_COUNT; s++)
   {
      hst_write_time[s] = 0;
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

   bme_init();
   wifi_init();
   espnow_init();
   display_init();

   pcd8544_set_backlight(false);
   pcd8544_finalize_frame_buf();
   pcd8544_clear_display();

   lcd_queue = xQueueCreate(10, sizeof(lcd_evt_t));

   // gpio_config_t io_conf;
   // io_conf.intr_type = GPIO_INTR_POSEDGE;
   // io_conf.pin_bit_mask = ((1ULL << BUTTON_0) | (1ULL << BUTTON_1));
   // io_conf.mode = GPIO_MODE_INPUT;
   // io_conf.pull_down_en = 0;
   // io_conf.pull_up_en = 1;
   // gpio_config(&io_conf);
   // gpio_install_isr_service(0);
   // gpio_isr_handler_add(BUTTON_0, gpio_isr_handler, (void *)LCD_EVT_BTN_0);
   // gpio_isr_handler_add(BUTTON_1, gpio_isr_handler, (void *)LCD_EVT_BTN_1);

   xTaskCreate(read_internal_sensor_task, "read_internal_sensor_task", 4 * 1024, NULL, 5, NULL);
   xTaskCreate(lcd_task, "lcd_task", 4 * 1024, NULL, 5, NULL);
   // xTaskCreate(lcd_rotate_task, "lcd_rotate_task", 4 * 1024, NULL, 5, NULL);
   xTaskCreate(read_buttons_task, "read_buttons_task", 4 * 1024, NULL, 5, NULL);
}
