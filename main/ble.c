#include <string.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

#include "ble.h"

static const char* TAG = "ble";

#define DEVICE_NAME "ESP_METEO"
#define PROFILE_APP_ID 0

#define PERM_READ ESP_GATT_PERM_READ_ENCRYPTED
#define PERM_READ_WRITE ESP_GATT_PERM_READ_ENCRYPTED | ESP_GATT_PERM_WRITE_ENCRYPTED

#define ADV_CONFIG_FLAG (1 << 0)
#define SCAN_RSP_CONFIG_FLAG (1 << 1)

static uint8_t adv_config_done = 0;

// @formatter:off
static uint8_t service_uuid[16] = {
        //TODO other services
        0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x18, 0x1A, 0x00, 0x00
};

static esp_ble_adv_data_t adv_data = {
        .set_scan_rsp = false,
        .include_name = true,
        .include_txpower = true,
        .min_interval = 0x0006,
        .max_interval = 0x0010,
        .appearance = 0x00,
        .manufacturer_len = 0,
        .p_manufacturer_data = NULL,
        .service_data_len = 0,
        .p_service_data = NULL,
        .service_uuid_len = sizeof(service_uuid),
        .p_service_uuid = service_uuid,
        .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_data_t scan_rsp_data = {
        .set_scan_rsp = true,
        .include_name = true,
        .include_txpower = true,
        .min_interval = 0x0006,
        .max_interval = 0x0010,
        .appearance = 0x00,
        .manufacturer_len = 0,
        .p_manufacturer_data = NULL,
        .service_data_len = 0,
        .p_service_data = NULL,
        .service_uuid_len = sizeof(service_uuid),
        .p_service_uuid = service_uuid,
        .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {
        .adv_int_min = 0x0800,
        .adv_int_max = 0x0800,
        .adv_type = ADV_TYPE_IND,
        .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
        .channel_map = ADV_CHNL_ALL,
        .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

enum
{
    ENV_SENS_IDX_SVC,

    ENV_SENS_IDX_CHAR_HUM,
    ENV_SENS_IDX_CHAR_VAL_HUM,
    ENV_SENS_IDX_CHAR_CFG_HUM,

    ENV_SENS_IDX_CHAR_TEMP,
    ENV_SENS_IDX_CHAR_VAL_TEMP,
    ENV_SENS_IDX_CHAR_CFG_TEMP,

    ENV_SENS_IDX_CHAR_PRESS,
    ENV_SENS_IDX_CHAR_VAL_PRESS,
    ENV_SENS_IDX_CHAR_CFG_PRESS,

    ENV_SENS_IDX_CHAR_IAQ,
    ENV_SENS_IDX_CHAR_VAL_IAQ,
    ENV_SENS_IDX_CHAR_CFG_IAQ,

    ENV_SENS_IDX_CHAR_CO2,
    ENV_SENS_IDX_CHAR_VAL_CO2,
    ENV_SENS_IDX_CHAR_CFG_CO2,

    ENV_SENS_IDX_CHAR_VOC,
    ENV_SENS_IDX_CHAR_VAL_VOC,
    ENV_SENS_IDX_CHAR_CFG_VOC,

    ENV_SENS_IDX_CHAR_OUT_HUM,
    ENV_SENS_IDX_CHAR_VAL_OUT_HUM,
    ENV_SENS_IDX_CHAR_CFG_OUT_HUM,

    ENV_SENS_IDX_CHAR_OUT_TEMP,
    ENV_SENS_IDX_CHAR_VAL_OUT_TEMP,
    ENV_SENS_IDX_CHAR_CFG_OUT_TEMP,

    ENV_SENS_IDX_NB
};

enum
{
   BAT_SERV_IDX_SVC,

   BAT_SERV_IDX_CHAR_BAT_LVL,
   BAT_SERV_IDX_CHAR_VAL_BAT_LVL,
   BAT_SERV_IDX_CHAR_CFG_BAT_LVL,

   BAT_SERV_IDX_NB
};

enum
{
   CUR_TIME_IDX_SVC,

   CUR_TIME_IDX_CHAR_DATE_TIME,
   CUR_TIME_IDX_CHAR_VAL_DATE_TIME,
   CUR_TIME_IDX_CHAR_CFG_DATE_TIME,

   CUR_TIME_IDX_NB
};

typedef struct {
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
} __attribute__((packed))  gatt_date_time_t;

static const uint16_t GATTS_SERVICE_UUID_ENV_SENS   = 0x181A;
static const uint16_t GATTS_SERVICE_UUID_BAT_SERV   = 0x180F;
static const uint16_t GATTS_SERVICE_UUID_CUR_TIME   = 0x1805;
static const uint16_t GATTS_CHAR_UUID_HUM           = 0x2A6F;
static const uint16_t GATTS_CHAR_UUID_TEMP          = 0x2A6E;
static const uint16_t GATTS_CHAR_UUID_PRESS         = 0x2A6D;
static const uint16_t GATTS_CHAR_UUID_IAQ           = 0x2F01;
static const uint16_t GATTS_CHAR_UUID_CO2           = 0x2F02;
static const uint16_t GATTS_CHAR_UUID_VOC           = 0x2F03;
static const uint16_t GATTS_CHAR_UUID_OUT_HUM       = 0x3A6F;
static const uint16_t GATTS_CHAR_UUID_OUT_TEMP      = 0x3A6E;
static const uint16_t GATTS_CHAR_UUID_OUT_BAT_LVL   = 0x3A19; // 0x2A19 reserved for GATTS_CHAR_UUID_BAT_LVL
static const uint16_t GATTS_CHAR_UUID_DATE_TIME     = 0x2A08;

static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint8_t char_prop_read = ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_read_write = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE;
static const uint8_t char_prop_read_notify = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;

static const uint8_t hum_ccc[2] = {0x00, 0x00};
static uint16_t hum_val = 20;

static const uint8_t temp_ccc[2] = {0x00, 0x00};
static int16_t temp_val = 0;

static uint32_t press_val = 0;
static uint8_t press_ccc[2] = {0x00, 0x00};

static uint16_t iaq_val = 0;
static uint8_t iaq_ccc[2] = {0x00, 0x00};

static uint16_t co2_val = 0;
static uint8_t co2_ccc[2] = {0x00, 0x00};

static uint16_t voc_val = 0;
static uint8_t voc_ccc[2] = {0x00, 0x00};

static uint16_t out_hum_val = 0;
static uint8_t out_hum_ccc[2] = {0x00, 0x00};

static int16_t out_temp_val = 0;
static uint8_t out_temp_ccc[2] = {0x00, 0x00};

static uint8_t out_bat_lvl_val = 0;
static uint8_t out_bat_lvl_ccc[2] = {0x00, 0x00};

static uint8_t date_time_ccc[2] = {0x00, 0x00};

static const esp_gatts_attr_db_t gatt_env_sens_db[ENV_SENS_IDX_NB] = {
    [ENV_SENS_IDX_SVC] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, PERM_READ, sizeof(uint16_t), sizeof(uint16_t), (uint8_t *)&GATTS_SERVICE_UUID_ENV_SENS}},

    [ENV_SENS_IDX_CHAR_HUM] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, PERM_READ, sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_read_notify}},
    [ENV_SENS_IDX_CHAR_VAL_HUM] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_HUM, PERM_READ, sizeof(uint16_t), sizeof(uint16_t), (uint8_t *)&hum_val}},
    [ENV_SENS_IDX_CHAR_CFG_HUM] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, PERM_READ_WRITE, 2 * sizeof(uint8_t), 2 * sizeof(uint8_t), (uint8_t *)hum_ccc}},

    [ENV_SENS_IDX_CHAR_TEMP] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, PERM_READ, sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_read_notify}},
    [ENV_SENS_IDX_CHAR_VAL_TEMP] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEMP, PERM_READ, sizeof(int16_t), sizeof(int16_t), (uint8_t *)&temp_val}},
    [ENV_SENS_IDX_CHAR_CFG_TEMP] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, PERM_READ_WRITE, 2 * sizeof(uint8_t), 2 * sizeof(uint8_t), (uint8_t *)temp_ccc}},

    [ENV_SENS_IDX_CHAR_PRESS] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, PERM_READ, sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_read_notify}},
    [ENV_SENS_IDX_CHAR_VAL_PRESS] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_PRESS, PERM_READ, sizeof(uint32_t), sizeof(uint32_t), (uint8_t *)&press_val}},
    [ENV_SENS_IDX_CHAR_CFG_PRESS] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, PERM_READ_WRITE, 2 * sizeof(uint8_t), 2 * sizeof(uint8_t), (uint8_t *)press_ccc}},

    [ENV_SENS_IDX_CHAR_IAQ] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, PERM_READ, sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_read_notify}},
    [ENV_SENS_IDX_CHAR_VAL_IAQ] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_IAQ, PERM_READ, sizeof(uint16_t), sizeof(uint16_t), (uint8_t *)&iaq_val}},
    [ENV_SENS_IDX_CHAR_CFG_IAQ] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, PERM_READ_WRITE, 2 * sizeof(uint8_t), 2 * sizeof(uint8_t), (uint8_t *)iaq_ccc}},

    [ENV_SENS_IDX_CHAR_CO2] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, PERM_READ, sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_read_notify}},
    [ENV_SENS_IDX_CHAR_VAL_CO2] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_CO2, PERM_READ, sizeof(uint16_t), sizeof(uint16_t), (uint8_t *)&co2_val}},
    [ENV_SENS_IDX_CHAR_CFG_CO2] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, PERM_READ_WRITE, 2 * sizeof(uint8_t), 2 * sizeof(uint8_t), (uint8_t *)co2_ccc}},

    [ENV_SENS_IDX_CHAR_VOC] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, PERM_READ, sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_read_notify}},
    [ENV_SENS_IDX_CHAR_VAL_VOC] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_VOC, PERM_READ, sizeof(uint16_t), sizeof(uint16_t), (uint8_t *)&voc_val}},
    [ENV_SENS_IDX_CHAR_CFG_VOC] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, PERM_READ_WRITE, 2 * sizeof(uint8_t), 2 * sizeof(uint8_t), (uint8_t *)voc_ccc}},

    [ENV_SENS_IDX_CHAR_OUT_HUM] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, PERM_READ, sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_read_notify}},
    [ENV_SENS_IDX_CHAR_VAL_OUT_HUM] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_OUT_HUM, PERM_READ, sizeof(uint16_t), sizeof(uint16_t), (uint8_t *)&out_hum_val}},
    [ENV_SENS_IDX_CHAR_CFG_OUT_HUM] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, PERM_READ_WRITE, 2 * sizeof(uint8_t), 2 * sizeof(uint8_t), (uint8_t *)out_hum_ccc}},

    [ENV_SENS_IDX_CHAR_OUT_TEMP] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, PERM_READ, sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_read_notify}},
    [ENV_SENS_IDX_CHAR_VAL_OUT_TEMP] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_OUT_TEMP, PERM_READ, sizeof(int16_t), sizeof(int16_t), (uint8_t *)&out_temp_val}},
    [ENV_SENS_IDX_CHAR_CFG_OUT_TEMP] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, PERM_READ_WRITE, 2 * sizeof(uint8_t), 2 * sizeof(uint8_t), (uint8_t *)out_temp_ccc}},

};

static const esp_gatts_attr_db_t gatt_bat_serv_db[BAT_SERV_IDX_NB] = {
    [BAT_SERV_IDX_SVC] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, PERM_READ, sizeof(uint16_t), sizeof(uint16_t), (uint8_t *)&GATTS_SERVICE_UUID_BAT_SERV}},

    [BAT_SERV_IDX_CHAR_BAT_LVL] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, PERM_READ, sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_read_notify}},
    [BAT_SERV_IDX_CHAR_VAL_BAT_LVL] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_OUT_BAT_LVL, PERM_READ, sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&out_bat_lvl_val}},
    [BAT_SERV_IDX_CHAR_CFG_BAT_LVL] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, PERM_READ_WRITE, 2 * sizeof(uint8_t), 2 * sizeof(uint8_t), (uint8_t *)out_bat_lvl_ccc}},

};

static const esp_gatts_attr_db_t gatt_cur_time_db[CUR_TIME_IDX_NB] = {
    [CUR_TIME_IDX_SVC] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, PERM_READ, sizeof(uint16_t), sizeof(uint16_t), (uint8_t *)&GATTS_SERVICE_UUID_CUR_TIME}},

    [CUR_TIME_IDX_CHAR_DATE_TIME] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, PERM_READ, sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_read_write}},
    [CUR_TIME_IDX_CHAR_VAL_DATE_TIME] =
        {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_DATE_TIME, PERM_READ_WRITE, sizeof(gatt_date_time_t), 0, NULL}},
    [CUR_TIME_IDX_CHAR_CFG_DATE_TIME] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, PERM_READ_WRITE, 2 * sizeof(uint8_t), 2 * sizeof(uint8_t), (uint8_t *)date_time_ccc}},

};

// @formatter:on

static uint16_t env_sens_handle_table[ENV_SENS_IDX_NB];
static uint16_t bat_serv_handle_table[BAT_SERV_IDX_NB];
static uint16_t cur_time_handle_table[CUR_TIME_IDX_NB];

static uint16_t ble_gatts_if;
static uint16_t ble_connection_id;
static bool ble_has_connection;
static QueueHandle_t ble_connection_queue;

void gatts_read(esp_gatt_if_t gatts_if, struct gatts_read_evt_param read)
{
    esp_gatt_rsp_t rsp;
    rsp.attr_value.handle = read.handle;
    rsp.attr_value.len = 0;

    if (read.handle == cur_time_handle_table[CUR_TIME_IDX_CHAR_VAL_DATE_TIME]) {
        rsp.attr_value.len = sizeof(gatt_date_time_t);
        gatt_date_time_t *date_time = (gatt_date_time_t*) rsp.attr_value.value;
        struct timeval timeval;
        gettimeofday(&timeval, NULL);
        struct tm tm = *localtime(&timeval.tv_sec);
        date_time->year = 0;
        date_time->month = 0;
        date_time->day = 0;
        date_time->hours = tm.tm_hour;
        date_time->minutes = tm.tm_min;
        date_time->seconds = tm.tm_sec;
    }

    if (rsp.attr_value.len > 0) {
        esp_ble_gatts_send_response(gatts_if, read.conn_id, read.trans_id, ESP_GATT_OK, &rsp);
    }
}

void gatts_write(esp_gatt_if_t gatts_if, struct gatts_write_evt_param write)
{
    if (write.handle == cur_time_handle_table[CUR_TIME_IDX_CHAR_VAL_DATE_TIME]) {
        gatt_date_time_t *date_time = (gatt_date_time_t*) write.value;

        struct timeval timeval;
        gettimeofday(&timeval, NULL);
        struct tm tm = *localtime(&timeval.tv_sec);
        tm.tm_hour = date_time->hours;
        tm.tm_min = date_time->minutes;
        tm.tm_sec = date_time->seconds;
        timeval.tv_sec = mktime(&tm);
        settimeofday(&timeval, NULL);
    }

    if (write.need_rsp) {
        if (write.is_prep) {
            esp_gatt_rsp_t gatt_rsp;
            gatt_rsp.attr_value.len = write.len;
            gatt_rsp.attr_value.handle = write.handle;
            gatt_rsp.attr_value.offset = write.offset;
            memcpy(gatt_rsp.attr_value.value, write.value, write.len);
            gatt_rsp.attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
            esp_ble_gatts_send_response(gatts_if, write.conn_id, write.trans_id, ESP_GATT_OK, &gatt_rsp);
        } else {
            esp_ble_gatts_send_response(gatts_if, write.conn_id, write.trans_id, ESP_GATT_OK, NULL);
        }
    }
}

void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGD(TAG, "Gatts event handler [event: %d]", event);
    switch (event) {
        case ESP_GATTS_REG_EVT:
            ble_gatts_if = gatts_if;
            esp_ble_gap_set_device_name(DEVICE_NAME);

            esp_ble_gap_config_local_privacy(false);

            esp_ble_gatts_create_attr_tab(gatt_env_sens_db, gatts_if, ENV_SENS_IDX_NB, 0);
            esp_ble_gatts_create_attr_tab(gatt_bat_serv_db, gatts_if, BAT_SERV_IDX_NB, 0);
            esp_ble_gatts_create_attr_tab(gatt_cur_time_db, gatts_if, CUR_TIME_IDX_NB, 0);
            break;
        case ESP_GATTS_CREAT_ATTR_TAB_EVT:
            if (param->add_attr_tab.svc_uuid.uuid.uuid16 == GATTS_SERVICE_UUID_ENV_SENS) {
                memcpy(env_sens_handle_table, param->add_attr_tab.handles, sizeof(env_sens_handle_table));
                esp_ble_gatts_start_service(env_sens_handle_table[ENV_SENS_IDX_SVC]);
            }
            if (param->add_attr_tab.svc_uuid.uuid.uuid16 == GATTS_SERVICE_UUID_BAT_SERV) {
                memcpy(bat_serv_handle_table, param->add_attr_tab.handles, sizeof(bat_serv_handle_table));
                esp_ble_gatts_start_service(bat_serv_handle_table[BAT_SERV_IDX_SVC]);
            }
            if (param->add_attr_tab.svc_uuid.uuid.uuid16 == GATTS_SERVICE_UUID_CUR_TIME) {
                memcpy(cur_time_handle_table, param->add_attr_tab.handles, sizeof(cur_time_handle_table));
                esp_ble_gatts_start_service(cur_time_handle_table[BAT_SERV_IDX_SVC]);
            }
            break;
        case ESP_GATTS_CONNECT_EVT:
            ble_connection_id = param->connect.conn_id;
            esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT_MITM);
            break;
        case ESP_GATTS_DISCONNECT_EVT:
            ble_has_connection = false;
            xQueueSend(ble_connection_queue, &ble_has_connection, portMAX_DELAY);
            esp_ble_gap_start_advertising(&adv_params);
            break;
        case ESP_GATTS_READ_EVT:
            gatts_read(gatts_if, param->read);
            break;
        case ESP_GATTS_WRITE_EVT:
            gatts_write(gatts_if, param->write);
            break;
        default:
            break;
    }
}

void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    ESP_LOGD(TAG, "Gap event handler [event: %d]", event);
    switch (event) {
        case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
            if (adv_config_done == 0) {
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~ADV_CONFIG_FLAG);
            if (adv_config_done == 0) {
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_SEC_REQ_EVT:
            esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
            break;
        case ESP_GAP_BLE_SET_LOCAL_PRIVACY_COMPLETE_EVT:
            esp_ble_gap_config_adv_data(&adv_data);
            adv_config_done |= ADV_CONFIG_FLAG;

            esp_ble_gap_config_adv_data(&scan_rsp_data);
            adv_config_done |= SCAN_RSP_CONFIG_FLAG;
            break;
        case ESP_GAP_BLE_AUTH_CMPL_EVT:
            if (param->ble_security.auth_cmpl.success) {
                ble_has_connection = true;
                xQueueSend(ble_connection_queue, &ble_has_connection, 0);
            }
            break;
        default:
            break;
    }
}

void ble_init(QueueHandle_t connection_queue)
{
    ble_connection_queue = connection_queue;

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT()
    ;
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));

    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));

    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_event_handler));
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));

    ESP_ERROR_CHECK(esp_ble_gatts_app_register(PROFILE_APP_ID));

    ESP_ERROR_CHECK(esp_ble_gatt_set_local_mtu(517));
   // ESP_ERROR_CHECK(esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_N12));

    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_MITM;
    esp_ble_io_cap_t iocap = ESP_IO_CAP_OUT;
    uint8_t key_size = 16;
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK | ESP_BLE_CSR_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK | ESP_BLE_CSR_KEY_MASK;
    uint8_t auth_option = ESP_BLE_ONLY_ACCEPT_SPECIFIED_AUTH_DISABLE;
    uint8_t oob_support = ESP_BLE_OOB_DISABLE;
    ESP_ERROR_CHECK(esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t)));
    ESP_ERROR_CHECK(esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t)));
    ESP_ERROR_CHECK(esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t)));
    ESP_ERROR_CHECK(
            esp_ble_gap_set_security_param(ESP_BLE_SM_ONLY_ACCEPT_SPECIFIED_SEC_AUTH, &auth_option, sizeof(uint8_t)));
    esp_ble_gap_set_security_param(ESP_BLE_SM_OOB_SUPPORT, &oob_support, sizeof(uint8_t));
    ESP_ERROR_CHECK(esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t)));
    ESP_ERROR_CHECK(esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t)));
}

void ble_set_telemetry(float humidity, float temperature, float pressure, float iaq, float co2, float voc)
{
    hum_val = humidity * 100;
    temp_val = temperature * 100;
    press_val = pressure * 10;
    iaq_val = roundf(iaq);
    co2_val = round(co2);
    voc_val = round(voc * 1000);

    esp_ble_gatts_set_attr_value(env_sens_handle_table[ENV_SENS_IDX_CHAR_VAL_HUM], sizeof(uint16_t),
            (uint8_t *) &hum_val);
    esp_ble_gatts_set_attr_value(env_sens_handle_table[ENV_SENS_IDX_CHAR_VAL_TEMP], sizeof(int16_t),
            (uint8_t *) &temp_val);
    esp_ble_gatts_set_attr_value(env_sens_handle_table[ENV_SENS_IDX_CHAR_VAL_PRESS], sizeof(uint32_t),
            (uint8_t *) &press_val);
    esp_ble_gatts_set_attr_value(env_sens_handle_table[ENV_SENS_IDX_CHAR_VAL_IAQ], sizeof(uint16_t),
            (uint8_t *) &iaq_val);
    esp_ble_gatts_set_attr_value(env_sens_handle_table[ENV_SENS_IDX_CHAR_VAL_CO2], sizeof(uint16_t),
            (uint8_t *) &co2_val);
    esp_ble_gatts_set_attr_value(env_sens_handle_table[ENV_SENS_IDX_CHAR_VAL_VOC], sizeof(uint16_t),
            (uint8_t *) &voc_val);
    if (ble_has_connection) {
        esp_ble_gatts_send_indicate(ble_gatts_if, ble_connection_id, env_sens_handle_table[ENV_SENS_IDX_CHAR_VAL_HUM],
                sizeof(uint16_t), (uint8_t *) &hum_val,
                false);
        esp_ble_gatts_send_indicate(ble_gatts_if, ble_connection_id, env_sens_handle_table[ENV_SENS_IDX_CHAR_VAL_TEMP],
                sizeof(int16_t), (uint8_t *) &temp_val,
                false);
        esp_ble_gatts_send_indicate(ble_gatts_if, ble_connection_id, env_sens_handle_table[ENV_SENS_IDX_CHAR_VAL_PRESS],
                sizeof(uint32_t), (uint8_t *) &press_val,
                false);
        esp_ble_gatts_send_indicate(ble_gatts_if, ble_connection_id, env_sens_handle_table[ENV_SENS_IDX_CHAR_VAL_IAQ],
                sizeof(uint16_t), (uint8_t *) &iaq_val,
                false);
        esp_ble_gatts_send_indicate(ble_gatts_if, ble_connection_id, env_sens_handle_table[ENV_SENS_IDX_CHAR_VAL_CO2],
                sizeof(uint16_t), (uint8_t *) &co2_val,
                false);
        esp_ble_gatts_send_indicate(ble_gatts_if, ble_connection_id, env_sens_handle_table[ENV_SENS_IDX_CHAR_VAL_VOC],
                sizeof(uint16_t), (uint8_t *) &voc_val,
                false);
    }
}

void ble_set_out_telemetry(float humidity, float temperature, uint8_t battery)
{
    out_hum_val = humidity * 100;
    out_temp_val = temperature * 100;
    out_bat_lvl_val = battery;

    esp_ble_gatts_set_attr_value(env_sens_handle_table[ENV_SENS_IDX_CHAR_VAL_OUT_HUM], sizeof(uint16_t),
            (uint8_t *) &out_hum_val);
    esp_ble_gatts_set_attr_value(env_sens_handle_table[ENV_SENS_IDX_CHAR_VAL_OUT_TEMP], sizeof(int16_t),
            (uint8_t *) &out_temp_val);
    esp_ble_gatts_set_attr_value(bat_serv_handle_table[BAT_SERV_IDX_CHAR_VAL_BAT_LVL], sizeof(uint8_t),
            (uint8_t *) &out_bat_lvl_val);
    if (ble_has_connection) {
        esp_ble_gatts_send_indicate(ble_gatts_if, ble_connection_id,
                env_sens_handle_table[ENV_SENS_IDX_CHAR_VAL_OUT_HUM], sizeof(uint16_t), (uint8_t *) &out_hum_val,
                false);
        esp_ble_gatts_send_indicate(ble_gatts_if, ble_connection_id,
                env_sens_handle_table[ENV_SENS_IDX_CHAR_VAL_OUT_TEMP], sizeof(int16_t), (uint8_t *) &out_temp_val,
                false);
        esp_ble_gatts_send_indicate(ble_gatts_if, ble_connection_id,
                bat_serv_handle_table[BAT_SERV_IDX_CHAR_VAL_BAT_LVL], sizeof(uint8_t), (uint8_t *) &out_bat_lvl_val,
                false);
    }
}

void ble_enable_pairing(uint32_t passkey)
{
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_MITM_BOND;
    ESP_ERROR_CHECK(esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t)));
    ESP_ERROR_CHECK(esp_ble_gap_set_security_param(ESP_BLE_SM_SET_STATIC_PASSKEY, &passkey, sizeof(uint32_t)));
}

void ble_disable_pairing()
{
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_MITM;
    ESP_ERROR_CHECK(esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t)));
}

void ble_remove_paired_device()
{
    int dev_num = esp_ble_get_bond_device_num();

    ESP_LOGI(TAG, "Removing %d paired devices", dev_num);

    esp_ble_bond_dev_t *dev_list = (esp_ble_bond_dev_t *) malloc(sizeof(esp_ble_bond_dev_t) * dev_num);
    esp_ble_get_bond_device_list(&dev_num, dev_list);
    for (int i = 0; i < dev_num; i++) {
        esp_ble_remove_bond_device(dev_list[i].bd_addr);
    }

    free(dev_list);
}
