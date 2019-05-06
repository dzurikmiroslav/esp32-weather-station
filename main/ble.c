#include <string.h>
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

#define LOG_TAG "BLE"

#define ENABLE_BLE_SECURITY CONFIG_ENABLE_BLE_SECURITY
#define DEVICE_NAME "ESP_METEO"
#define PROFILE_APP_ID 0

#if ENABLE_BLE_SECURITY
#define PERM_READ ESP_GATT_PERM_READ_ENC_MITM
#define PERM_READ_WRITE ESP_GATT_PERM_READ_ENC_MITM | ESP_GATT_PERM_WRITE_ENC_MITM
#else
#define PERM_READ ESP_GATT_PERM_READ
#define PERM_READ_WRITE ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE
#endif /* ENABLE_BLE_SECURITY */

#define ADV_CONFIG_FLAG (1 << 0)
#define SCAN_RSP_CONFIG_FLAG (1 << 1)

static uint8_t adv_config_done = 0;

static uint8_t service_uuid[16] = {0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x18, 0x1A, 0x00, 0x00};

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

    ENV_SENS_IDX_CHAR_INT_HUM,
    ENV_SENS_IDX_CHAR_VAL_INT_HUM,
    ENV_SENS_IDX_CHAR_CFG_INT_HUM,

    ENV_SENS_IDX_CHAR_INT_TEMP,
    ENV_SENS_IDX_CHAR_VAL_INT_TEMP,
    ENV_SENS_IDX_CHAR_CFG_INT_TEMP,

    ENV_SENS_IDX_CHAR_INT_CO2,
    ENV_SENS_IDX_CHAR_VAL_INT_CO2,
    ENV_SENS_IDX_CHAR_CFG_INT_CO2,

    ENV_SENS_IDX_CHAR_INT_TVOC,
    ENV_SENS_IDX_CHAR_VAL_INT_TVOC,
    ENV_SENS_IDX_CHAR_CFG_INT_TVOC,

    ENV_SENS_IDX_CHAR_EXT_HUM,
    ENV_SENS_IDX_CHAR_VAL_EXT_HUM,
    ENV_SENS_IDX_CHAR_CFG_EXT_HUM,

    ENV_SENS_IDX_CHAR_EXT_TEMP,
    ENV_SENS_IDX_CHAR_VAL_EXT_TEMP,
    ENV_SENS_IDX_CHAR_CFG_EXT_TEMP,

    ENV_SENS_IDX_CHAR_EXT_PRESS,
    ENV_SENS_IDX_CHAR_VAL_EXT_PRESS,
    ENV_SENS_IDX_CHAR_CFG_EXT_PRESS,

    ENV_SENS_IDX_NB
};

static const uint16_t GATTS_SERVICE_UUID_ENV_SENS = 0x181A;
static const uint16_t GATTS_CHAR_UUID_INT_HUM = 0x2A6F;
static const uint16_t GATTS_CHAR_UUID_INT_TEMP = 0x2A6E;
static const uint16_t GATTS_CHAR_UUID_INT_CO2 = 0x2F01;
static const uint16_t GATTS_CHAR_UUID_INT_TVOC = 0x2F02;
static const uint16_t GATTS_CHAR_UUID_EXT_HUM = 0x3A6F;
static const uint16_t GATTS_CHAR_UUID_EXT_TEMP = 0x3A6E;
static const uint16_t GATTS_CHAR_UUID_EXT_PRESS = 0x3A6D;

static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint8_t char_prop_read_notify = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;

static const uint8_t int_hum_ccc[2] = {0x00, 0x00};
static uint16_t int_hum_val = 20;

static const uint8_t int_temp_ccc[2] = {0x00, 0x00};
static int16_t int_temp_val = 0;

static uint16_t int_co2_val = 0;
static uint8_t int_co2_ccc[2] = {0x00, 0x00};

static uint16_t int_tvoc_val = 0;
static uint8_t int_tvoc_ccc[2] = {0x00, 0x00};

static uint16_t ext_hum_val = 0;
static uint8_t ext_hum_ccc[2] = {0x00, 0x00};

static int16_t ext_temp_val = 0;
static uint8_t ext_temp_ccc[2] = {0x00, 0x00};

static uint32_t ext_press_val = 0;
static uint8_t ext_press_ccc[2] = {0x00, 0x00};

static const esp_gatts_attr_db_t gatt_env_sens_db[ENV_SENS_IDX_NB] = {
    [ENV_SENS_IDX_SVC] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, PERM_READ, sizeof(uint16_t), sizeof(uint16_t), (uint8_t *)&GATTS_SERVICE_UUID_ENV_SENS}},

    [ENV_SENS_IDX_CHAR_INT_HUM] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, PERM_READ, sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_read_notify}},
    [ENV_SENS_IDX_CHAR_VAL_INT_HUM] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_INT_HUM, PERM_READ, sizeof(uint16_t), sizeof(uint16_t), (uint8_t *)&int_hum_val}},
    [ENV_SENS_IDX_CHAR_CFG_INT_HUM] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, PERM_READ_WRITE, 2 * sizeof(uint8_t), 2 * sizeof(uint8_t), (uint8_t *)int_hum_ccc}},

    [ENV_SENS_IDX_CHAR_INT_TEMP] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, PERM_READ, sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_read_notify}},
    [ENV_SENS_IDX_CHAR_VAL_INT_TEMP] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_INT_TEMP, PERM_READ, sizeof(int16_t), sizeof(int16_t), (uint8_t *)&int_temp_val}},
    [ENV_SENS_IDX_CHAR_CFG_INT_TEMP] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, PERM_READ_WRITE, 2 * sizeof(uint8_t), 2 * sizeof(uint8_t), (uint8_t *)int_temp_ccc}},

    [ENV_SENS_IDX_CHAR_INT_CO2] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, PERM_READ, sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_read_notify}},
    [ENV_SENS_IDX_CHAR_VAL_INT_CO2] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_INT_CO2, PERM_READ, sizeof(uint16_t), sizeof(uint16_t), (uint8_t *)&int_co2_val}},
    [ENV_SENS_IDX_CHAR_CFG_INT_CO2] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, PERM_READ_WRITE, 2 * sizeof(uint8_t), 2 * sizeof(uint8_t), (uint8_t *)int_co2_ccc}},

    [ENV_SENS_IDX_CHAR_INT_TVOC] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, PERM_READ, sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_read_notify}},
    [ENV_SENS_IDX_CHAR_VAL_INT_TVOC] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_INT_TVOC, PERM_READ, sizeof(uint16_t), sizeof(uint16_t), (uint8_t *)&int_tvoc_val}},
    [ENV_SENS_IDX_CHAR_CFG_INT_TVOC] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, PERM_READ_WRITE, 2 * sizeof(uint8_t), 2 * sizeof(uint8_t), (uint8_t *)int_tvoc_ccc}},

    [ENV_SENS_IDX_CHAR_EXT_HUM] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, PERM_READ, sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_read_notify}},
    [ENV_SENS_IDX_CHAR_VAL_EXT_HUM] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_EXT_HUM, PERM_READ, sizeof(uint16_t), sizeof(uint16_t), (uint8_t *)&ext_hum_val}},
    [ENV_SENS_IDX_CHAR_CFG_EXT_HUM] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, PERM_READ_WRITE, 2 * sizeof(uint8_t), 2 * sizeof(uint8_t), (uint8_t *)ext_hum_ccc}},

    [ENV_SENS_IDX_CHAR_EXT_TEMP] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, PERM_READ, sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_read_notify}},
    [ENV_SENS_IDX_CHAR_VAL_EXT_TEMP] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_EXT_TEMP, PERM_READ, sizeof(int16_t), sizeof(int16_t), (uint8_t *)&ext_temp_val}},
    [ENV_SENS_IDX_CHAR_CFG_EXT_TEMP] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, PERM_READ_WRITE, 2 * sizeof(uint8_t), 2 * sizeof(uint8_t), (uint8_t *)ext_temp_ccc}},

    [ENV_SENS_IDX_CHAR_EXT_PRESS] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, PERM_READ, sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_read_notify}},
    [ENV_SENS_IDX_CHAR_VAL_EXT_PRESS] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_EXT_PRESS, PERM_READ, sizeof(uint32_t), sizeof(uint32_t), (uint8_t *)&ext_press_val}},
    [ENV_SENS_IDX_CHAR_CFG_EXT_PRESS] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, PERM_READ_WRITE, 2 * sizeof(uint8_t), 2 * sizeof(uint8_t), (uint8_t *)ext_press_ccc}},
};

uint16_t env_sens_handle_table[ENV_SENS_IDX_NB];

static uint16_t ble_gatts_if;
static uint16_t ble_connection_id;
static bool ble_has_connection;
static QueueHandle_t ble_connection_queue;

void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGD(LOG_TAG, "Gatts event handler [event: %d]", event);
    switch (event)
    {
    case ESP_GATTS_REG_EVT:
        ble_gatts_if = gatts_if;
        esp_ble_gap_set_device_name(DEVICE_NAME);
#if ENABLE_BLE_SECURITY
        esp_ble_gap_config_local_privacy(false);
#else
        esp_ble_gap_config_adv_data(&adv_data);
        adv_config_done |= ADV_CONFIG_FLAG;

        esp_ble_gap_config_adv_data(&scan_rsp_data);
        adv_config_done |= SCAN_RSP_CONFIG_FLAG;
#endif /* ENABLE_BLE_SECURITY */
        esp_ble_gatts_create_attr_tab(gatt_env_sens_db, gatts_if, ENV_SENS_IDX_NB, 0);
        break;
    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
        memcpy(env_sens_handle_table, param->add_attr_tab.handles, sizeof(env_sens_handle_table));
        esp_ble_gatts_start_service(env_sens_handle_table[ENV_SENS_IDX_SVC]);
        break;
    case ESP_GATTS_CONNECT_EVT:
        ble_connection_id = param->connect.conn_id;
#if ENABLE_BLE_SECURITY
        esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT_MITM);
#else
        ble_has_connection = true;
        xQueueSend(ble_connection_queue, &ble_has_connection, 0);
#endif /* ENABLE_BLE_SECURITY */
        break;
    case ESP_GATTS_DISCONNECT_EVT:
        ble_has_connection = false;
        xQueueSend(ble_connection_queue, &ble_has_connection, portMAX_DELAY);
        esp_ble_gap_start_advertising(&adv_params);
        break;
    default:
        break;
    }
}

void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    ESP_LOGD(LOG_TAG, "Gap event handler [event: %d]", event);
    switch (event)
    {
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
        if (adv_config_done == 0)
        {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~ADV_CONFIG_FLAG);
        if (adv_config_done == 0)
        {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
#if ENABLE_BLE_SECURITY
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
        if (param->ble_security.auth_cmpl.success)
        {
            ble_has_connection = true;
            xQueueSend(ble_connection_queue, &ble_has_connection, 0);
        }
        break;
#endif /* ENABLE_BLE_SECURITY */
    default:
        break;
    }
}

void ble_init(QueueHandle_t connection_queue)
{
    ble_connection_queue = connection_queue;

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));

    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));

    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_event_handler));
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));

    ESP_ERROR_CHECK(esp_ble_gatts_app_register(PROFILE_APP_ID));

    ESP_ERROR_CHECK(esp_ble_gatt_set_local_mtu(500));
    ESP_ERROR_CHECK(esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_N12));

#if ENABLE_BLE_SECURITY
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_MITM_BOND;
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;
    uint8_t key_size = 16;
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;

    uint8_t auth_option = ESP_BLE_ONLY_ACCEPT_SPECIFIED_AUTH_ENABLE;
    ESP_ERROR_CHECK(esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t)));
    ESP_ERROR_CHECK(esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t)));
    ESP_ERROR_CHECK(esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t)));
    ESP_ERROR_CHECK(esp_ble_gap_set_security_param(ESP_BLE_SM_ONLY_ACCEPT_SPECIFIED_SEC_AUTH, &auth_option, sizeof(uint8_t)));
    ESP_ERROR_CHECK(esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t)));
    ESP_ERROR_CHECK(esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t)));
#endif /* ENABLE_BLE_SECURITY */
}

void ble_set_int_humidity_temperature(float humidity, float temperature)
{
    int_hum_val = humidity * 100;
    int_temp_val = temperature * 100;

    esp_ble_gatts_set_attr_value(env_sens_handle_table[ENV_SENS_IDX_CHAR_VAL_INT_HUM], sizeof(uint16_t), (uint8_t *)&int_hum_val);
    esp_ble_gatts_set_attr_value(env_sens_handle_table[ENV_SENS_IDX_CHAR_VAL_INT_TEMP], sizeof(int16_t), (uint8_t *)&int_temp_val);
    if (ble_has_connection)
    {
        esp_ble_gatts_send_indicate(ble_gatts_if, ble_connection_id, env_sens_handle_table[ENV_SENS_IDX_CHAR_VAL_INT_HUM], sizeof(uint16_t), (uint8_t *)&int_hum_val, false);
        esp_ble_gatts_send_indicate(ble_gatts_if, ble_connection_id, env_sens_handle_table[ENV_SENS_IDX_CHAR_VAL_INT_TEMP], sizeof(int16_t), (uint8_t *)&int_temp_val, false);
    }
}

void ble_set_int_co2_tvoc(uint16_t co2, uint16_t tvoc)
{
    int_co2_val = co2;
    int_tvoc_val = tvoc;

    esp_ble_gatts_set_attr_value(env_sens_handle_table[ENV_SENS_IDX_CHAR_VAL_INT_CO2], sizeof(uint16_t), (uint8_t *)&int_co2_val);
    esp_ble_gatts_set_attr_value(env_sens_handle_table[ENV_SENS_IDX_CHAR_VAL_INT_TVOC], sizeof(uint16_t), (uint8_t *)&int_tvoc_val);
    if (ble_has_connection)
    {
        esp_ble_gatts_send_indicate(ble_gatts_if, ble_connection_id, env_sens_handle_table[ENV_SENS_IDX_CHAR_VAL_INT_CO2], sizeof(uint16_t), (uint8_t *)&int_co2_val, false);
        esp_ble_gatts_send_indicate(ble_gatts_if, ble_connection_id, env_sens_handle_table[ENV_SENS_IDX_CHAR_VAL_INT_TVOC], sizeof(uint16_t), (uint8_t *)&int_tvoc_val, false);
    }
}

void ble_set_ext_humidity_temperature_pressure(float humidity, float temperature, float pressure)
{
    ext_hum_val = humidity * 100;
    ext_temp_val = temperature * 100;
    ext_press_val = pressure * 100 * 10; /*hPa -> Pa*/

    esp_ble_gatts_set_attr_value(env_sens_handle_table[ENV_SENS_IDX_CHAR_VAL_EXT_HUM], sizeof(uint16_t), (uint8_t *)&ext_hum_val);
    esp_ble_gatts_set_attr_value(env_sens_handle_table[ENV_SENS_IDX_CHAR_VAL_EXT_TEMP], sizeof(int16_t), (uint8_t *)&ext_temp_val);
    esp_ble_gatts_set_attr_value(env_sens_handle_table[ENV_SENS_IDX_CHAR_VAL_EXT_PRESS], sizeof(uint32_t), (uint8_t *)&ext_press_val);
    if (ble_has_connection)
    {
        esp_ble_gatts_send_indicate(ble_gatts_if, ble_connection_id, env_sens_handle_table[ENV_SENS_IDX_CHAR_VAL_EXT_HUM], sizeof(uint16_t), (uint8_t *)&ext_hum_val, false);
        esp_ble_gatts_send_indicate(ble_gatts_if, ble_connection_id, env_sens_handle_table[ENV_SENS_IDX_CHAR_VAL_EXT_TEMP], sizeof(int16_t), (uint8_t *)&ext_temp_val, false);
        esp_ble_gatts_send_indicate(ble_gatts_if, ble_connection_id, env_sens_handle_table[ENV_SENS_IDX_CHAR_VAL_EXT_PRESS], sizeof(uint32_t), (uint8_t *)&ext_press_val, false);
    }
}

void ble_enable_pairing(uint32_t passkey)
{
#if ENABLE_BLE_SECURITY
    esp_ble_io_cap_t iocap = ESP_IO_CAP_OUT;
    uint8_t auth_option = ESP_BLE_ONLY_ACCEPT_SPECIFIED_AUTH_DISABLE;
    ESP_ERROR_CHECK(esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t)));
    ESP_ERROR_CHECK(esp_ble_gap_set_security_param(ESP_BLE_SM_ONLY_ACCEPT_SPECIFIED_SEC_AUTH, &auth_option, sizeof(uint8_t)));
    ESP_ERROR_CHECK(esp_ble_gap_set_security_param(ESP_BLE_SM_SET_STATIC_PASSKEY, &passkey, sizeof(uint32_t)));
#else
    ESP_LOGE(LOG_TAG, "This feature is not supported");
#endif /* ENABLE_BLE_SECURITY */
}

void ble_disable_pairing()
{
#if ENABLE_BLE_SECURITY
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;
    uint8_t auth_option = ESP_BLE_ONLY_ACCEPT_SPECIFIED_AUTH_ENABLE;
    ESP_ERROR_CHECK(esp_ble_gap_set_security_param(ESP_BLE_SM_ONLY_ACCEPT_SPECIFIED_SEC_AUTH, &auth_option, sizeof(uint8_t)));
    ESP_ERROR_CHECK(esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t)));
#else
    ESP_LOGE(LOG_TAG, "This feature is not supported");
#endif /* ENABLE_BLE_SECURITY */
}

void ble_remove_paired_device()
{
#if ENABLE_BLE_SECURITY
    int dev_num = esp_ble_get_bond_device_num();

    ESP_LOGI(LOG_TAG, "Removing %d paired devices", dev_num);

    esp_ble_bond_dev_t *dev_list = (esp_ble_bond_dev_t *)malloc(sizeof(esp_ble_bond_dev_t) * dev_num);
    esp_ble_get_bond_device_list(&dev_num, dev_list);
    for (int i = 0; i < dev_num; i++)
    {
        esp_ble_remove_bond_device(dev_list[i].bd_addr);
    }

    free(dev_list);
#else
    ESP_LOGE(LOG_TAG, "This feature is not supported");
#endif /* ENABLE_BLE_SECURITY */
}
