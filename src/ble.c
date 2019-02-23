#include <freertos/FreeRTOS.h>
#include <esp_system.h>
#include <esp_log.h>
#include <esp_bt.h>
#include <esp_gap_ble_api.h>
#include <esp_gatts_api.h>
#include <esp_bt_defs.h>
#include <esp_bt_main.h>
#include <esp_gatt_common_api.h>
#include <string.h>

#include "ble.h"

#define LOG_TAG "BLE"

#define DEVICE_NAME "ESP_METEO"
#define SVC_INT_ENVIROMENTAL_SENSING_ID 0
#define SVC_EXT_ENVIROMENTAL_SENSING_ID 1
#define PERM_READ ESP_GATT_PERM_READ_ENCRYPTED
#define PERM_READ_WRITE ESP_GATT_PERM_READ_ENCRYPTED | ESP_GATT_PERM_WRITE_ENCRYPTED
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
    .own_addr_type = BLE_ADDR_TYPE_RANDOM,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

enum
{
    INT_ENV_IDX_SVC,

    INT_ENV_IDX_CHAR_HUM,
    INT_ENV_IDX_CHAR_VAL_HUM,
    INT_ENV_IDX_CHAR_CFG_HUM,

    INT_ENV_IDX_CHAR_TEMP,
    INT_ENV_IDX_CHAR_VAL_TEMP,
    INT_ENV_IDX_CHAR_CFG_TEMP,

    INT_ENV_IDX_CHAR_CO2,
    INT_ENV_IDX_CHAR_VAL_CO2,
    INT_ENV_IDX_CHAR_CFG_CO2,

    INT_ENV_IDX_CHAR_TVOC,
    INT_ENV_IDX_CHAR_VAL_TVOC,
    INT_ENV_IDX_CHAR_CFG_TVOC,

    INT_ENV_IDX_NB
};

enum
{
    EXT_ENV_IDX_SVC,

    EXT_ENV_IDX_CHAR_HUM,
    EXT_ENV_IDX_CHAR_VAL_HUM,
    EXT_ENV_IDX_CHAR_CFG_HUM,

    EXT_ENV_IDX_CHAR_TEMP,
    EXT_ENV_IDX_CHAR_VAL_TEMP,
    EXT_ENV_IDX_CHAR_CFG_TEMP,

    EXT_ENV_IDX_CHAR_PRESS,
    EXT_ENV_IDX_CHAR_VAL_PRESS,
    EXT_ENV_IDX_CHAR_CFG_PRESS,

    EXT_ENV_IDX_NB
};

static const uint16_t GATTS_SERVICE_UUID_ENV_SENS = 0x181A;
static const uint16_t GATTS_CHAR_UUID_HUM = 0x2A6F;
static const uint16_t GATTS_CHAR_UUID_TEMP = 0x2A6E;
static const uint16_t GATTS_CHAR_UUID_PRESS = 0x2A6D;
static const uint16_t GATTS_CHAR_UUID_CO2 = 0xFF01;
static const uint16_t GATTS_CHAR_UUID_TVOC = 0xFF02;

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

static const esp_gatts_attr_db_t gatt_int_env_db[INT_ENV_IDX_NB] = {
    [INT_ENV_IDX_SVC] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, PERM_READ, sizeof(uint16_t), sizeof(GATTS_SERVICE_UUID_ENV_SENS), (uint8_t *)&GATTS_SERVICE_UUID_ENV_SENS}},

    [INT_ENV_IDX_CHAR_HUM] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, PERM_READ, sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_read_notify}},
    [INT_ENV_IDX_CHAR_VAL_HUM] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_HUM, PERM_READ, sizeof(uint16_t), sizeof(uint16_t), (uint8_t *)&int_hum_val}},
    [INT_ENV_IDX_CHAR_CFG_HUM] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, PERM_READ_WRITE, 2 * sizeof(uint8_t), 2 * sizeof(uint8_t), (uint8_t *)int_hum_ccc}},

    [INT_ENV_IDX_CHAR_TEMP] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, PERM_READ, sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_read_notify}},
    [INT_ENV_IDX_CHAR_VAL_TEMP] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEMP, PERM_READ, sizeof(int16_t), sizeof(int16_t), (uint8_t *)&int_temp_val}},
    [INT_ENV_IDX_CHAR_CFG_TEMP] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, PERM_READ_WRITE, 2 * sizeof(uint8_t), 2 * sizeof(uint8_t), (uint8_t *)int_temp_ccc}},

    [INT_ENV_IDX_CHAR_CO2] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, PERM_READ, sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_read_notify}},
    [INT_ENV_IDX_CHAR_VAL_CO2] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_CO2, PERM_READ, sizeof(uint16_t), sizeof(uint16_t), (uint8_t *)&int_co2_val}},
    [INT_ENV_IDX_CHAR_CFG_CO2] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, PERM_READ_WRITE, 2 * sizeof(uint8_t), 2 * sizeof(uint8_t), (uint8_t *)int_co2_ccc}},

    [INT_ENV_IDX_CHAR_TVOC] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, PERM_READ, sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_read_notify}},
    [INT_ENV_IDX_CHAR_VAL_TVOC] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TVOC, PERM_READ, sizeof(uint16_t), sizeof(uint16_t), (uint8_t *)&int_tvoc_val}},
    [INT_ENV_IDX_CHAR_CFG_TVOC] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, PERM_READ_WRITE, 2 * sizeof(uint8_t), 2 * sizeof(uint8_t), (uint8_t *)int_tvoc_ccc}},
};

static const esp_gatts_attr_db_t gatt_ext_env_db[EXT_ENV_IDX_NB] = {
    [EXT_ENV_IDX_SVC] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, PERM_READ, sizeof(uint16_t), sizeof(GATTS_SERVICE_UUID_ENV_SENS), (uint8_t *)&GATTS_SERVICE_UUID_ENV_SENS}},

    [EXT_ENV_IDX_CHAR_HUM] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, PERM_READ, sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_read_notify}},
    [EXT_ENV_IDX_CHAR_VAL_HUM] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_HUM, PERM_READ, sizeof(uint16_t), sizeof(uint16_t), (uint8_t *)&ext_hum_val}},
    [EXT_ENV_IDX_CHAR_CFG_HUM] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, PERM_READ_WRITE, 2 * sizeof(uint8_t), 2 * sizeof(uint8_t), (uint8_t *)ext_hum_ccc}},

    [EXT_ENV_IDX_CHAR_TEMP] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, PERM_READ, sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_read_notify}},
    [EXT_ENV_IDX_CHAR_VAL_TEMP] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEMP, PERM_READ, sizeof(int16_t), sizeof(int16_t), (uint8_t *)&ext_temp_val}},
    [EXT_ENV_IDX_CHAR_CFG_TEMP] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, PERM_READ_WRITE, 2 * sizeof(uint8_t), 2 * sizeof(uint8_t), (uint8_t *)ext_temp_ccc}},

    [EXT_ENV_IDX_CHAR_PRESS] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, PERM_READ, sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_read_notify}},
    [EXT_ENV_IDX_CHAR_VAL_PRESS] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_PRESS, PERM_READ, sizeof(uint32_t), sizeof(uint32_t), (uint8_t *)&ext_press_val}},
    [EXT_ENV_IDX_CHAR_CFG_PRESS] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, PERM_READ_WRITE, 2 * sizeof(uint8_t), 2 * sizeof(uint8_t), (uint8_t *)ext_press_ccc}},
};

uint16_t int_env_sens_handle_table[INT_ENV_IDX_NB];
uint16_t ext_env_sens_handle_table[EXT_ENV_IDX_NB];

typedef struct
{
    uint16_t gatts_if;
    uint16_t conn_id;
    bool has_conn;
} gatts_profile_inst_t;

static gatts_profile_inst_t profile;

void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGD(LOG_TAG, "Gatts event handler [event: %d]", event);
    switch (event)
    {
    case ESP_GATTS_REG_EVT:
        profile.gatts_if = gatts_if;
        esp_ble_gap_set_device_name(DEVICE_NAME);
        esp_ble_gap_config_local_privacy(true);
        esp_ble_gatts_create_attr_tab(gatt_int_env_db, gatts_if, INT_ENV_IDX_NB, SVC_INT_ENVIROMENTAL_SENSING_ID);
        esp_ble_gatts_create_attr_tab(gatt_ext_env_db, gatts_if, EXT_ENV_IDX_NB, SVC_EXT_ENVIROMENTAL_SENSING_ID);
        break;
    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
        if (param->add_attr_tab.svc_uuid.uuid.uuid16 == GATTS_SERVICE_UUID_ENV_SENS)
        {
            if (param->add_attr_tab.num_handle == INT_ENV_IDX_NB)
            {
                memcpy(int_env_sens_handle_table, param->add_attr_tab.handles, sizeof(int_env_sens_handle_table));
                esp_ble_gatts_start_service(int_env_sens_handle_table[INT_ENV_IDX_SVC]);
            }
            else
            {
                memcpy(ext_env_sens_handle_table, param->add_attr_tab.handles, sizeof(ext_env_sens_handle_table));
                esp_ble_gatts_start_service(ext_env_sens_handle_table[EXT_ENV_IDX_SVC]);
            }
        }
        break;
    case ESP_GATTS_CONNECT_EVT:
        profile.conn_id = param->connect.conn_id;
        profile.has_conn = true;
        esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT_MITM);
        break;
    case ESP_GATTS_DISCONNECT_EVT:
        profile.has_conn = false;
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
    case ESP_GAP_BLE_SEC_REQ_EVT:
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
        break;
    case ESP_GAP_BLE_SET_LOCAL_PRIVACY_COMPLETE_EVT:
        esp_ble_gap_config_adv_data(&adv_data);
        adv_config_done |= ADV_CONFIG_FLAG;

        esp_ble_gap_config_adv_data(&scan_rsp_data);
        adv_config_done |= SCAN_RSP_CONFIG_FLAG;
        break;
    default:
        break;
    }
}

void ble_init()
{
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));

    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));

    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_event_handler));
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));

    ESP_ERROR_CHECK(esp_ble_gatts_app_register(0));

    ESP_ERROR_CHECK(esp_ble_gatt_set_local_mtu(500));
    ESP_ERROR_CHECK(esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_N12));

    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_MITM_BOND;
    esp_ble_io_cap_t iocap = ESP_IO_CAP_OUT; //ESP_IO_CAP_NONE;
    uint8_t key_size = 16;
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;

    uint32_t passkey = 123456;
    uint8_t auth_option = ESP_BLE_ONLY_ACCEPT_SPECIFIED_AUTH_ENABLE;
    ESP_ERROR_CHECK(esp_ble_gap_set_security_param(ESP_BLE_SM_SET_STATIC_PASSKEY, &passkey, sizeof(uint32_t)));
    ESP_ERROR_CHECK(esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t)));
    ESP_ERROR_CHECK(esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t)));
    ESP_ERROR_CHECK(esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t)));
    ESP_ERROR_CHECK(esp_ble_gap_set_security_param(ESP_BLE_SM_ONLY_ACCEPT_SPECIFIED_SEC_AUTH, &auth_option, sizeof(uint8_t)));
    ESP_ERROR_CHECK(esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t)));
    ESP_ERROR_CHECK(esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t)));
}

void ble_start_advertising()
{
    esp_ble_gap_start_advertising(&adv_params);
}

void ble_stop_advetising()
{
    esp_ble_gap_stop_advertising();
}

void ble_set_int_humidity_temperature(float humidity, float temperature)
{
    int_hum_val = humidity * 100;
    int_temp_val = temperature * 100;

    esp_ble_gatts_set_attr_value(int_env_sens_handle_table[INT_ENV_IDX_CHAR_VAL_HUM], sizeof(uint16_t), (uint8_t *)&int_hum_val);
    esp_ble_gatts_set_attr_value(int_env_sens_handle_table[INT_ENV_IDX_CHAR_VAL_TEMP], sizeof(int16_t), (uint8_t *)&int_temp_val);
    if (profile.has_conn)
    {
        esp_ble_gatts_send_indicate(profile.gatts_if, profile.conn_id, int_env_sens_handle_table[INT_ENV_IDX_CHAR_VAL_HUM], sizeof(uint16_t), (uint8_t *)&int_hum_val, false);
        esp_ble_gatts_send_indicate(profile.gatts_if, profile.conn_id, int_env_sens_handle_table[INT_ENV_IDX_CHAR_VAL_TEMP], sizeof(int16_t), (uint8_t *)&int_temp_val, false);
    }
}

void ble_set_int_co2_tvoc(uint16_t co2, uint16_t tvoc)
{
    int_co2_val = co2;
    int_tvoc_val = tvoc;

    esp_ble_gatts_set_attr_value(int_env_sens_handle_table[INT_ENV_IDX_CHAR_VAL_CO2], sizeof(uint16_t), (uint8_t *)&int_co2_val);
    esp_ble_gatts_set_attr_value(int_env_sens_handle_table[INT_ENV_IDX_CHAR_VAL_TVOC], sizeof(uint16_t), (uint8_t *)&int_tvoc_val);
    if (profile.has_conn)
    {
        esp_ble_gatts_send_indicate(profile.gatts_if, profile.conn_id, int_env_sens_handle_table[INT_ENV_IDX_CHAR_VAL_CO2], sizeof(uint16_t), (uint8_t *)&int_co2_val, false);
        esp_ble_gatts_send_indicate(profile.gatts_if, profile.conn_id, int_env_sens_handle_table[INT_ENV_IDX_CHAR_VAL_TVOC], sizeof(uint16_t), (uint8_t *)&int_tvoc_val, false);
    }
}

void ble_set_ext_humidity_temperature_pressure(float humidity, float temperature, float pressure)
{
    ext_hum_val = humidity * 100;
    ext_temp_val = temperature * 100;
    ext_press_val = pressure * 100 * 10; /*hPa -> Pa*/

    esp_ble_gatts_set_attr_value(ext_env_sens_handle_table[EXT_ENV_IDX_CHAR_VAL_HUM], sizeof(uint16_t), (uint8_t *)&ext_hum_val);
    esp_ble_gatts_set_attr_value(ext_env_sens_handle_table[EXT_ENV_IDX_CHAR_VAL_TEMP], sizeof(int16_t), (uint8_t *)&ext_temp_val);
    esp_ble_gatts_set_attr_value(ext_env_sens_handle_table[EXT_ENV_IDX_CHAR_VAL_PRESS], sizeof(uint32_t), (uint8_t *)&ext_press_val);
    if (profile.has_conn)
    {
        esp_ble_gatts_send_indicate(profile.gatts_if, profile.conn_id, ext_env_sens_handle_table[EXT_ENV_IDX_CHAR_VAL_HUM], sizeof(uint16_t), (uint8_t *)&ext_press_val, false);
        esp_ble_gatts_send_indicate(profile.gatts_if, profile.conn_id, ext_env_sens_handle_table[EXT_ENV_IDX_CHAR_VAL_TEMP], sizeof(int16_t), (uint8_t *)&ext_press_val, false);
        esp_ble_gatts_send_indicate(profile.gatts_if, profile.conn_id, ext_env_sens_handle_table[EXT_ENV_IDX_CHAR_VAL_PRESS], sizeof(uint32_t), (uint8_t *)&ext_press_val, false);
    }
}

bool ble_has_paired_device()
{
    return esp_ble_get_bond_device_num() > 0;
}

void ble_remove_paired_device()
{
    int dev_num = esp_ble_get_bond_device_num();

    esp_ble_bond_dev_t *dev_list = (esp_ble_bond_dev_t *)malloc(sizeof(esp_ble_bond_dev_t) * dev_num);
    esp_ble_get_bond_device_list(&dev_num, dev_list);
    for (int i = 0; i < dev_num; i++)
    {
        esp_ble_remove_bond_device(dev_list[i].bd_addr);
    }

    free(dev_list);
}