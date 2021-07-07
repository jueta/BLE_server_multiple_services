/*
 This example code is in the Public Domain (or CC0 licensed, at your option.)

 Unless required by applicable law or agreed to in writing, this
 software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 CONDITIONS OF ANY KIND, either express or implied.
 */

/****************************************************************************
 *
 * This demo showcases creating a GATT database using a predefined attribute table.
 * It acts as a GATT server and can send adv data, be connected by client.
 * Run the gatt_client demo, the client demo will automatically connect to the gatt_server_service_table demo.
 * Client demo will enable GATT server's notify after connection. The two devices will then exchange
 * data.
 *
 ****************************************************************************/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "string.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "math.h"
#include "soc/soc_memory_layout.h"  // for esp_ptr_byte_accessible
#include "sdkconfig.h"
#include "esp_gatt_common_api.h"
#include "gatts_table_multi_service.h"


//#include "esp_gap_ble_api.h"
#include "/home/eric/esp/esp-idf/components/bt/host/bluedroid/api/include/api/esp_gap_ble_api.h"
#include "/home/eric/esp/esp-idf/components/bt/host/bluedroid/api/include/api/esp_gatts_api.h"
#include "/home/eric/esp/esp-idf/components/bt/host/bluedroid/api/include/api/esp_bt_main.h"
#include "/home/eric/esp/esp-idf/components/bt/host/bluedroid/api/include/api/esp_bt_device.h"
#include "/home/eric/esp/esp-idf/components/bt/host/bluedroid/api/include/api/esp_gatt_common_api.h"

#define FOLDING 1
//#include "sdkconfig.h"
#define BYTES_PER_LINE 16
#define GATTS_TABLE_TAG "BLE_TABLE"

#define PROFILE_NUM               1
#define PROFILE_APP_IDX           0
#define ESP_APP_ID				  0x55
#define SVC_INST_ID 			  0

#define SAMPLE_DEVICE_NAME          "BLE Multi-Service"

#define GATTS_DEMO_CHAR_VAL_LEN_MAX 500
#define PREPARE_BUF_MAX_SIZE        1024
#define CHAR_DECLARATION_SIZE       (sizeof(uint8_t))

#define ADV_CONFIG_FLAG             (1 << 0)
#define SCAN_RSP_CONFIG_FLAG        (1 << 1)


static uint8_t adv_config_done = 0; // Advertising configuration done value

// tabelas de servicos
uint16_t svc_A_handle_table[SVC_A_IDX_NB];
uint16_t svc_B_handle_table[SVC_B_IDX_NB];

typedef struct {
	uint8_t *prepare_buf;
	int prepare_len;
} prepare_type_env_t;

static prepare_type_env_t prepare_write_env;

static esp_gatt_if_t global_gatts_if = 0xfe;
static uint16_t global_conn_id = 0xfffe;
bool is_connected = false;

static xQueueHandle ble_notify_queue = NULL;
static bool notifyDone = false;
uint8_t notify_data = 0;

static uint8_t manufacturer[3] = { 'S', 'M', 'X' };

static bool ble_svc_A_notify_enable[2] = {false, false};
static char write_var_svc_A[2][50] = 	{"null", "null"};

static bool ble_svc_B_notify_enable[2] = {false, false};
static char write_var_svc_B[2][50] = 	{"null", "null"};




//#define CONFIG_SET_RAW_ADV_DATA
#ifdef CONFIG_SET_RAW_ADV_DATA
static uint8_t raw_adv_data[] = {
	/* flags */
	0x02, 0x01, 0x06,
	/* tx power*/
	0x02, 0x0a, 0xeb,
	/* service uuid */
	0x03, 0x03, 0xFF, 0x00,
	/* device name */
	0x0f, 0x09, 'S', 'p', 'u', 'm', 'p', 'M', 'i', 'x'
};
static uint8_t raw_scan_rsp_data[] = {
	/* flags */
	0x02, 0x01, 0x06,
	/* tx power */
	0x02, 0x0a, 0xeb,
	/* service uuid */
	0x03, 0x03, 0xFF,0x00
};

#else
static uint8_t service_uuid[16] = {
		/* LSB <--------------------------------------------------------------------------------> MSB */
		//first uuid, 16bit, [12],[13] is the value
		0xee, 0xee, 0xee, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00,
		0xFF, 0x00, 0x00, 0x00, };

/* The length of adv data must be less than 31 bytes */
static esp_ble_adv_data_t adv_data =
	{
		.set_scan_rsp = false,
		.include_name = true,
		.include_txpower = false,
		.min_interval = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
		.max_interval = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
		.appearance = 0x00,
		.manufacturer_len =	sizeof(manufacturer),
		.p_manufacturer_data = manufacturer,
		.service_data_len = 0,
		.p_service_data = NULL,
		.service_uuid_len = sizeof(service_uuid),
		.p_service_uuid = service_uuid,
		.flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
	};

// scan response data
static esp_ble_adv_data_t scan_rsp_data =
	{
		.set_scan_rsp = true,
		.include_name = true,
		.include_txpower = false,
		.min_interval = 0x0006,
		.max_interval = 0x0010,
		.appearance = 0x00,
		.manufacturer_len =	sizeof(manufacturer),
		.p_manufacturer_data = manufacturer,
		.service_data_len = 0,
		.p_service_data = NULL,
		.service_uuid_len = sizeof(service_uuid),
		.p_service_uuid = service_uuid,
		.flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
	};
#endif /* CONFIG_SET_RAW_ADV_DATA */

static esp_ble_adv_params_t adv_params =
	{
		.adv_int_min = 0x100,
		.adv_int_max = 0x100,
		.adv_type = ADV_TYPE_IND,
		.own_addr_type = BLE_ADDR_TYPE_PUBLIC,
		.channel_map = ADV_CHNL_ALL,
		.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
	};

struct gatts_profile_inst {
	esp_gatts_cb_t gatts_cb;
	uint16_t gatts_if;
	uint16_t app_id;
	uint16_t conn_id;
	uint16_t service_handle;
	esp_gatt_srvc_id_t service_id;
	uint16_t char_handle;
	esp_bt_uuid_t char_uuid;
	esp_gatt_perm_t perm;
	esp_gatt_char_prop_t property;
	uint16_t descr_handle;
	esp_bt_uuid_t descr_uuid;
};

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst gl_profile_tab[PROFILE_NUM] =
	{
		[PROFILE_APP_IDX] = { .gatts_cb = gatts_profile_event_handler,
				.gatts_if = ESP_GATT_IF_NONE, /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
		},
	};

/* Tipo de Servico */
static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE; // Primary Service
//static const uint16_t secundary_service_uuid = ESP_GATT_UUID_SEC_SERVICE; // Primary Service

/* UUIDs dos atributos */
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE; // Characteristic declaration
static const uint16_t user_desc_uuid = ESP_GATT_UUID_CHAR_DESCRIPTION; // Characteristic Description
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG; // Characteristic client configuration

/* Permissoes das Caracteristicas */
static const uint8_t char_prop_notify = ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_write_notify = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
//static const uint8_t char_prop_read                =  ESP_GATT_CHAR_PROP_BIT_READ;
//static const uint8_t char_prop_write = ESP_GATT_CHAR_PROP_BIT_WRITE;
//static const uint8_t char_prop_read_write_notify   = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
//static const uint8_t char_prop_read_write		   = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ;


/* Servicos */
static const uint16_t GATTS_SERVICE_A_UUID = 0xAA00; // Service Declaration
static const uint16_t GATTS_SERVICE_B_UUID = 0xBB00; // Service Declaration
/* Caracteristicas Servico A */
// Caracteristica A1
static const uint16_t CHAR_1_SVC_A = 0xAA01;
char char_1_svc_a[50] = "null";
static const char desc_char_1_svc_a[] = "Svc A / Caracteristica 1";
static const uint8_t ccc_char_1_svc_a[2] = { 0x00, 0x00 };
// Caracteristica A2
static const uint16_t CHAR_2_SVC_A = 0xAA02;
char char_2_svc_a[50] = "null";
static const char desc_char_2_svc_a[] = "Svc A / Caracteristica 2";
static const uint8_t ccc_char_2_svc_a[2] = { 0x00, 0x00 };

/* Caracteristicas Servico B */
// Caracteristica B1
static const uint16_t CHAR_1_SVC_B = 0xBB01;
char char_1_svc_b[50] = "null";
static const char desc_char_1_svc_b[] = "Svc B / Caracteristica 1";
static const uint8_t ccc_char_1_svc_b[2] = { 0x00, 0x00 };
// Caracteristica B2
static const uint16_t CHAR_2_SVC_B = 0xBB02;
char char_2_svc_b[50] = "null";
static const char desc_char_2_svc_b[] = "Svc B / Caracteristica 2";
static const uint8_t ccc_char_2_svc_b[2] = { 0x00, 0x00 };

/* Debugging via UART */
#define TXD_PIN_UART0 (GPIO_NUM_1)
#define RXD_PIN_UART0 (GPIO_NUM_3)
static const int RX_BUF_SIZE = 1024;

/* Full Database Description - Used to add attributes into the database */

static const esp_gatts_attr_db_t gatt_db_service_A[SVC_A_IDX_NB] = {

		// * * * * * * * * * * * * * * * * //
		// * * * * *  SERVICE A  * * * * * //
		// * * * * * * * * * * * * * * * * //
		[IDX_SVC_A] =
			{
				{ ESP_GATT_AUTO_RSP },
				{ ESP_UUID_LEN_16,
				(uint8_t *) &primary_service_uuid, // Servico primario
				ESP_GATT_PERM_READ_ENC_MITM,
				sizeof(uint16_t),
				sizeof(GATTS_SERVICE_A_UUID),
				(uint8_t *) &GATTS_SERVICE_A_UUID} // UUID do Servico A
			},

		// * * * * * * * * * * * * * * * //
		// * *   CHAR 1 - SVC A  * * * * //
		// * * * * * * * * * * * * * * * //

		/* Characteristic Declaration */
		[IDX_CHAR_SVC_A_CHAR1] =
			{
				{ ESP_GATT_AUTO_RSP },
				{ ESP_UUID_LEN_16,
				(uint8_t *) &character_declaration_uuid,
				ESP_GATT_PERM_READ_ENC_MITM,
				CHAR_DECLARATION_SIZE,
				CHAR_DECLARATION_SIZE,
				(uint8_t *) &char_prop_notify } // Permissao para Notify
			}, // R/W

		/* Characteristic Value */
		[IDX_CHAR_VAL_SVC_A_CHAR1] =
			{
				{ ESP_GATT_AUTO_RSP },
				{ ESP_UUID_LEN_16,
				(uint8_t *) &CHAR_1_SVC_A,
				ESP_GATT_PERM_READ_ENC_MITM | ESP_GATT_PERM_WRITE_ENC_MITM,
				GATTS_DEMO_CHAR_VAL_LEN_MAX,
				sizeof(char_1_svc_a),
				(uint8_t *) &char_1_svc_a }
			},

		/* Characteristic User Description */
		[IDX_CHAR_DESC_SVC_A_CHAR1] =
			{
				{ ESP_GATT_AUTO_RSP },
				{ ESP_UUID_LEN_16,
				(uint8_t *) &user_desc_uuid,
				ESP_GATT_PERM_READ_ENC_MITM,
				GATTS_DEMO_CHAR_VAL_LEN_MAX,
				sizeof(desc_char_1_svc_a),
				(uint8_t *) desc_char_1_svc_a }
			},

		/* Client Characteristic Configuration Descriptor */
		[IDX_CHAR_CFG_SVC_A_CHAR1] =
			{
				{ ESP_GATT_AUTO_RSP },
				{ ESP_UUID_LEN_16,
				(uint8_t *) &character_client_config_uuid,
				ESP_GATT_PERM_READ_ENC_MITM | ESP_GATT_PERM_WRITE_ENC_MITM,
				sizeof(uint16_t),
				sizeof(ccc_char_1_svc_a),
				(uint8_t *) ccc_char_1_svc_a }
			},


		// * * * * * * * * * * * * * * * //
		// * *   CHAR 2 - SVC A  * * * * //
		// * * * * * * * * * * * * * * * //

		/* Characteristic Declaration */
		[IDX_CHAR_SVC_A_CHAR2] =
			{
				{ ESP_GATT_AUTO_RSP },
				{ ESP_UUID_LEN_16,
				(uint8_t *) &character_declaration_uuid,
				ESP_GATT_PERM_READ_ENC_MITM,
				CHAR_DECLARATION_SIZE,
				CHAR_DECLARATION_SIZE,
				(uint8_t *) &char_prop_write_notify } // Permissao para Notify
			}, // R/W

		/* Characteristic Value */
		[IDX_CHAR_VAL_SVC_A_CHAR2] =
			{
				{ ESP_GATT_AUTO_RSP },
				{ ESP_UUID_LEN_16,
				(uint8_t *) &CHAR_2_SVC_A,
				ESP_GATT_PERM_READ_ENC_MITM | ESP_GATT_PERM_WRITE_ENC_MITM,
				GATTS_DEMO_CHAR_VAL_LEN_MAX,
				sizeof(char_2_svc_a),
				(uint8_t *) &char_2_svc_a }
			},

		/* Characteristic User Description */
		[IDX_CHAR_DESC_SVC_A_CHAR2] =
			{
				{ ESP_GATT_AUTO_RSP },
				{ ESP_UUID_LEN_16,
				(uint8_t *) &user_desc_uuid,
				ESP_GATT_PERM_READ_ENC_MITM,
				GATTS_DEMO_CHAR_VAL_LEN_MAX,
				sizeof(desc_char_2_svc_a),
				(uint8_t *) desc_char_2_svc_a }
			},

		/* Client Characteristic Configuration Descriptor */
		[IDX_CHAR_CFG_SVC_A_CHAR2] =
			{
				{ ESP_GATT_AUTO_RSP },
				{ ESP_UUID_LEN_16,
				(uint8_t *) &character_client_config_uuid,
				ESP_GATT_PERM_READ_ENC_MITM | ESP_GATT_PERM_WRITE_ENC_MITM,
				sizeof(uint16_t),
				sizeof(ccc_char_2_svc_a),
				(uint8_t *) ccc_char_2_svc_a }
			},
};

static const esp_gatts_attr_db_t gatt_db_service_B[SVC_B_IDX_NB] = {

		// * * * * * * * * * * * * * * * * //
		// * * * * *  SERVICE B  * * * * * //
		// * * * * * * * * * * * * * * * * //
		[IDX_SVC_B] =
			{
				{ ESP_GATT_AUTO_RSP },
				{ ESP_UUID_LEN_16,
				(uint8_t *) &primary_service_uuid, // Servico primario
				ESP_GATT_PERM_READ_ENC_MITM, sizeof(uint16_t),
				sizeof(GATTS_SERVICE_B_UUID),
				(uint8_t *) &GATTS_SERVICE_B_UUID} // UUID do Servico B
			},

		// * * * * * * * * * * * * * * * //
		// * *   CHAR 1 - SVC B  * * * * //
		// * * * * * * * * * * * * * * * //

		/* Characteristic Declaration */
		[IDX_CHAR_SVC_B_CHAR1] =
			{
				{ ESP_GATT_AUTO_RSP },
				{ ESP_UUID_LEN_16,
				(uint8_t *) &character_declaration_uuid,
				ESP_GATT_PERM_READ_ENC_MITM,
				CHAR_DECLARATION_SIZE,
				CHAR_DECLARATION_SIZE,
				(uint8_t *) &char_prop_notify } // Permissao para Notify
			}, // R/W

		/* Characteristic Value */
		[IDX_CHAR_VAL_SVC_B_CHAR1] =
			{
				{ ESP_GATT_AUTO_RSP },
				{ ESP_UUID_LEN_16,
				(uint8_t *) &CHAR_1_SVC_B,
				ESP_GATT_PERM_READ_ENC_MITM | ESP_GATT_PERM_WRITE_ENC_MITM,
				GATTS_DEMO_CHAR_VAL_LEN_MAX,
				sizeof(char_1_svc_b),
				(uint8_t *) &char_1_svc_b }
			},

		/* Characteristic User Description */
		[IDX_CHAR_DESC_SVC_B_CHAR1] =
			{
				{ ESP_GATT_AUTO_RSP },
				{ ESP_UUID_LEN_16,
				(uint8_t *) &user_desc_uuid,
				ESP_GATT_PERM_READ_ENC_MITM,
				GATTS_DEMO_CHAR_VAL_LEN_MAX,
				sizeof(desc_char_1_svc_b),
				(uint8_t *) desc_char_1_svc_b }
			},

		/* Client Characteristic Configuration Descriptor */
		[IDX_CHAR_CFG_SVC_B_CHAR1] =
			{
				{ ESP_GATT_AUTO_RSP },
				{ ESP_UUID_LEN_16,
				(uint8_t *) &character_client_config_uuid,
				ESP_GATT_PERM_READ_ENC_MITM | ESP_GATT_PERM_WRITE_ENC_MITM,
				sizeof(uint16_t),
				sizeof(ccc_char_1_svc_b),
				(uint8_t *) ccc_char_1_svc_b }
			},


		// * * * * * * * * * * * * * * * //
		// * *   CHAR 2 - SVC B  * * * * //
		// * * * * * * * * * * * * * * * //

		/* Characteristic Declaration */
		[IDX_CHAR_SVC_B_CHAR2] =
			{
				{ ESP_GATT_AUTO_RSP },
				{ ESP_UUID_LEN_16,
				(uint8_t *) &character_declaration_uuid,
				ESP_GATT_PERM_READ_ENC_MITM,
				CHAR_DECLARATION_SIZE,
				CHAR_DECLARATION_SIZE,
				(uint8_t *) &char_prop_write_notify } // Permissao para Notify
			}, // R/W

		/* Characteristic Value */
		[IDX_CHAR_VAL_SVC_B_CHAR2] =
			{
				{ ESP_GATT_AUTO_RSP },
				{ ESP_UUID_LEN_16,
				(uint8_t *) &CHAR_2_SVC_B,
				ESP_GATT_PERM_READ_ENC_MITM | ESP_GATT_PERM_WRITE_ENC_MITM,
				GATTS_DEMO_CHAR_VAL_LEN_MAX,
				sizeof(char_2_svc_b),
				(uint8_t *) &char_2_svc_b }
			},

		/* Characteristic User Description */
		[IDX_CHAR_DESC_SVC_B_CHAR2] =
			{
				{ ESP_GATT_AUTO_RSP },
				{ ESP_UUID_LEN_16,
				(uint8_t *) &user_desc_uuid,
				ESP_GATT_PERM_READ_ENC_MITM,
				GATTS_DEMO_CHAR_VAL_LEN_MAX,
				sizeof(desc_char_2_svc_b),
				(uint8_t *) desc_char_2_svc_b }
			},

		/* Client Characteristic Configuration Descriptor */
		[IDX_CHAR_CFG_SVC_B_CHAR2] =
			{
				{ ESP_GATT_AUTO_RSP },
				{ ESP_UUID_LEN_16,
				(uint8_t *) &character_client_config_uuid,
				ESP_GATT_PERM_READ_ENC_MITM | ESP_GATT_PERM_WRITE_ENC_MITM,
				sizeof(uint16_t),
				sizeof(ccc_char_2_svc_b),
				(uint8_t *) ccc_char_2_svc_b }
			},
};


static char *esp_key_type_to_str(esp_ble_key_type_t key_type) {
	char *key_str = NULL;
	switch (key_type) {
	case ESP_LE_KEY_NONE:
		key_str = "ESP_LE_KEY_NONE";
		break;
	case ESP_LE_KEY_PENC:
		key_str = "ESP_LE_KEY_PENC";
		break;
	case ESP_LE_KEY_PID:
		key_str = "ESP_LE_KEY_PID";
		break;
	case ESP_LE_KEY_PCSRK:
		key_str = "ESP_LE_KEY_PCSRK";
		break;
	case ESP_LE_KEY_PLK:
		key_str = "ESP_LE_KEY_PLK";
		break;
	case ESP_LE_KEY_LLK:
		key_str = "ESP_LE_KEY_LLK";
		break;
	case ESP_LE_KEY_LENC:
		key_str = "ESP_LE_KEY_LENC";
		break;
	case ESP_LE_KEY_LID:
		key_str = "ESP_LE_KEY_LID";
		break;
	case ESP_LE_KEY_LCSRK:
		key_str = "ESP_LE_KEY_LCSRK";
		break;
	default:
		key_str = "INVALID BLE KEY TYPE";
		break;

	}

	return key_str;
}

static char *esp_auth_req_to_str(esp_ble_auth_req_t auth_req) {
	char *auth_str = NULL;
	switch (auth_req) {
	case ESP_LE_AUTH_NO_BOND:
		auth_str = "ESP_LE_AUTH_NO_BOND";
		break;
	case ESP_LE_AUTH_BOND:
		auth_str = "ESP_LE_AUTH_BOND";
		break;
	case ESP_LE_AUTH_REQ_MITM:
		auth_str = "ESP_LE_AUTH_REQ_MITM";
		break;
	case ESP_LE_AUTH_REQ_BOND_MITM:
		auth_str = "ESP_LE_AUTH_REQ_BOND_MITM";
		break;
	case ESP_LE_AUTH_REQ_SC_ONLY:
		auth_str = "ESP_LE_AUTH_REQ_SC_ONLY";
		break;
	case ESP_LE_AUTH_REQ_SC_BOND:
		auth_str = "ESP_LE_AUTH_REQ_SC_BOND";
		break;
	case ESP_LE_AUTH_REQ_SC_MITM:
		auth_str = "ESP_LE_AUTH_REQ_SC_MITM";
		break;
	case ESP_LE_AUTH_REQ_SC_MITM_BOND:
		auth_str = "ESP_LE_AUTH_REQ_SC_MITM_BOND";
		break;
	default:
		auth_str = "INVALID BLE AUTH REQ";
		break;
	}

	return auth_str;
}

static void show_bonded_devices(void) {
	int dev_num = esp_ble_get_bond_device_num();

	esp_ble_bond_dev_t *dev_list = (esp_ble_bond_dev_t *) malloc(sizeof(esp_ble_bond_dev_t) * dev_num);
	esp_ble_get_bond_device_list(&dev_num, dev_list);

	ESP_LOGI(GATTS_TABLE_TAG, "Bonded devices number : %d\n", dev_num);
	ESP_LOGI(GATTS_TABLE_TAG, "Bonded devices list : %d\n", dev_num);

	for (int i = 0; i < dev_num; i++) {
		esp_log_buffer_hex(GATTS_TABLE_TAG, (void *)dev_list[i].bd_addr, sizeof(esp_bd_addr_t));
	}

	free(dev_list);
}

void __attribute__((unused)) remove_all_bonded_devices(void) {
	int dev_num = esp_ble_get_bond_device_num();

	esp_ble_bond_dev_t *dev_list = (esp_ble_bond_dev_t *) malloc(sizeof(esp_ble_bond_dev_t) * dev_num);
	esp_ble_get_bond_device_list(&dev_num, dev_list);
	for (int i = 0; i < dev_num; i++) {
		esp_ble_remove_bond_device(dev_list[i].bd_addr);
	}

	free(dev_list);
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
	switch (event) {
#ifdef CONFIG_SET_RAW_ADV_DATA
	case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
		adv_config_done &= (~ADV_CONFIG_FLAG);
		if (adv_config_done == 0) {
			esp_ble_gap_start_advertising(&adv_params);
		}
		break;
	case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
		adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
		if (adv_config_done == 0) {
			esp_ble_gap_start_advertising(&adv_params);
		}
		break;
#else
	case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
		adv_config_done &= (~ADV_CONFIG_FLAG);
		if (adv_config_done == 0) {
			esp_ble_gap_start_advertising(&adv_params);
		}
		break;
	case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
		adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
		if (adv_config_done == 0) {
			esp_ble_gap_start_advertising(&adv_params);
		}
		break;
#endif
	case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
		/* advertising start complete event to indicate advertising start successfully or failed */
		if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
			ESP_LOGE(GATTS_TABLE_TAG, "advertising start failed");
		} else {
			ESP_LOGI(GATTS_TABLE_TAG, "advertising start successfully");
		}
		break;
	case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
		if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
			ESP_LOGE(GATTS_TABLE_TAG, "Advertising stop failed");
		} else {
			ESP_LOGI(GATTS_TABLE_TAG, "Stop adv successfully\n");
		}
		break;
	case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
		ESP_LOGI(GATTS_TABLE_TAG,
				"update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
				param->update_conn_params.status,
				param->update_conn_params.min_int,
				param->update_conn_params.max_int,
				param->update_conn_params.conn_int,
				param->update_conn_params.latency,
				param->update_conn_params.timeout);
		break;

	case ESP_GAP_BLE_PASSKEY_REQ_EVT: /* passkey request event */
		ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_PASSKEY_REQ_EVT");
		/* Call the following function to input the passkey which is displayed on the remote device */
		//esp_ble_passkey_reply(gl_profile_tab[HEART_PROFILE_APP_IDX].remote_bda, true, 0x00);
		break;
	case ESP_GAP_BLE_OOB_REQ_EVT: {
		ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_OOB_REQ_EVT");
		uint8_t tk[16] = { 1 }; //If you paired with OOB, both devices need to use the same tk
		esp_ble_oob_req_reply(param->ble_security.ble_req.bd_addr, tk, sizeof(tk));
		break;
	}
	case ESP_GAP_BLE_LOCAL_IR_EVT: /* BLE local IR event */
		ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_LOCAL_IR_EVT");
		break;
	case ESP_GAP_BLE_LOCAL_ER_EVT: /* BLE local ER event */
		ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_LOCAL_ER_EVT");
		break;
	case ESP_GAP_BLE_NC_REQ_EVT:
		/* The app will receive this evt when the IO has DisplayYesNO capability and the peer device IO also has DisplayYesNo capability.
		 show the passkey number to the user to confirm it with the number displayed by peer device. */
		esp_ble_confirm_reply(param->ble_security.ble_req.bd_addr, true);
		ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_NC_REQ_EVT, the passkey Notify number: %d", param->ble_security.key_notif.passkey);
		break;
	case ESP_GAP_BLE_SEC_REQ_EVT:
		/* send the positive(true) security response to the peer device to accept the security request.
		 If not accept the security request, should send the security response with negative(false) accept value*/
		esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
		break;
	case ESP_GAP_BLE_PASSKEY_NOTIF_EVT: ///the app will receive this evt when the IO  has Output capability and the peer device IO has Input capability.
		///show the passkey number to the user to input it in the peer device.
		ESP_LOGI(GATTS_TABLE_TAG, "The passkey Notify number:%d", param->ble_security.key_notif.passkey);
		break;
	case ESP_GAP_BLE_KEY_EVT:
		//shows the ble key info share with peer device to the user.
		ESP_LOGI(GATTS_TABLE_TAG, "key type = %s", esp_key_type_to_str(param->ble_security.ble_key.key_type));
		break;
	case ESP_GAP_BLE_AUTH_CMPL_EVT: {
		esp_bd_addr_t bd_addr;
		memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr, sizeof(esp_bd_addr_t));
		ESP_LOGI(GATTS_TABLE_TAG, "remote BD_ADDR: %08x%04x", (bd_addr[0] << 24) + (bd_addr[1] << 16) + (bd_addr[2] << 8) + bd_addr[3], (bd_addr[4] << 8) + bd_addr[5]);
		ESP_LOGI(GATTS_TABLE_TAG, "address type = %d", param->ble_security.auth_cmpl.addr_type);
		ESP_LOGI(GATTS_TABLE_TAG, "pair status = %s", param->ble_security.auth_cmpl.success ? "success" : "fail");
		if (!param->ble_security.auth_cmpl.success) {
			ESP_LOGI(GATTS_TABLE_TAG, "fail reason = 0x%x", param->ble_security.auth_cmpl.fail_reason);
		} else {
			ESP_LOGI(GATTS_TABLE_TAG, "auth mode = %s", esp_auth_req_to_str(param->ble_security.auth_cmpl.auth_mode));
			esp_ble_gap_update_whitelist(true, bd_addr, BLE_WL_ADDR_TYPE_PUBLIC);
		}
		show_bonded_devices();
		break;
	}
	case ESP_GAP_BLE_REMOVE_BOND_DEV_COMPLETE_EVT: {
		ESP_LOGD(GATTS_TABLE_TAG, "ESP_GAP_BLE_REMOVE_BOND_DEV_COMPLETE_EVT status = %d", param->remove_bond_dev_cmpl.status);
		ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_REMOVE_BOND_DEV");
		ESP_LOGI(GATTS_TABLE_TAG, "-----ESP_GAP_BLE_REMOVE_BOND_DEV----");
		esp_log_buffer_hex(GATTS_TABLE_TAG, (void *)param->remove_bond_dev_cmpl.bd_addr, sizeof(esp_bd_addr_t));
		ESP_LOGI(GATTS_TABLE_TAG, "------------------------------------");
		esp_ble_gap_update_whitelist(false, (void *) param->remove_bond_dev_cmpl.bd_addr, BLE_WL_ADDR_TYPE_PUBLIC);
		break;
	}
	case ESP_GAP_BLE_SET_LOCAL_PRIVACY_COMPLETE_EVT:
		if (param->local_privacy_cmpl.status != ESP_BT_STATUS_SUCCESS) {
			ESP_LOGE(GATTS_TABLE_TAG, "config local privacy failed, error status = %x", param->local_privacy_cmpl.status);
			break;
		}

		esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
		if (ret) {
			ESP_LOGE(GATTS_TABLE_TAG, "config adv data failed, error code = %x", ret);
		} else {
			adv_config_done |= ADV_CONFIG_FLAG;
		}

		ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
		if (ret) {
			ESP_LOGE(GATTS_TABLE_TAG, "config adv data failed, error code = %x", ret);
		} else {
			adv_config_done |= SCAN_RSP_CONFIG_FLAG;
		}
		break;
	default:
		break;
	}
}

void example_prepare_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param) {
	ESP_LOGI(GATTS_TABLE_TAG, "prepare write, handle = %d, value len = %d", param->write.handle, param->write.len);
	esp_gatt_status_t status = ESP_GATT_OK;
	if (prepare_write_env->prepare_buf == NULL) {
		prepare_write_env->prepare_buf = (uint8_t *) malloc(PREPARE_BUF_MAX_SIZE * sizeof(uint8_t));
		prepare_write_env->prepare_len = 0;
		if (prepare_write_env->prepare_buf == NULL) {
			ESP_LOGE(GATTS_TABLE_TAG, "%s, Gatt_server prep no mem", __func__);
			status = ESP_GATT_NO_RESOURCES;
		}
	} else {
		if (param->write.offset > PREPARE_BUF_MAX_SIZE) {
			status = ESP_GATT_INVALID_OFFSET;
		} else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE) {
			status = ESP_GATT_INVALID_ATTR_LEN;
		}
	}
	/*send response when param->write.need_rsp is true */
	if (param->write.need_rsp) {
		esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *) malloc(sizeof(esp_gatt_rsp_t));
		if (gatt_rsp != NULL) {
			gatt_rsp->attr_value.len = param->write.len;
			gatt_rsp->attr_value.handle = param->write.handle;
			gatt_rsp->attr_value.offset = param->write.offset;
			gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
			memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
			esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
			if (response_err != ESP_OK) {
				ESP_LOGE(GATTS_TABLE_TAG, "Send response error");
			}
			free(gatt_rsp);
		} else {
			ESP_LOGE(GATTS_TABLE_TAG, "%s, malloc failed", __func__);
		}
	}
	if (status != ESP_GATT_OK) {
		return;
	}

	memcpy(prepare_write_env->prepare_buf + param->write.offset, param->write.value, param->write.len);
	prepare_write_env->prepare_len += param->write.len;
}



static void write_variable(esp_ble_gatts_cb_param_t *param) {
	if (param->write.handle >= svc_A_handle_table[IDX_SVC_A] && param->write.handle <= svc_A_handle_table[SVC_A_IDX_NB-1]){
		for (int i = 1; i < (((SVC_A_IDX_NB - 1) / 4) + 1); i++) {
			uint8_t idx = (i * 4) - 2;
			if (svc_A_handle_table[idx] == param->write.handle) {
				memset(write_var_svc_A[i - 1], 0, strlen(write_var_svc_A[i - 1]));
				ESP_LOGE("Write on Service [A]", "Char Idx: %d; Char handle: %d; val: %s", idx, param->write.handle, param->write.value);
				getBuffer(write_var_svc_A[i - 1], param->write.value, param->write.len);
			}
		}
	}
	else if (param->write.handle >= svc_B_handle_table[IDX_SVC_B] && param->write.handle <= svc_B_handle_table[SVC_B_IDX_NB-1]) {
		for (int i = 1; i < (((SVC_B_IDX_NB - 1) / 4) + 1); i++) {
			uint8_t idx = (i * 4) - 2;
			if (svc_B_handle_table[idx] == param->write.handle) {
				memset(write_var_svc_B[i - 1], 0, strlen(write_var_svc_B[i - 1]));
				ESP_LOGE("Write on Service [B]", "Char Idx: %d; Char handle: %d; val: %s", idx, param->write.handle, param->write.value);
				getBuffer(write_var_svc_A[i - 1], param->write.value, param->write.len);
			}
		}

	}
}


static void notify_config(esp_ble_gatts_cb_param_t *param) {
	if (param->write.handle >= svc_A_handle_table[IDX_SVC_A] && param->write.handle <= svc_A_handle_table[SVC_A_IDX_NB-1]){
		for (int i = 1; i < ((((SVC_A_IDX_NB - 1) / 4) + 1)); i++) {
			uint8_t idx = i * 4;
			if ((svc_A_handle_table[idx] == param->write.handle) && param->write.len == 2) {
				uint16_t descr_value = param->write.value[1] << 8 | param->write.value[0];
				if (descr_value == 0x0001) {
					/* Enable Notify */
					ble_svc_A_notify_enable[i - 1] = true;
					ESP_LOGE("TESTE-NOTIFY [A]", "idx: %u, handle: %u", idx, param->write.handle);
				} else if (descr_value == 0x0002) {
					/* Enable Indicate */
				} else if (descr_value == 0x0000) {
					/* Disable Indicate */
					ble_svc_A_notify_enable[i - 1] = false;
					ESP_LOGE("TESTE-NOTIFY-DIS [A]", "idx: %u, handle: %u", idx, param->write.handle);
				} else {
					/* Unknown */
					esp_log_buffer_hex(GATTS_TABLE_TAG, param->write.value, param->write.len);
				}
			}
		}
	}
	else if (param->write.handle >= svc_B_handle_table[IDX_SVC_B] && param->write.handle <= svc_B_handle_table[SVC_B_IDX_NB-1]) {
		for (int i = 1; i < ((((SVC_B_IDX_NB - 1) / 4) + 1)); i++) {
			uint8_t idx = i * 4;
			if ((svc_B_handle_table[idx] == param->write.handle) && param->write.len == 2) {
				uint16_t descr_value = param->write.value[1] << 8 | param->write.value[0];
				if (descr_value == 0x0001) {
					/* Enable Notify */
					ble_svc_B_notify_enable[i - 1] = true;
					ESP_LOGE("TESTE-NOTIFY [B]", "idx: %u, handle: %u", idx, param->write.handle);
				} else if (descr_value == 0x0002) {
					/* Enable Indicate */
				} else if (descr_value == 0x0000) {
					/* Disable Indicate */
					ble_svc_B_notify_enable[i - 1] = false;
					ESP_LOGE("TESTE-NOTIFY-DIS [B]", "idx: %u, handle: %u", idx, param->write.handle);
				} else {
					/* Unknown */
					esp_log_buffer_hex(GATTS_TABLE_TAG, param->write.value, param->write.len);
				}
			}
		}
	}
}


void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param) {
	if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC && prepare_write_env->prepare_buf) {
		esp_log_buffer_hex(GATTS_TABLE_TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
	} else {
		ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATT_PREP_WRITE_CANCEL");
	}
	if (prepare_write_env->prepare_buf) {
		free(prepare_write_env->prepare_buf);
		prepare_write_env->prepare_buf = NULL;
	}
	prepare_write_env->prepare_len = 0;
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
	switch (event) {
	case ESP_GATTS_REG_EVT: {
        ESP_LOGI(GATTS_TABLE_TAG, "REGISTER_APP_EVT, status %d, app_id %d\n", param->reg.status, param->reg.app_id);

		esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(SAMPLE_DEVICE_NAME);
		if (set_dev_name_ret) {
			ESP_LOGE(GATTS_TABLE_TAG, "set device name failed, error code = %x", set_dev_name_ret);
		}
#ifdef CONFIG_SET_RAW_ADV_DATA

		esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));
		if (raw_adv_ret) {
			ESP_LOGE(GATTS_TABLE_TAG, "config raw adv data failed, error code = %x ", raw_adv_ret);
		}
		adv_config_done |= ADV_CONFIG_FLAG;

		esp_err_t raw_scan_ret = esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, sizeof(raw_scan_rsp_data));
		if (raw_scan_ret) {
			ESP_LOGE(GATTS_TABLE_TAG, "config raw scan rsp data failed, error code = %x", raw_scan_ret);
		}
		adv_config_done |= SCAN_RSP_CONFIG_FLAG;

#else
		//config adv data
		esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
		if (ret){
			ESP_LOGE(GATTS_TABLE_TAG, "config adv data failed, error code = %x", ret);
		}
		adv_config_done |= ADV_CONFIG_FLAG;

		//config scan response data
		ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
		if (ret){
			ESP_LOGE(GATTS_TABLE_TAG, "config scan response data failed, error code = %x", ret);
		}
		adv_config_done |= SCAN_RSP_CONFIG_FLAG;

		//generate a resolvable random address
		esp_ble_gap_config_local_privacy(true);
#endif

		/* Criacao das tabelas de atributos dos servicos */
		esp_err_t create_attr_ret = esp_ble_gatts_create_attr_tab(gatt_db_service_A, gatts_if, SVC_A_IDX_NB, SVC_INST_ID);
		if (create_attr_ret) {
			ESP_LOGE(GATTS_TABLE_TAG, "[A] create attr table failed, error code = %x", create_attr_ret);
		}

		create_attr_ret = esp_ble_gatts_create_attr_tab(gatt_db_service_B, gatts_if, SVC_B_IDX_NB, SVC_INST_ID);
		if (create_attr_ret) {
			ESP_LOGE(GATTS_TABLE_TAG, "[B] create attr table failed, error code = %x", create_attr_ret);
		}

	}
		break;
	case ESP_GATTS_READ_EVT:
		ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_READ_EVT");
		break;
	case ESP_GATTS_WRITE_EVT:
		if (!param->write.is_prep) {
			// the data length of gattc write  must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
			ESP_LOGI(GATTS_TABLE_TAG, "GATT_WRITE_EVT, handle = %d, value len = %d, value : %s", param->write.handle, param->write.len, param->write.value);
			esp_log_buffer_hex(GATTS_TABLE_TAG, param->write.value, param->write.len);

			/* Register Variable Writes from App */
			write_variable(param);

			/* Configure notifications */
			notify_config(param);

			/* send response when param->write.need_rsp is true*/
			if (param->write.need_rsp) {
				esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
			}
		} else {
			/* handle prepare write */
			example_prepare_write_event_env(gatts_if, &prepare_write_env, param);
		}
		break;
	case ESP_GATTS_EXEC_WRITE_EVT:
		ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_EXEC_WRITE_EVT");
		example_exec_write_event_env(&prepare_write_env, param);
		break;
	case ESP_GATTS_MTU_EVT:
		ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
		break;
	case ESP_GATTS_CONF_EVT:
//		ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONF_EVT, status = %d, attr_handle %d", param->conf.status, param->conf.handle);
		break;
	case ESP_GATTS_START_EVT:
		ESP_LOGI(GATTS_TABLE_TAG, "SERVICE_START_EVT, status %d, service_handle %d", param->start.status, param->start.service_handle);
		break;
	case ESP_GATTS_CONNECT_EVT:
		ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONNECT_EVT, conn_id = %d", param->connect.conn_id);
		esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT_MITM);
		uint16_t cmd = 0;
		global_conn_id = param->connect.conn_id;
		global_gatts_if = gatts_if;
		is_connected = true;
		xQueueSend(ble_notify_queue, &cmd, 10/portTICK_PERIOD_MS);

		break;
	case ESP_GATTS_DISCONNECT_EVT:
		ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_DISCONNECT_EVT, reason = 0x%x", param->disconnect.reason);
		is_connected = false;
		esp_ble_gap_start_advertising(&adv_params);
		break;
	case ESP_GATTS_CREAT_ATTR_TAB_EVT: {
		if (param->add_attr_tab.status != ESP_GATT_OK) {
			ESP_LOGE(GATTS_TABLE_TAG, "create attribute table failed, error code=0x%x", param->add_attr_tab.status);
		} else {
			switch(param->add_attr_tab.svc_uuid.uuid.uuid16){
				case(0xAA00):{
					if (param->add_attr_tab.num_handle != SVC_A_IDX_NB){
						ESP_LOGE(GATTS_TABLE_TAG, "[A] create attribute table abnormally, num_handle (%d) - doesnt equal to SVC_A_IDX_NB(%d)", param->add_attr_tab.num_handle, SVC_A_IDX_NB);
					}
					else {
						ESP_LOGI(GATTS_TABLE_TAG, "[A] create attribute table successfully, the number handle = %d\n",param->add_attr_tab.num_handle);
						memcpy(svc_A_handle_table, param->add_attr_tab.handles, sizeof(svc_A_handle_table));
						esp_ble_gatts_start_service(svc_A_handle_table[IDX_SVC_A]);
					}
					break;
				}
				case(0xBB00):{
					if (param->add_attr_tab.num_handle != SVC_B_IDX_NB){
						ESP_LOGE(GATTS_TABLE_TAG, "[B] create attribute table abnormally, num_handle (%d) - doesnt equal to SVC_A_IDX_NB(%d)", param->add_attr_tab.num_handle, SVC_A_IDX_NB);
					}
					else {
						ESP_LOGI(GATTS_TABLE_TAG, "[B] create attribute table successfully, the number handle = %d\n",param->add_attr_tab.num_handle);
						memcpy(svc_B_handle_table, param->add_attr_tab.handles, sizeof(svc_B_handle_table));
						esp_ble_gatts_start_service(svc_B_handle_table[IDX_SVC_B]);
					}
					break;
				}
				default:{
        			ESP_LOGE(GATTS_TABLE_TAG, "attribute table does not match any known cases, the number handle = %d\n",param->add_attr_tab.num_handle);
        			break;
				}
			}
		}
		break;
	}
	case ESP_GATTS_STOP_EVT:
	case ESP_GATTS_OPEN_EVT:
	case ESP_GATTS_CANCEL_OPEN_EVT:
	case ESP_GATTS_CLOSE_EVT:
	case ESP_GATTS_LISTEN_EVT:
	case ESP_GATTS_CONGEST_EVT:
	case ESP_GATTS_UNREG_EVT:
	case ESP_GATTS_DELETE_EVT:
	default:
		break;
	}
}


static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{

	/* If event is register event, store the gatts_if for each profile */
	if (event == ESP_GATTS_REG_EVT) {
		if (param->reg.status == ESP_GATT_OK) {
			gl_profile_tab[PROFILE_APP_IDX].gatts_if = gatts_if;
		} else {
			ESP_LOGE(GATTS_TABLE_TAG, "reg app failed, app_id %04x, status %d", param->reg.app_id, param->reg.status);
			return;
		}
	}
	do {
		int idx;
		for (idx = 0; idx < PROFILE_NUM; idx++) {
			/* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
			if (gatts_if == ESP_GATT_IF_NONE
					|| gatts_if == gl_profile_tab[idx].gatts_if) {
				if (gl_profile_tab[idx].gatts_cb) {
					gl_profile_tab[idx].gatts_cb(event, gatts_if, param);
				}
			}
		}
	} while (0);
}

void ble_notify_task(void * arg) {
 	uint16_t cmd_id;
 	static char aux_var_svc_A[2][50] = {"null", "null"};
 	static char aux_var_svc_B[2][50] = {"null", "null"};
	while(1) {
		vTaskDelay(500 / portTICK_PERIOD_MS);
		if (xQueueReceive(ble_notify_queue, &cmd_id, portMAX_DELAY)) {
			while (1) {
				notify_data++;
				vTaskDelay(2000 / portTICK_PERIOD_MS);
				if(is_connected){
					/* Notify Service A */
					sprintf(char_1_svc_a, "%d", notify_data);

					if(strcmp(char_1_svc_a, aux_var_svc_A[0]) != 0){
						if (ble_svc_A_notify_enable[0]) {
							esp_ble_gatts_send_indicate(global_gatts_if,
														global_conn_id,
														svc_A_handle_table[IDX_CHAR_VAL_SVC_A_CHAR1],
														strlen(char_1_svc_a),
														(uint8_t *)char_1_svc_a,
														false);
							ESP_LOGW("NOTIFY", "char_1_svc_a - %s", char_1_svc_a);
							strcpy(aux_var_svc_A[0], char_1_svc_a);
						}
					}
					if(strcmp(char_2_svc_a, aux_var_svc_A[1]) != 0){
						if (ble_svc_A_notify_enable[1]) {
							esp_ble_gatts_send_indicate(global_gatts_if,
														global_conn_id,
														svc_A_handle_table[IDX_CHAR_VAL_SVC_A_CHAR2],
														strlen(char_2_svc_a),
														(uint8_t *)char_2_svc_a,
														false);
							ESP_LOGW("NOTIFY", "char_2_svc_a - %s", char_2_svc_a);
							strcpy(aux_var_svc_A[1], char_2_svc_a);
						}
					}
					/* Notify Service B */
					sprintf(char_1_svc_b, "%d", notify_data);

					if(strcmp(char_1_svc_b, aux_var_svc_B[0]) != 0){
						if (ble_svc_B_notify_enable[0]) {
							esp_ble_gatts_send_indicate(global_gatts_if,
														global_conn_id,
														svc_B_handle_table[IDX_CHAR_VAL_SVC_B_CHAR1],
														strlen(char_1_svc_b),
														(uint8_t *)char_1_svc_b,
														false);
							ESP_LOGW("NOTIFY", "char_1_svc_b - %s", char_1_svc_b);
							strcpy(aux_var_svc_B[0], char_1_svc_b);
						}
					}
					if(strcmp(char_2_svc_b, aux_var_svc_B[1]) != 0){
						if (ble_svc_B_notify_enable[1]) {
							esp_ble_gatts_send_indicate(global_gatts_if,
														global_conn_id,
														svc_B_handle_table[IDX_CHAR_VAL_SVC_B_CHAR2],
														strlen(char_2_svc_b),
														(uint8_t *)char_2_svc_b,
														false);
							ESP_LOGW("NOTIFY", "char_2_svc_b - %s", char_2_svc_b);
							strcpy(aux_var_svc_B[1], char_2_svc_b);
						}
					}

					notifyDone = true;

				}
			}
		}
	}
	vTaskDelete(NULL);
}

void waitNotify(){
	notifyDone = false;
	while (notifyDone != true){
		TickType_t last_wake_time = xTaskGetTickCount();
		vTaskDelayUntil(&last_wake_time, (200) / portTICK_PERIOD_MS);
	}
}

void printDeviceAddress() {
	const uint8_t* point = esp_bt_dev_get_address();
	char mac[75] = "## QRCODE ## https://slumpmix-mockup.herokuapp.com/qrcode/";
	for (int i = 0; i < 6; i++) {
		char str[3];
		sprintf(str, "%02X", (int) point[i]);
		strcat(mac, str);
		if (i < 5) {
			strcat(mac, ":");
		}
	}
	printf("%s\n", mac);
}

uint32_t password_generator() {
    const uint8_t* point = esp_bt_dev_get_address();
    char mac[17] = "";
	for (int i = 0; i < 6; i++) {
		char str[3];
		sprintf(str, "%02X", (int) point[i]);
		strcat(mac, str);
		if (i < 5) {
			strcat(mac, ":");
		}
	}

	char numbersArray[12] = "";
    unsigned long long int numbers;
    int idx = 0;

    for(int i = 0; i < 17; i++){
        if (mac[i] >= 48 && mac[i] <= 57 && idx < 13){
            numbersArray[idx] = mac[i];
            idx++;
        }
    }
    ESP_LOGE("PASSWORD", "%s", numbersArray);
    sscanf(numbersArray, "%llu", &numbers);
    numbers = (numbers * 666 + 133700) % 1000000u;
    sprintf(numbersArray, "%llu", numbers);
    return numbers;
}

void app_main(void) {
	esp_err_t ret;

	/* Initialize NVS. */
	ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

	ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
	ret = esp_bt_controller_init(&bt_cfg);
	if (ret) {
		ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
		return;
	}

	ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
	if (ret) {
		ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
		return;
	}

	ret = esp_bluedroid_init();
	if (ret) {
		ESP_LOGE(GATTS_TABLE_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
		return;
	}

	ret = esp_bluedroid_enable();
	if (ret) {
		ESP_LOGE(GATTS_TABLE_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
		return;
	}

	ret = esp_ble_gatts_register_callback(gatts_event_handler);
	if (ret) {
		ESP_LOGE(GATTS_TABLE_TAG, "gatts register error, error code = %x", ret);
		return;
	}

	ret = esp_ble_gap_register_callback(gap_event_handler);
	if (ret) {
		ESP_LOGE(GATTS_TABLE_TAG, "gap register error, error code = %x", ret);
		return;
	}

	ret = esp_ble_gatts_app_register(ESP_APP_ID);
	if (ret) {
		ESP_LOGE(GATTS_TABLE_TAG, "gatts app register error, error code = %x", ret);
		return;
	}

	esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
	if (local_mtu_ret) {
		ESP_LOGE(GATTS_TABLE_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
	}

	/* set the security iocap & auth_req & key size & init key response key parameters to the stack*/

	//set static passkey
//	uint32_t passkey = 123456;
	uint32_t passkey = password_generator();
	passkey = 123456;
	ESP_LOGI("PASSWORD", "%u", passkey);
	esp_ble_gap_set_security_param(ESP_BLE_SM_SET_STATIC_PASSKEY, &passkey,
			sizeof(uint32_t));

	esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_MITM_BOND; //bonding with peer device after authentication
	esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req,
			sizeof(uint8_t));

	esp_ble_io_cap_t iocap = ESP_IO_CAP_OUT; //set the IO capability to No output No input
	esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap,
			sizeof(uint8_t));

	uint8_t key_size = 16;      //the key size should be 7~16 bytes
	esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size,
			sizeof(uint8_t));

	uint8_t auth_option = ESP_BLE_ONLY_ACCEPT_SPECIFIED_AUTH_ENABLE;
	esp_ble_gap_set_security_param(ESP_BLE_SM_ONLY_ACCEPT_SPECIFIED_SEC_AUTH,
			&auth_option, sizeof(uint8_t));

	uint8_t oob_support = ESP_BLE_OOB_DISABLE;
	esp_ble_gap_set_security_param(ESP_BLE_SM_OOB_SUPPORT, &oob_support,
			sizeof(uint8_t));

	/*If your BLE device acts as a Slave, the init_key means you hope which types of key of the master should distribute to you,
	 and the response key means which key you can distribute to the master;
	 If your BLE device acts as a master, the response key means you hope which types of key of the slave should distribute to you,
	 and the init key means which key you can distribute to the slave.*/
	uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
	esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key,
			sizeof(uint8_t));

	uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
	esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key,
			sizeof(uint8_t));

	ble_notify_queue = xQueueCreate(20, sizeof(uint32_t));
	xTaskCreate(ble_notify_task, "ble_notify_task", 2048, NULL, 16, NULL);
	printDeviceAddress();

	printf("Menu de variaveis: \n\n1- char_1_svc_a\n2- char_2_svc_a\n\n");

    initUART0();
    xTaskCreate(rx_task_uart0, "uart0_rx_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    show_bonded_devices();
//    remove_all_bonded_devices();

//    xTaskCreate(tx_task, "uart_tx_task", 1024*2, NULL, configMAX_PRIORITIES-1, NULL);
	/* Just show how to clear all the bonded devices
	 * Delay 30s, clear all the bonded devices
	 */

}

void readFromBleVars(int characteristic, char * buf){
	strcpy(buf, write_var_svc_A[characteristic - 1]);
	strcpy(write_var_svc_A[characteristic - 1], "null");
}

void readFromBle_comando(char * ret){
	char buf[50];
	memset(buf, 0, sizeof(buf));
	readFromBleVars(1, buf);
	strcpy(ret, buf);
}


void initUART0(void) {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_0, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM_0, TXD_PIN_UART0, RXD_PIN_UART0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}


void tx_task(void *arg)
{
    while (1) {
//    	uart_write_bytes()
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

char comandoMenuString[10];
int comandoMenuInt = 0;

void rx_task_uart0(void *arg)
{
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM_0, data, RX_BUF_SIZE, 1000 / portTICK_RATE_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            switch (comandoMenuInt){
				case 0: {
					printf("Comando Selecionado: %s\n", data);
					vTaskDelay(100 / portTICK_PERIOD_MS);
					printf("Digite o valor: \n");
					sprintf(comandoMenuString, "%s", (char*)data);
					comandoMenuInt = strtol(comandoMenuString, NULL, 10);
					break;
				}
				case 1: {
					printf("%s\nValor recebido char_1_svc_a: %s", data, data);
					strcpy(char_1_svc_a, (char*)data);
					comandoMenuInt = 0;
					break;
				}
				case 2: {
					printf("%s\nValor recebido char_2_svc_a: %s", data, data);
					strcpy(char_2_svc_a, (char*)data);
					comandoMenuInt = 0;
					break;
				}
			}

            if (comandoMenuInt == 0){
            	printf("\n\nMenu de variaveis: \n\n1- char_1_svc_a\n2- char_2_svc_a\n\n");
			}
        }
    }
    free(data);
}

/* LER INFORMAÇÃO ESCRITA PELO APP E ESCREVER EM RET EM FORMA DE STRING */
void getBuffer(char * ret, const void *buffer, uint16_t buff_len)
{
    if (buff_len == 0) {
        return;
    }
    char temp_buffer[BYTES_PER_LINE + 3];
    char test_buffer[50];
	memset(test_buffer, 0, strlen(test_buffer));

    const char *ptr_line;
    int bytes_cur_line;

    do {
        if (buff_len > BYTES_PER_LINE) {
            bytes_cur_line = BYTES_PER_LINE;
        } else {
            bytes_cur_line = buff_len;
        }
        if (!esp_ptr_byte_accessible(buffer)) {
            //use memcpy to get around alignment issue
            memcpy(temp_buffer, buffer, (bytes_cur_line + 3) / 4 * 4);
            ptr_line = temp_buffer;
        } else {
            ptr_line = buffer;
        }

        for (int i = 0; i < bytes_cur_line; i ++) {
            test_buffer[i] = ptr_line[i];
        }
        buffer += bytes_cur_line;
        buff_len -= bytes_cur_line;
    } while (buff_len);
	memset(ret, 0, strlen(ret));
	strcat(ret, test_buffer);
}



