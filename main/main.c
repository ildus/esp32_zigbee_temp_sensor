#include <time.h>
#include <sys/time.h>
#include "string.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "zcl/esp_zigbee_zcl_common.h"
#include "esp_zigbee_core.h"

#include "temp_sensor.h"

/*------ Clobal definitions -----------*/
bool time_updated = false, connected = false;
uint16_t pressure = 0, CO2_value = 0;
static const char * TAG = "TEMP_SENSOR";

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
	ESP_ERROR_CHECK(esp_zb_bdb_start_top_level_commissioning(mode_mask));
}

#define MAX_CHILDREN 10 /* the max amount of connected devices */
#define INSTALLCODE_POLICY_ENABLE false /* enable the install code policy for security */
#define SENSOR_ENDPOINT 1
#define CO2_CUSTOM_CLUSTER 0xFFF2 /* Custom cluster used because standart cluster not working*/
#define ESP_ZB_PRIMARY_CHANNEL_MASK \
	ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK /* Zigbee primary channel mask use in the example */
#define OTA_UPGRADE_MANUFACTURER \
	0x1001 /* The attribute indicates the file version of the downloaded image on the device*/
#define OTA_UPGRADE_IMAGE_TYPE \
	0x1011 /* The attribute indicates the value for the manufacturer of the device */
#define OTA_UPGRADE_FILE_VERSION \
	0x01010101 /* The attribute indicates the file version of the running firmware image on the device */
#define OTA_UPGRADE_HW_VERSION 0x0101 /* The parameter indicates the version of hardware */
#define OTA_UPGRADE_MAX_DATA_SIZE \
	64 /* The parameter indicates the maximum data size of query block image */

/* Manual reporting atribute to coordinator */
static void reportAttribute(
	uint8_t endpoint, uint16_t clusterID, uint16_t attributeID, void * value, uint8_t value_length)
{
	esp_zb_zcl_report_attr_cmd_t cmd = {
        .zcl_basic_cmd = {
            .dst_addr_u.addr_short = 0x0000,
            .dst_endpoint = endpoint,
            .src_endpoint = endpoint,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = clusterID,
        .attributeID = attributeID,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
    };
	esp_zb_zcl_attr_t * value_r = esp_zb_zcl_get_attribute(
		endpoint, clusterID, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, attributeID);
	memcpy(value_r->data_p, value, value_length);
	esp_zb_zcl_report_attr_cmd_req(&cmd);
}

/* Task for update attribute value */
void update_attribute(void * q)
{
	SensorData sensor_data;
	QueueHandle_t queue = q;

	while (true)
	{
		if (connected && xQueueReceive(queue, &sensor_data, 0) == pdPASS)
		{
			ESP_LOGI(TAG, "Got sensor data from the queue, sending..");

			/* Write new temperature value */
			esp_zb_zcl_status_t state_tmp = esp_zb_zcl_set_attribute_val(
				SENSOR_ENDPOINT,
				ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
				ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
				ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
				&sensor_data.temperature,
				false);

			/* Check for error */
			if (state_tmp != ESP_ZB_ZCL_STATUS_SUCCESS)
			{
				ESP_LOGE(TAG, "Setting temperature attribute failed!");
			}

			/* Write new humidity value */
			esp_zb_zcl_status_t state_hum = esp_zb_zcl_set_attribute_val(
				SENSOR_ENDPOINT,
				ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
				ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
				ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID,
				&sensor_data.humidity,
				false);

			/* Check for error */
			if (state_hum != ESP_ZB_ZCL_STATUS_SUCCESS)
			{
				ESP_LOGE(TAG, "Setting humidity attribute failed!");
			}

			/* Write new pressure value */
			if (pressure != 0)
			{
				esp_zb_zcl_status_t state_press = esp_zb_zcl_set_attribute_val(
					SENSOR_ENDPOINT,
					ESP_ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT,
					ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
					ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_VALUE_ID,
					&pressure,
					false);

				/* Check for error */
				if (state_press != ESP_ZB_ZCL_STATUS_SUCCESS)
				{
					ESP_LOGE(TAG, "Setting pressure attribute failed!");
				}
			}

			if (CO2_value != 0)
			{
				/* Write new CO2_value value */
				esp_zb_zcl_status_t state_co2 = esp_zb_zcl_set_attribute_val(
					SENSOR_ENDPOINT,
					CO2_CUSTOM_CLUSTER,
					ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
					0,
					&CO2_value,
					false);

				/* Check for error */
				if (state_co2 != ESP_ZB_ZCL_STATUS_SUCCESS)
				{
					ESP_LOGE(TAG, "Setting CO2_value attribute failed!");
				}

				/* CO2 Cluster is custom and we must report it manually*/
				reportAttribute(SENSOR_ENDPOINT, CO2_CUSTOM_CLUSTER, 0, &CO2_value, 2);
			}
		}
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}

static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t * message)
{
	esp_err_t ret = ESP_OK;
	ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
	ESP_RETURN_ON_FALSE(
		message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS,
		ESP_ERR_INVALID_ARG,
		TAG,
		"Received message: error status(%d)",
		message->info.status);
	ESP_LOGI(
		TAG,
		"Received message: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d)",
		message->info.dst_endpoint,
		message->info.cluster,
		message->attribute.id,
		message->attribute.data.size);
	if (message->info.dst_endpoint == SENSOR_ENDPOINT)
	{
		switch (message->info.cluster)
		{
			case ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY:
				ESP_LOGI(TAG, "Identify pressed");
				break;
			default:
				ESP_LOGI(
					TAG,
					"Message data: cluster(0x%x), attribute(0x%x)  ",
					message->info.cluster,
					message->attribute.id);
		}
	}
	return ret;
}

static esp_err_t zb_read_attr_resp_handler(const esp_zb_zcl_cmd_read_attr_resp_message_t * message)
{
	ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
	ESP_RETURN_ON_FALSE(
		message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS,
		ESP_ERR_INVALID_ARG,
		TAG,
		"Received message: error status(%d)",
		message->info.status);
	ESP_LOGI(
		TAG,
		"Read attribute response: status(%d), cluster(0x%x), attribute(0x%x), type(0x%x), "
		"value(%d)",
		message->info.status,
		message->info.cluster,
		message->attribute.id,
		message->attribute.data.type,
		message->attribute.data.value ? *(uint8_t *)message->attribute.data.value : 0);

	if (message->info.dst_endpoint == SENSOR_ENDPOINT)
	{
		switch (message->info.cluster)
		{
			case ESP_ZB_ZCL_CLUSTER_ID_TIME: {
				struct timeval tv;
				tv.tv_sec = *(uint32_t *)message->attribute.data.value + 946684800
					- 1080; //after adding OTA cluster time shifted to 1080 sec... strange issue ...
				settimeofday(&tv, NULL);
				time_updated = true;
				break;
			}
			default: {
				ESP_LOGI(
					TAG,
					"Message data: cluster(0x%x), attribute(0x%x)  ",
					message->info.cluster,
					message->attribute.id);
			}
		}
	}
	return ESP_OK;
}

static esp_err_t
zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void * message)
{
	esp_err_t ret = ESP_OK;
	switch (callback_id)
	{
		case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
			ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)message);
			break;
		case ESP_ZB_CORE_CMD_READ_ATTR_RESP_CB_ID:
			ret = zb_read_attr_resp_handler((esp_zb_zcl_cmd_read_attr_resp_message_t *)message);
			break;
		default:
			ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
			break;
	}
	return ret;
}

void read_server_time()
{
	esp_zb_zcl_read_attr_cmd_t read_req;
	read_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
	read_req.attributeID = ESP_ZB_ZCL_ATTR_TIME_LOCAL_TIME_ID;
	read_req.clusterID = ESP_ZB_ZCL_CLUSTER_ID_TIME;
	read_req.zcl_basic_cmd.dst_endpoint = 1;
	read_req.zcl_basic_cmd.src_endpoint = 1;
	read_req.zcl_basic_cmd.dst_addr_u.addr_short = 0x0000;
	esp_zb_zcl_read_attr_cmd_req(&read_req);
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t * signal_struct)
{
	uint32_t * p_sg_p = signal_struct->p_app_signal;
	esp_err_t err_status = signal_struct->esp_err_status;
	esp_zb_app_signal_type_t sig_type = *p_sg_p;
	esp_zb_zdo_signal_leave_params_t * leave_params = NULL;
	switch (sig_type)
	{
		case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
		case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
		case ESP_ZB_BDB_SIGNAL_STEERING:
			if (err_status != ESP_OK)
			{
				connected = false;
				ESP_LOGW(
					TAG,
					"Stack %s failure with %s status, steering",
					esp_zb_zdo_signal_to_string(sig_type),
					esp_err_to_name(err_status));
				esp_zb_scheduler_alarm(
					(esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
					ESP_ZB_BDB_MODE_NETWORK_STEERING,
					1000);
			}
			else
			{
				/* device auto start successfully and on a formed network */
				connected = true;
				esp_zb_ieee_addr_t extended_pan_id;
				esp_zb_get_extended_pan_id(extended_pan_id);
				ESP_LOGI(
					TAG,
					"Joined network successfully (Extended PAN ID: "
					"%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d)",
					extended_pan_id[7],
					extended_pan_id[6],
					extended_pan_id[5],
					extended_pan_id[4],
					extended_pan_id[3],
					extended_pan_id[2],
					extended_pan_id[1],
					extended_pan_id[0],
					esp_zb_get_pan_id(),
					esp_zb_get_current_channel());
				read_server_time();
			}
			break;
		case ESP_ZB_ZDO_SIGNAL_LEAVE:
			leave_params = (esp_zb_zdo_signal_leave_params_t *)esp_zb_app_signal_get_params(p_sg_p);
			if (leave_params->leave_type == ESP_ZB_NWK_LEAVE_TYPE_RESET)
			{
				ESP_LOGI(TAG, "Reset device");
				esp_zb_factory_reset();
			}
			break;
		default:
			ESP_LOGI(
				TAG,
				"ZDO signal: %s (0x%x), status: %s",
				esp_zb_zdo_signal_to_string(sig_type),
				sig_type,
				esp_err_to_name(err_status));
			break;
	}
}

static void set_zcl_string(char * buffer, char * value)
{
	buffer[0] = (char)strlen(value);
	memcpy(buffer + 1, value, buffer[0]);
}

#define ESP_ZB_ZR_CONFIG() \
	{ \
		.esp_zb_role = ESP_ZB_DEVICE_TYPE_ROUTER, \
		.install_code_policy = INSTALLCODE_POLICY_ENABLE, \
		.nwk_cfg.zczr_cfg = { \
			.max_children = MAX_CHILDREN, \
		}, \
	}

static void esp_zb_task(void * pvParameters)
{
	static char manufacturer[16], model[16], firmware_version[16];

	/* initialize Zigbee stack */
	esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZR_CONFIG();
	esp_zb_init(&zb_nwk_cfg);

	uint16_t undefined_value;
	undefined_value = 0x8000;
	/* basic cluster create with fully customized */
	set_zcl_string(manufacturer, MANUFACTURER_NAME);
	set_zcl_string(model, MODEL_NAME);
	set_zcl_string(firmware_version, FIRMWARE_VERSION);
	uint8_t dc_power_source;
	dc_power_source = 4;
	esp_zb_attribute_list_t * esp_zb_basic_cluster
		= esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_BASIC);
	esp_zb_basic_cluster_add_attr(
		esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, manufacturer);
	esp_zb_basic_cluster_add_attr(
		esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, model);
	esp_zb_basic_cluster_add_attr(
		esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_SW_BUILD_ID, firmware_version);
	esp_zb_basic_cluster_add_attr(
		esp_zb_basic_cluster,
		ESP_ZB_ZCL_ATTR_BASIC_POWER_SOURCE_ID,
		&dc_power_source); /**< DC source. */

	/* identify cluster create with fully customized */
	uint8_t identyfi_id;
	identyfi_id = 0;
	esp_zb_attribute_list_t * esp_zb_identify_cluster
		= esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY);
	esp_zb_identify_cluster_add_attr(
		esp_zb_identify_cluster, ESP_ZB_ZCL_CMD_IDENTIFY_IDENTIFY_ID, &identyfi_id);

	/* Temperature cluster */
	esp_zb_attribute_list_t * esp_zb_temperature_meas_cluster
		= esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT);
	esp_zb_temperature_meas_cluster_add_attr(
		esp_zb_temperature_meas_cluster,
		ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
		&undefined_value);
	esp_zb_temperature_meas_cluster_add_attr(
		esp_zb_temperature_meas_cluster,
		ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_MIN_VALUE_ID,
		&undefined_value);
	esp_zb_temperature_meas_cluster_add_attr(
		esp_zb_temperature_meas_cluster,
		ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_MAX_VALUE_ID,
		&undefined_value);

	/* Humidity cluster */
	esp_zb_attribute_list_t * esp_zb_humidity_meas_cluster
		= esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT);
	esp_zb_humidity_meas_cluster_add_attr(
		esp_zb_humidity_meas_cluster,
		ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID,
		&undefined_value);
	esp_zb_humidity_meas_cluster_add_attr(
		esp_zb_humidity_meas_cluster,
		ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_MIN_VALUE_ID,
		&undefined_value);
	esp_zb_humidity_meas_cluster_add_attr(
		esp_zb_humidity_meas_cluster,
		ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_MAX_VALUE_ID,
		&undefined_value);

	/* Presure cluster */
	esp_zb_attribute_list_t * esp_zb_press_meas_cluster
		= esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT);
	esp_zb_pressure_meas_cluster_add_attr(
		esp_zb_press_meas_cluster, ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_VALUE_ID, &undefined_value);
	esp_zb_pressure_meas_cluster_add_attr(
		esp_zb_press_meas_cluster,
		ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_MIN_VALUE_ID,
		&undefined_value);
	esp_zb_pressure_meas_cluster_add_attr(
		esp_zb_press_meas_cluster,
		ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_MAX_VALUE_ID,
		&undefined_value);

	/* Time cluster */
	esp_zb_attribute_list_t * esp_zb_server_time_cluster
		= esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_TIME);

	/* Custom cluster for CO2 ( standart cluster not working), solution only for HOMEd */
	const uint16_t attr_id = 0;
	const uint8_t attr_type = ESP_ZB_ZCL_ATTR_TYPE_U16;
	const uint8_t attr_access = ESP_ZB_ZCL_ATTR_MANUF_SPEC | ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY
		| ESP_ZB_ZCL_ATTR_ACCESS_REPORTING;

	esp_zb_attribute_list_t * custom_co2_attributes_list
		= esp_zb_zcl_attr_list_create(CO2_CUSTOM_CLUSTER);
	esp_zb_custom_cluster_add_custom_attr(
		custom_co2_attributes_list, attr_id, attr_type, attr_access, &undefined_value);

	/** Create ota client cluster with attributes.
     *  Manufacturer code, image type and file version should match with configured values for server.
     *  If the client values do not match with configured values then it shall discard the command and
     *  no further processing shall continue.
     */
	esp_zb_ota_cluster_cfg_t ota_cluster_cfg = {
		.ota_upgrade_downloaded_file_ver = OTA_UPGRADE_FILE_VERSION,
		.ota_upgrade_manufacturer = OTA_UPGRADE_MANUFACTURER,
		.ota_upgrade_image_type = OTA_UPGRADE_IMAGE_TYPE,
	};
	esp_zb_attribute_list_t * esp_zb_ota_client_cluster
		= esp_zb_ota_cluster_create(&ota_cluster_cfg);
	/** add client parameters to ota client cluster */
	esp_zb_ota_upgrade_client_parameter_t ota_client_parameter_config = {
		.query_timer
		= ESP_ZB_ZCL_OTA_UPGRADE_QUERY_TIMER_COUNT_DEF, /* time interval for query next image request command */
		.hardware_version = OTA_UPGRADE_HW_VERSION, /* version of hardware */
		.max_data_size = OTA_UPGRADE_MAX_DATA_SIZE, /* maximum data size of query block image */
	};
	void * ota_client_parameters = esp_zb_ota_client_parameter(&ota_client_parameter_config);
	esp_zb_ota_cluster_add_attr(
		esp_zb_ota_client_cluster,
		ESP_ZB_ZCL_ATTR_OTA_UPGRADE_CLIENT_PARAMETER_ID,
		ota_client_parameters);

	/* Create full cluster list enabled on device */
	esp_zb_cluster_list_t * esp_zb_cluster_list = esp_zb_zcl_cluster_list_create();
	esp_zb_cluster_list_add_basic_cluster(
		esp_zb_cluster_list, esp_zb_basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
	esp_zb_cluster_list_add_identify_cluster(
		esp_zb_cluster_list, esp_zb_identify_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
	esp_zb_cluster_list_add_temperature_meas_cluster(
		esp_zb_cluster_list, esp_zb_temperature_meas_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
	esp_zb_cluster_list_add_humidity_meas_cluster(
		esp_zb_cluster_list, esp_zb_humidity_meas_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
	esp_zb_cluster_list_add_pressure_meas_cluster(
		esp_zb_cluster_list, esp_zb_press_meas_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
	esp_zb_cluster_list_add_time_cluster(
		esp_zb_cluster_list, esp_zb_server_time_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
	esp_zb_cluster_list_add_custom_cluster(
		esp_zb_cluster_list, custom_co2_attributes_list, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
	esp_zb_cluster_list_add_ota_cluster(
		esp_zb_cluster_list, esp_zb_ota_client_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);

	esp_zb_ep_list_t * esp_zb_ep_list = esp_zb_ep_list_create();
	esp_zb_ep_list_add_ep(
		esp_zb_ep_list,
		esp_zb_cluster_list,
		SENSOR_ENDPOINT,
		ESP_ZB_AF_HA_PROFILE_ID,
		ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID);

	/* END */
	esp_zb_device_register(esp_zb_ep_list);
	esp_zb_core_action_handler_register(zb_action_handler);
	esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
	ESP_ERROR_CHECK(esp_zb_start(true));
	esp_zb_main_loop_iteration();
}

#define ESP_ZB_DEFAULT_RADIO_CONFIG() \
	{ \
		.radio_mode = RADIO_MODE_NATIVE, \
	}

#define ESP_ZB_DEFAULT_HOST_CONFIG() \
	{ \
		.host_connection_mode = HOST_CONNECTION_MODE_NONE, \
	}

void app_main(void)
{
	QueueHandle_t queue = xQueueCreate(1, sizeof(SensorData));
	I2C_conf conf;

	esp_zb_platform_config_t config = {
		.radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
		.host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
	};
	ESP_ERROR_CHECK(nvs_flash_init());
	ESP_ERROR_CHECK(esp_zb_platform_config(&config));

	conf.handle = init_sensor();
	conf.queue = queue;

	xTaskCreate(sensor_reader_task, "sensor_task", 4096, &conf, 1, NULL);
	xTaskCreate(update_attribute, "update_attr", 4096, queue, 5, NULL);
	xTaskCreate(esp_zb_task, "zigbee_main", 4096, NULL, 6, NULL);
}
