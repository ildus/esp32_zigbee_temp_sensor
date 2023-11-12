#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/gpio_types.h"
#include "soc/gpio_num.h"
#include "esp_log.h"
#include "temp_sensor.h"

#define AHT10_ADDRESS_0X38 \
	0x38 //chip I2C address no.1 for AHT10/AHT15/AHT20, address pin connected to GND
#define AHT10_ADDRESS_0X39 0x39 //chip I2C address no.2 for AHT10 only, address pin connected to Vcc

#define AHT10_INIT_CMD 0xE1 //initialization command for AHT10/AHT15
#define AHT10_START_MEASURMENT_CMD 0xAC //start measurment command
#define AHT10_NORMAL_CMD 0xA8 //normal cycle mode command, no info in datasheet!!!
#define AHT10_SOFT_RESET_CMD 0xBA //soft reset command

#define AHT10_INIT_NORMAL_MODE 0x00 //enable normal mode
#define AHT10_INIT_CYCLE_MODE 0x20 //enable cycle mode
#define AHT10_INIT_CMD_MODE 0x40 //enable command mode
#define AHT10_INIT_CAL_ENABLE 0x08 //load factory calibration coeff

#define AHT10_DATA_MEASURMENT_CMD \
	0x33 //no info in datasheet!!! my guess it is DAC resolution, saw someone send 0x00 instead
#define AHT10_DATA_NOP 0x00 //no info in datasheet!!!

#define AHT10_MEASURMENT_DELAY 80 //at least 75 milliseconds
#define AHT10_POWER_ON_DELAY 40 //at least 20..40 milliseconds
#define AHT10_CMD_DELAY 350 //at least 300 milliseconds, no info in datasheet!!!
#define AHT10_SOFT_RESET_DELAY 20 //less than 20 milliseconds
#define AHT10_ERROR 0xFF //returns 255, if communication error is occurred

static char * TAG = "I2C_AHT10";
static void set_normal_mode(i2c_master_dev_handle_t dev_handle);
static const int DEFAULT_TIMEOUT_MS = 1000;
static const int DEFAULT_WAIT_CYCLE_MS = 10000;

static uint8_t read_status_byte(i2c_master_dev_handle_t dev_handle);
static bool is_calibration_loaded(uint8_t status);
static bool is_busy(uint8_t status);
void enable_factory_calibration(i2c_master_dev_handle_t dev_handle);

static void i2c_delay(uint32_t ms)
{
	vTaskDelay(ms / portTICK_PERIOD_MS);
}

i2c_master_dev_handle_t init_sensor(void)
{
	i2c_master_bus_config_t i2c_mst_config = {
		.clk_source = I2C_CLK_SRC_DEFAULT,
		.i2c_port = -1,
		.scl_io_num = GPIO_NUM_7,
		.sda_io_num = GPIO_NUM_6,
		.glitch_ignore_cnt = 7,
		.flags.enable_internal_pullup = true,
	};

	i2c_master_bus_handle_t bus_handle;
	ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

	i2c_device_config_t dev_cfg = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = AHT10_ADDRESS_0X38,
		.scl_speed_hz = 10000,
	};

	i2c_master_dev_handle_t dev_handle;

	esp_err_t err = i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle);
	ESP_ERROR_CHECK(err);
	if (err != ESP_OK)
		return NULL;

	return dev_handle;
}

static void set_normal_mode(i2c_master_dev_handle_t dev_handle)
{
	uint8_t payload[3] = {AHT10_NORMAL_CMD, AHT10_DATA_NOP, AHT10_DATA_NOP};
	ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, payload, sizeof(payload), DEFAULT_TIMEOUT_MS));
	i2c_delay(AHT10_CMD_DELAY);
}

uint16_t decode_temp(uint8_t * payload)
{
	uint32_t temperature = ((uint32_t)(payload[3] & 0x0F) << 16) | ((uint16_t)payload[4] << 8)
		| payload[5]; //20-bit raw temperature data

	return (uint16_t)((float)temperature * 0.000191 - 50);
}

uint16_t decode_humidity(uint8_t * payload)
{
	uint32_t rawData = (((uint32_t)payload[1] << 16) | ((uint16_t)payload[2] << 8) | (payload[3]))
		>> 4; //20-bit raw humidity data
	float humidity = (float)rawData * 0.000095;

	if (humidity < 0)
		return 0;
	if (humidity > 100)
		return 100;

	return (uint16_t)humidity;
}

static bool read_sensor_data(i2c_master_dev_handle_t dev_handle, SensorData * result)
{
	uint8_t cmd[3] = {AHT10_START_MEASURMENT_CMD, AHT10_DATA_MEASURMENT_CMD, AHT10_DATA_NOP};

	if (i2c_master_transmit(dev_handle, cmd, sizeof(cmd), DEFAULT_TIMEOUT_MS) != ESP_OK)
	{
		ESP_LOGI(TAG, "i2c transmit error");
		return false;
	}

	uint8_t status = read_status_byte(dev_handle); //force to read status byte
	if (status == AHT10_ERROR)
	{
		ESP_LOGI(TAG, "i2c returned error status");
		return false;
	}

	if (!is_calibration_loaded(status))
	{
		//error handler, calibration coefficient turned off
		ESP_LOGI(TAG, "calibration coefficient turned off");
		return false;
	}

	if (is_busy(status))
	{
		i2c_delay(AHT10_MEASURMENT_DELAY);
	}

	uint8_t payload[6];

	if (i2c_master_receive(dev_handle, payload, sizeof(payload), DEFAULT_TIMEOUT_MS) == ESP_OK)
	{
		if (payload[0] == AHT10_ERROR)
		{
			ESP_LOGI(TAG, "payload error");
			return false;
		}

		result->temperature = decode_temp(payload);
		result->humidity = decode_humidity(payload);

		ESP_LOGI(TAG, "got temp from AHT10: %d", result->temperature);
		ESP_LOGI(TAG, "got humidity from AHT10: %d", result->humidity);
	}

	return true;
}

void sensor_soft_reset(i2c_master_dev_handle_t dev_handle)
{
	uint8_t cmd[1] = {AHT10_SOFT_RESET_CMD};
	ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, cmd, sizeof(cmd), DEFAULT_TIMEOUT_MS));

	i2c_delay(AHT10_SOFT_RESET_DELAY);

	// reinitialize sensor registers after reset
	set_normal_mode(dev_handle);
	enable_factory_calibration(dev_handle);
}

static uint8_t read_status_byte(i2c_master_dev_handle_t dev_handle)
{
	uint8_t payload[1];

	if (i2c_master_receive(dev_handle, payload, sizeof(payload), DEFAULT_TIMEOUT_MS) != ESP_OK)
	{
		ESP_LOGI(TAG, "status byte reading error");
		return AHT10_ERROR;
	}

	return payload[0];
}

/* read calibration bit from status byte */
static bool is_calibration_loaded(uint8_t status)
{
	// read 3rd bit
	return status & 0b00001000;
}

/* read busy bit */
static bool is_busy(uint8_t status)
{
	// read 7th bit
	return status & 0b10000000;
}

void enable_factory_calibration(i2c_master_dev_handle_t dev_handle)
{
	uint8_t cmd[3] = {AHT10_INIT_CMD, AHT10_INIT_CAL_ENABLE, AHT10_DATA_NOP};

	if (i2c_master_transmit(dev_handle, cmd, sizeof(cmd), DEFAULT_TIMEOUT_MS) != ESP_OK)
	{
		ESP_LOGI(TAG, "calibration command error");
		return;
	}
	i2c_delay(AHT10_CMD_DELAY);

	uint8_t status = read_status_byte(dev_handle); //force to read status byte
	if (status == AHT10_ERROR)
	{
		ESP_LOGI(TAG, "i2c returned error status");
		return;
	}

	if (!is_calibration_loaded(status))
		ESP_LOGI(TAG, "calibration was not loaded after the command");
}

void set_cycle_mode(i2c_master_dev_handle_t dev_handle)
{
	uint8_t cmd[3]
		= {AHT10_INIT_CMD, AHT10_INIT_CYCLE_MODE | AHT10_INIT_CAL_ENABLE, AHT10_DATA_NOP};

	if (i2c_master_transmit(dev_handle, cmd, sizeof(cmd), DEFAULT_TIMEOUT_MS) != ESP_OK)
	{
		ESP_LOGI(TAG, "cycle and calibration command error");
		return;
	}

	i2c_delay(AHT10_CMD_DELAY);
}

void sensor_reader_task(void * c)
{
	I2C_conf * conf = c;

	SensorData result;
	i2c_master_dev_handle_t dev_handle = conf->handle;
	QueueHandle_t queue = conf->queue;

	if (dev_handle == NULL)
	{
		ESP_LOGI(TAG, "Failed to connect to AHT10 using I2C");
		return;
	}

	i2c_delay(AHT10_POWER_ON_DELAY);

	// initialize sensor registers on start
	set_normal_mode(dev_handle);
	enable_factory_calibration(dev_handle);

	while (1)
	{
		if (read_sensor_data(dev_handle, &result))
        {
			xQueueSend(queue, (void *)&result, 100 / portTICK_PERIOD_MS);
			ESP_LOGI(TAG, "Sent sensor data to the queue");
        }

		i2c_delay(DEFAULT_WAIT_CYCLE_MS);
	}
}
