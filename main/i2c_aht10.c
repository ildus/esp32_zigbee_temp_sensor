#include <stdint.h>

#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/gpio_types.h"
#include "soc/gpio_num.h"
#include "esp_log.h"

#define AHT10_ADDRESS_0X38 \
	0x38 //chip I2C address no.1 for AHT10/AHT15/AHT20, address pin connected to GND
#define AHT10_ADDRESS_0X39 0x39 //chip I2C address no.2 for AHT10 only, address pin connected to Vcc

#define AHT10_INIT_CMD 0xE1 //initialization command for AHT10/AHT15
#define AHT20_INIT_CMD 0xBE //initialization command for AHT20
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

#define AHT10_FORCE_READ_DATA true //force to read data
#define AHT10_USE_READ_DATA false //force to use data from previous read
#define AHT10_ERROR 0xFF //returns 255, if communication error is occurred

static void set_normal_mode(i2c_master_dev_handle_t dev_handle);

typedef enum : uint8_t
{
	AHT10_SENSOR = 0x00,
	AHT15_SENSOR = 0x01,
	AHT20_SENSOR = 0x02
} ASAIR_I2C_SENSOR;

typedef struct
{
	uint16_t temperature;
	uint16_t humidity;
} SensorData;

i2c_master_dev_handle_t init_aht10(void)
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
	if (err == ESP_OK)
	{
		set_normal_mode(dev_handle);
		return dev_handle;
	}

	return NULL;
}

static void set_normal_mode(i2c_master_dev_handle_t dev_handle)
{
	uint8_t payload[3] = {AHT10_NORMAL_CMD, AHT10_DATA_NOP, AHT10_DATA_NOP};
	ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, payload, sizeof(payload), -1));
}

static void read_temp(i2c_master_dev_handle_t dev_handle, SensorData * result)
{
	uint8_t payload[3] = {AHT10_START_MEASURMENT_CMD, AHT10_DATA_MEASURMENT_CMD, AHT10_DATA_NOP};
	ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, payload, sizeof(payload), -1));
}

void sensor_reader_task(void * h)
{
	SensorData result;
	i2c_master_dev_handle_t dev_handle = h;

    if (h == NULL)
    {
        ESP_LOGI("AHT10", "Failed to connect to AHT10 using I2C");
        return;
    }

	while (1)
	{
		read_temp(dev_handle, &result);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}
