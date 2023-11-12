#include <stdint.h>
#include "driver/i2c_master.h"

#define MANUFACTURER_NAME "berlin_iot"
#define MODEL_NAME "esp32.temp_sensor.aht10"
#define FIRMWARE_VERSION "0.1"

i2c_master_dev_handle_t init_sensor(void);
void sensor_reader_task(void * c);

typedef struct
{
	uint16_t temperature;
	uint16_t humidity;
} SensorData;

typedef struct
{
	void * handle;
	void * queue;
} I2C_conf;
