#if defined(ESP32) && !defined(ARDUINO)
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <esp_spi_flash.h>
#include <driver/i2c.h>
#include <esp_err.h>
#include <INA219C.h>
#include <sdkconfig.h>

#include <INA219C.h>

#define GPIO_SDA 21
#define GPIO_SCL 22

void
halt()
{
	ESP_LOGE(__func__, "Terminating. Please reset");
	while (1)
		vTaskDelay(1000);
}

esp_err_t
i2c_init(const gpio_num_t sda, const gpio_num_t scl)
{
	i2c_config_t i2c_config;
	esp_err_t r;

	i2c_config.mode = I2C_MODE_MASTER;
	i2c_config.sda_io_num = sda;
	i2c_config.scl_io_num = scl;
	i2c_config.sda_pullup_en = GPIO_PULLUP_DISABLE;
	i2c_config.scl_pullup_en = GPIO_PULLUP_DISABLE;
	i2c_config.master.clk_speed = 400000L; // 400KHz

	r = i2c_param_config(I2C_NUM_0, &i2c_config);
	ESP_ERROR_CHECK(r);
	i2c_driver_install(I2C_NUM_0, i2c_config.mode, 0, 0, 0);
	ESP_ERROR_CHECK(r);
	return r;
}

void
task_measure(void * pvParameters)
{
	static char log_tag[] = "test";
	uint16_t data;
	struct ina219c_dev dev;
	int8_t r;
	ina219c_mode mode;
	ina219c_range range;
	ina219c_pga_gain_t gain;

	vTaskDelay(2000 / portTICK_PERIOD_MS);

	dev = ina219c_create(INA219C_ADDRESS);
	ESP_LOGI(log_tag, "Starting the task");

	ESP_LOGI(log_tag, "Initialinzing I2C...");
	i2c_init((gpio_num_t)GPIO_SDA, (gpio_num_t)GPIO_SCL);
	ESP_LOGI(log_tag, "I2C initialized.");

	ESP_LOGI(log_tag, "ina219c_read16() reads INA219C_REG_CONFIG and returns 0");
	if (ina219c_read16(&dev, INA219C_REG_CONFIG, &data) != 0) {
		ESP_LOGE(log_tag, "ina219c_read16() failed.");
		vTaskDelete(NULL);
	}
	ESP_LOGI(log_tag, "ina219c_reset() returns 0");
	if (ina219c_reset(&dev) != 0) {
		ESP_LOGE(log_tag, "failed to ina219c_reset()");
		vTaskDelete(NULL);
	}
	ESP_LOGI(log_tag, "ina219c_read16() reads INA219C_REG_CONFIG");
	if (ina219c_read16(&dev, INA219C_REG_CONFIG, &data) != 0) {
		ESP_LOGE(log_tag, "failed to ina219c_read16()");
		vTaskDelete(NULL);
	}
	ESP_LOGI(log_tag, "INA219C_REG_CONFIG has the default value after reset command");
	if (data != INA219C_REG_CONFIG_DEFAULT) {
		ESP_LOGW(log_tag, "INA219C_REG_CONFIG is not INA219C_REG_CONFIG_DEFAULT after reset: 0x%x", data);
		vTaskDelete(NULL);
	}
	ESP_LOGI(log_tag, "ina219c_get_mode() returns 0");
	if (ina219c_get_mode(&dev, &mode) != 0) {
		ESP_LOGE(log_tag, "failed to ina219c_get_mode()");
		vTaskDelete(NULL);
	}
	ESP_LOGI(log_tag, "ina219c_get_mode() returns default value");
	if (mode != INA219C_MODE_SHUNT_BUS_CONTINUOUS) {
		ESP_LOGE(log_tag, "mode is not INA219C_MODE_SHUNT_BUS_CONTINUOUS: %d", mode);
		vTaskDelete(NULL);
	}
	/*
	ESP_LOGI(log_tag, "ina219c_get_bus_voltage_range() returns default value, 0x%x", INA219C_BUS_VOLTAGE_RANGE_32V);
	if (range != INA219C_BUS_VOLTAGE_RANGE_32V) {
		ESP_LOGE(log_tag, "value is not 0x%x: 0x%x", INA219C_BUS_VOLTAGE_RANGE_32V, range);
		vTaskDelete(NULL);
	}
	*/

	ESP_LOGI(log_tag, "ina219c_get_calibration() returns 0");
	uint16_t value;
	if (ina219c_get_calibration(&dev, &value) != 0) {
		ESP_LOGE(log_tag, "ina219c_get_calibration() failed");
		vTaskDelete(NULL);
	} else {
		ESP_LOGI(log_tag, "calibration value: 0x%x", value);
	}

	ESP_LOGI(log_tag, "ina219c_get_pga_gain() returns 0");
	r = ina219c_get_pga_gain(&dev, &gain);
	if (r != 0) {
		ESP_LOGE(log_tag, "ina219c_get_pga_gain() failed: %d", r);
		vTaskDelete(NULL);
	}
	ESP_LOGI(log_tag, "PGA gain is INA219C_PGA_GAIN_320MV (default)");
	if (gain != INA219C_PGA_GAIN_320MV) {
		ESP_LOGE(log_tag, "gain is not INA219C_PGA_GAIN_320MV: 0x%x", gain);
		vTaskDelete(NULL);
	}

	ina219_resolution_t res;
	ESP_LOGI(log_tag, "ina219c_get_sadc_value() returns 0");
	if (ina219c_get_sadc_value(&dev, &res) != 0) {
		ESP_LOGE(log_tag, "ina219c_get_sadc_value() failed");
		vTaskDelete(NULL);
	}
	ESP_LOGI(log_tag, "ina219c_get_sadc_value() returns default value");
	if (res != INA219C_RESOLUTION_12BIT_1) {
		ESP_LOGE(log_tag, "resolution is not INA219C_RESOLUTION_12BIT_1 0x%x", res);
		vTaskDelete(NULL);
	}

	ESP_LOGI(log_tag, "ina219c_get_badc_value() returns 0");
	if (ina219c_get_badc_value(&dev, &res) != 0) {
		ESP_LOGE(log_tag, "ina219c_get_badc_value() failed");
		vTaskDelete(NULL);
	}
	ESP_LOGI(log_tag, "ina219c_get_badc_value() returns default value");
	if (res != INA219C_RESOLUTION_12BIT_1) {
		ESP_LOGE(log_tag, "resolution is not INA219C_RESOLUTION_12BIT_1 0x%x", res);
		vTaskDelete(NULL);
	}
	ESP_LOGI(log_tag, "decomplement() returns 32000");
	if (ina219c_decomplement(0b1000001100000000, 1) != -32000) {
		ESP_LOGE(log_tag, "decomplement() failed: %d", ina219c_decomplement(0b1000001100000000, 1));
		vTaskDelete(NULL);
	}


	ESP_LOGI(log_tag, "ina219c_set_bus_voltage_range() returns 0");
	range = INA219C_BUS_VOLTAGE_RANGE_16V;
	r = ina219c_set_bus_voltage_range(&dev, range);
	if (r != 0) {
		ESP_LOGE(log_tag, "ina219c_set_bus_voltage_range() failed: %d", r);
		vTaskDelete(NULL);
	}
	if (ina219c_get_bus_voltage_range(&dev, &range) != 0) {
		ESP_LOGE(log_tag, "ina219c_get_bus_voltage_range() failed");
		vTaskDelete(NULL);
	}
	if (range != (ina219c_range)INA219C_BUS_VOLTAGE_RANGE_16V) {
		ESP_LOGE("log_tag", "range is not INA219C_BUS_VOLTAGE_RANGE_16V: 0x%x", range);
		vTaskDelete(NULL);
	}
	ina219c_mode modes[8] = {
		INA219C_MODE_POWERDOWN,
		INA219C_MODE_SHUNT_TRIGGERED,
		INA219C_MODE_BUS_TRIGGERED,
		INA219C_MODE_SHUNT_BUS_TRIGGERED,
		INA219C_MODE_ADC_OFF,
		INA219C_MODE_SHUNT_CONTINUOUS,
		INA219C_MODE_BUS_CONTINUOUS,
		INA219C_MODE_SHUNT_BUS_CONTINUOUS
	};
	for (uint8_t i = 0; i <= sizeof(modes) / sizeof(modes[0]) - 1; i++) {
		ESP_LOGI(log_tag, "ina219c_get_mode() sets mode 0x%x", modes[i]);
		if (ina219c_set_mode(&dev, modes[i]) != 0) {
			ESP_LOGE(log_tag, "ina219c_get_mode() failed");
			vTaskDelete(NULL);
		}
		if (ina219c_get_mode(&dev, &mode) != 0) {
			ESP_LOGE(log_tag, "ina219c_get_mode() failed");
			vTaskDelete(NULL);
		}
		if (mode != modes[i]) {
			ESP_LOGE(log_tag, "expected mode 0x%x got 0x%x", mode, modes[i]);
			vTaskDelete(NULL);
		}
	}
	ESP_LOGI(log_tag, "ina219c_get_bus_voltage_range() returns 0");
	if (ina219c_get_bus_voltage_range(&dev, &range) != 0) {
		ESP_LOGE(log_tag, "ina219c_get_bus_voltage_range() failed");
		vTaskDelete(NULL);
	}

	ESP_LOGI(log_tag, "All tests passed");
	for(;;) {
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}

void
app_main(void)
{

	static char LOG_TAG[] = "app_main";
	esp_chip_info_t chip_info;

	esp_chip_info(&chip_info);
	ESP_LOGI(LOG_TAG, "This is ESP32 chip with %d CPU cores, WiFi%s%s, silicon revision %d",
	    chip_info.cores,
	    (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
	    (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "",
	    chip_info.revision);

	ESP_LOGI(LOG_TAG, "%dMB %s flash\n",
	    spi_flash_get_chip_size() / (1024 * 1024),
	    (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
	TaskHandle_t handle_measure;
	xTaskCreate(task_measure,
	    "measure",
	    configMINIMAL_STACK_SIZE * 3,
	    NULL,
	    1,
	    &handle_measure);
	configASSERT(handle_measure);
	if (handle_measure == NULL) {
		ESP_LOGW(LOG_TAG, "Failed to create measure task");
		vTaskDelete(handle_measure);
	}

	ESP_LOGI(LOG_TAG, "app_main() started");
	for(;;) {
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}
#endif
