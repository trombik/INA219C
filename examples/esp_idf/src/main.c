#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <esp_spi_flash.h>
#include <driver/i2c.h>
#include <esp_err.h>
#include <sdkconfig.h>

#include <TRB_INA219.h>

const i2c_port_t port = I2C_NUM_1;

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

	r = i2c_param_config(port, &i2c_config);
	ESP_ERROR_CHECK(r);
	i2c_driver_install(port, i2c_config.mode, 0, 0, 0);
	ESP_ERROR_CHECK(r);
	ina219_set_i2c_port(port);
	return r;
}

void
task_measure(void * pvParameters)
{
	static char log_tag[] = "task_measure";
	struct ina219_dev dev;
	float v_bus;
	float v_shunt;
	float p;
	float i_bus;

	vTaskDelay(2000 / portTICK_PERIOD_MS);
	ESP_LOGI(log_tag, "Starting the task");

	dev = ina219_create(INA219_ADDRESS);
	dev.i_max_expected = 1;

	ESP_LOGI(log_tag, "Initialinzing I2C...");
	i2c_init((gpio_num_t)GPIO_SDA, (gpio_num_t)GPIO_SCL);
	ESP_LOGI(log_tag, "I2C initialized.");

	ESP_LOGI(log_tag, "Resetting INA219...");
	if (ina219_reset(&dev) != 0) {
		ESP_LOGE(log_tag, "failed to ina219_reset()");
		vTaskDelete(NULL);
	}
	ESP_LOGI(log_tag, "done resetting INA219");

	dev.gain = INA219_PGA_GAIN_40MV;
	ESP_LOGI(log_tag, "Configuring...");
	if (ina219_configure(&dev) != 0) {
		ESP_LOGE(log_tag, "Failed to ina219_configure()");
		vTaskDelete(NULL);
	}
	ESP_LOGI(log_tag, "Done Configuring");

	ESP_LOGI(log_tag, "Calibrating...");
	if (ina219_set_calibration(&dev) != 0) {
		ESP_LOGE(log_tag, "failed to ina219_set_calibration()");
		vTaskDelete(NULL);
	}
	ESP_LOGI(log_tag, "Calibrated");

	ESP_LOGI(log_tag, "Waiting for conversion to be ready...");
	for (uint8_t i = 0; i <= 100; i++) {
		if (i == 100) {
			ESP_LOGE(log_tag, "Timeout while ina219_conversion_is_ready()");
		}
		if (ina219_conversion_is_ready(&dev) == INA219_CONVERSION_IS_READY) {
			break;
		}
	}
	ESP_LOGI(log_tag, "conversion is now ready");

	for(;;) {
		if (ina219_get_v_bus(&dev, &v_bus) != 0) {
			ESP_LOGE(log_tag, "failed to ina219_get_v_bus()");
			vTaskDelay(1000 / portTICK_PERIOD_MS);
			continue;
		}
		ESP_LOGI(log_tag, "Vbus: %f", v_bus);
		if (ina219_get_v_shunt(&dev, &v_shunt) != 0) {
			ESP_LOGE(log_tag, "failed to ina219_get_v_shunt()");
			vTaskDelay(1000 / portTICK_PERIOD_MS);
			continue;
		}
		ESP_LOGI(log_tag, "Vshunt: %f", v_shunt);
		if (ina219_get_p_bus(&dev, &p) != 0) {
			ESP_LOGE(log_tag, "failed to ina219_get_p_bus()");
			vTaskDelay(1000 / portTICK_PERIOD_MS);
			continue;
		}
		ESP_LOGI(log_tag, "Power: %f", p);
		if (ina219_get_i_bus(&dev, &i_bus) != 0) {
			ESP_LOGE(log_tag, "failed to ina219_get_i_bus()");
			vTaskDelay(1000 / portTICK_PERIOD_MS);
			continue;
		}
		ESP_LOGI(log_tag, "Current: %f", i_bus);
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
