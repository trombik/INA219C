#include <stdio.h>
#include "unity.h"
#include "unity_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "INA219C.h"

void
unityTask(void *pvParameters) {
	vTaskDelay(1000 / portTICK_PERIOD_MS);
	unity_run_menu();
}

void
app_main() {
	/* Note: if unpinning this task, change the way run times are calculated
	 * in unity_platform */
	xTaskCreatePinnedToCore(unityTask, "unityTask", UNITY_FREERTOS_STACK_SIZE, NULL,
	    UNITY_FREERTOS_PRIORITY, NULL, UNITY_FREERTOS_CPU);
}
