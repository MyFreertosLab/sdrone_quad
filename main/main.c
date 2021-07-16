#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_system.h>
#include <nvs_flash.h>
#include <nvs.h>
#include <driver/i2c.h>
#include <sdrone_controller_task.h>

static sdrone_state_t sdrone_state;
static sdrone_state_handle_t sdrone_state_handle = &sdrone_state;
static TaskHandle_t controller_task_handle;
static TaskHandle_t motors_task_handle;
static TaskHandle_t rc_task_handle;
static TaskHandle_t imu_task_handle;

static esp_err_t my_i2c_init(int i2c_master_port) {
	i2c_config_t conf = {
	    .mode = I2C_MODE_MASTER,
	    .sda_io_num = GPIO_NUM_4,         // select GPIO specific to your project
	    .sda_pullup_en = GPIO_PULLUP_ENABLE,
	    .scl_io_num = GPIO_NUM_0,         // select GPIO specific to your project
	    .scl_pullup_en = GPIO_PULLUP_ENABLE,
	    .master.clk_speed = 400000,  // select frequency specific to your project
	    // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
	};
	ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));
	ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, I2C_MODE_MASTER, 0,0,0));
	return ESP_OK;
}

esp_err_t my_nvs_init(){
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
	return err;
}

void app_main(void)
{

	/*
	 * TODO: define component for hw initialization
	 */
	ESP_ERROR_CHECK(my_nvs_init());
	ESP_ERROR_CHECK(my_i2c_init(1));

	// init controller (this must be the first)
    xTaskCreate(sdrone_controller_task, "sdrone_controller_task", 4096, sdrone_state_handle, 8, &controller_task_handle);

    // init actuators
    xTaskCreate(sdrone_motors_task, "sdrone_motors_task", 4096, &(sdrone_state_handle->motors_state), 9, &motors_task_handle);

    // init sensors
    xTaskCreate(sdrone_rc_task, "sdrone_rc_task", 2048, &(sdrone_state_handle->rc_state), 5, &rc_task_handle);
    xTaskCreate(sdrone_imu_task, "sdrone_imu_task", 4096, &(sdrone_state_handle->imu_state), 10, &imu_task_handle);

    vTaskDelay(pdMS_TO_TICKS(100));

    sdrone_state_handle->motors_state.controller_task_handle = controller_task_handle;
    sdrone_state_handle->motors_state.driver_id = (uint32_t)SDRONE_MOTORS_DRIVER_ID;
    sdrone_state_handle->motors_state.controller_driver_id = (uint32_t)SDRONE_CONTROLLER_DRIVER_ID;

    sdrone_state_handle->rc_state.controller_task_handle = controller_task_handle;
    sdrone_state_handle->rc_state.driver_id = (uint32_t)SDRONE_RC_DRIVER_ID;
    sdrone_state_handle->rc_state.controller_driver_id = (uint32_t)SDRONE_CONTROLLER_DRIVER_ID;

    sdrone_state_handle->imu_state.controller_task_handle = controller_task_handle;
    sdrone_state_handle->imu_state.driver_id = (uint32_t)SDRONE_IMU_DRIVER_ID;
    sdrone_state_handle->imu_state.controller_driver_id = (uint32_t)SDRONE_CONTROLLER_DRIVER_ID;

    // Send notification to Controller.
	xTaskNotify(controller_task_handle,pdPASS,eSetValueWithOverwrite);

}

