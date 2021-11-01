/*
 * sdrone_imu_task.c
 *
 *  Created on: 22 mag 2021
 *      Author: andrea
 */

#include <string.h>
#include <sdrone_imu_task.h>
#include <mpu9250_accel.h>
#include <mpu9250_gyro.h>
#include <mpu9250_mag.h>

void sdrone_imu_init(
		sdrone_imu_state_handle_t sdrone_imu_state_handle) {
	printf("sdrone_imu_init init initial state and imu\n");
	memset(sdrone_imu_state_handle, 0, sizeof(*sdrone_imu_state_handle));
	ESP_ERROR_CHECK(mpu9250_init(&(sdrone_imu_state_handle->imu)));
	sdrone_imu_state_handle->imu_task_handle =
			xTaskGetCurrentTaskHandle();
	printf(
			"sdrone_imu_init initial state and imu initialized\n");
}

esp_err_t sdrone_imu_discard_messages(sdrone_imu_state_handle_t sdrone_imu_state_handle, uint16_t num_msgs) {
	printf("Discarding %d Samples ... \n", num_msgs);
	for(uint16_t i = 0; i < num_msgs; i++) {
		if(ulTaskNotifyTake( pdTRUE,pdMS_TO_TICKS(500)) == pdTRUE ) {

		}
	}
	printf("Samples discarded\n");
	return ESP_OK;
}

void sdrone_imu_read_data_cycle(sdrone_imu_state_handle_t sdrone_imu_state_handle) {
	mpu9250_init_t imu_data_local;
	mpu9250_handle_t mpu9250_handle = &imu_data_local;
	memcpy(mpu9250_handle, &sdrone_imu_state_handle->imu, sizeof(*mpu9250_handle));

	mpu9250_handle_t imu_data_handle = &sdrone_imu_state_handle->imu;
	uint8_t acc_g_factor_initialized = 0;
	while (true) {
		if( ulTaskNotifyTake( pdTRUE,pdMS_TO_TICKS(500) ) == pdTRUE) {
			ESP_ERROR_CHECK(mpu9250_load_data(mpu9250_handle));
			if(acc_g_factor_initialized == 0) {
				sdrone_imu_state_handle->imu.data.accel.acc_g_factor = mpu9250_handle->data.attitude[Z_POS]/sdrone_imu_state_handle->imu.data.accel.mss.array[Z_POS];
				acc_g_factor_initialized = 1;
			}
			if ((sdrone_imu_state_handle->controller_task_handle != NULL) && (imu_data_handle->data.txrx_signal != IMU_TXRX_TRANSMITTED)) {
				memcpy(imu_data_handle, mpu9250_handle, sizeof(mpu9250_init_t));
				imu_data_handle->data.txrx_signal = IMU_TXRX_TRANSMITTED;
				xTaskNotify(sdrone_imu_state_handle->controller_task_handle,sdrone_imu_state_handle->driver_id,eSetValueWithOverwrite);
			}

	    } else {
	    	ESP_ERROR_CHECK(mpu9250_test_connection(mpu9250_handle));
	    }
	}
}

void sdrone_imu_cycle(sdrone_imu_state_handle_t sdrone_imu_state_handle) {
	mpu9250_handle_t mpu9250_handle = &sdrone_imu_state_handle->imu;
	// load circular buffer
	for(uint8_t i = 0; i < CIRCULAR_BUFFER_SIZE; i++) {
		if( ulTaskNotifyTake( pdTRUE,pdMS_TO_TICKS(500) ) == 1) {
			ESP_ERROR_CHECK(mpu9250_load_raw_data(mpu9250_handle));
		}
	}
	sdrone_imu_read_data_cycle(sdrone_imu_state_handle);
}
void sdrone_imu_task(void *arg) {
	sdrone_imu_state_handle_t sdrone_imu_state_handle =
			(sdrone_imu_state_handle_t) arg;
	sdrone_imu_init(sdrone_imu_state_handle);
	sdrone_imu_cycle(sdrone_imu_state_handle);
}
