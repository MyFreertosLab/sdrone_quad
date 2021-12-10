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
	float vertical_acc_sum = 0.0f;
	uint16_t calibration_sample = SDRONE_IMU_INIT_CYCLES;
	mpu9250_handle_t imu_data_handle = &sdrone_imu_state_handle->imu;
	while (true) {
		if( ulTaskNotifyTake( pdTRUE,pdMS_TO_TICKS(500) ) == pdTRUE) {
			ESP_ERROR_CHECK(mpu9250_load_data(mpu9250_handle));
			if (calibration_sample > 0) {
				calibration_sample--;
				vertical_acc_sum += mpu9250_handle->data.accel_without_g_if[Z_POS]/SDRONE_IMU_INIT_CYCLES;
				if(calibration_sample == 0) {
					mpu9250_handle->data.accel.acc_g_factor = -mpu9250_handle->data.gravity_bf[Z_POS]/(vertical_acc_sum + SDRONE_GRAVITY_ACCELERATION);
					mpu9250_handle->data.acc_g_factor_initialized = 1;
					mpu9250_handle->data.vertical_acc_offset = vertical_acc_sum*mpu9250_handle->data.accel.acc_g_factor;
					mpu9250_handle->data.speed_if[X_POS] = 0.0f;
					mpu9250_handle->data.speed_if[Y_POS] = 0.0f;
					mpu9250_handle->data.speed_if[Z_POS] = 0.0f;
					printf("Vertical Acc Offset [%5.5f]\n", mpu9250_handle->data.vertical_acc_offset);
					printf("Vertical Acc Factor [%5.5f]\n", mpu9250_handle->data.accel.acc_g_factor);
				}
			};
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
