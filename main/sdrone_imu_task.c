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
#include <mpu9250_calibrator.h>

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

	ESP_ERROR_CHECK(mpu9250_gyro_set_fsr(mpu9250_handle, INV_FSR_1000DPS));
	ESP_ERROR_CHECK(mpu9250_acc_set_fsr(mpu9250_handle, INV_FSR_8G));
	ESP_ERROR_CHECK(sdrone_imu_discard_messages(sdrone_imu_state_handle, 10000));
	uint8_t counter = 0;
	mpu9250_handle_t imu_data_handle = &sdrone_imu_state_handle->imu;

	while (true) {

		if( ulTaskNotifyTake( pdTRUE,pdMS_TO_TICKS(500) ) == pdTRUE) {
			counter++;
			counter %=100;
			ESP_ERROR_CHECK(mpu9250_load_data(mpu9250_handle));
			if ((sdrone_imu_state_handle->controller_task_handle != NULL) && (imu_data_handle->data.txrx_signal != IMU_TXRX_TRANSMITTED)) {
				memcpy(imu_data_handle, mpu9250_handle, sizeof(mpu9250_init_t));
				imu_data_handle->data.txrx_signal = IMU_TXRX_TRANSMITTED;
				xTaskNotify(sdrone_imu_state_handle->controller_task_handle,sdrone_imu_state_handle->driver_id,eSetValueWithOverwrite);
			}

//			if(counter%100 == 0) {
//				printf("Gyro.: S[%d][%d][%d] X[%d][%d][%d]\n", mpu9250_handle->data.gyro.cal.kalman[X_POS].sample, mpu9250_handle->data.gyro.cal.kalman[Y_POS].sample, mpu9250_handle->data.gyro.cal.kalman[Z_POS].sample, mpu9250_handle->data.gyro.cal.kalman[X_POS].X , mpu9250_handle->data.gyro.cal.kalman[Y_POS].X, mpu9250_handle->data.gyro.cal.kalman[Z_POS].X);
//				printf("Accel: S[%d][%d][%d] X[%d][%d][%d]\n", mpu9250_handle->data.accel.cal.kalman[X_POS].sample, mpu9250_handle->data.accel.cal.kalman[Y_POS].sample, mpu9250_handle->data.accel.cal.kalman[Z_POS].sample, mpu9250_handle->data.accel.cal.kalman[X_POS].X , mpu9250_handle->data.accel.cal.kalman[Y_POS].X, mpu9250_handle->data.accel.cal.kalman[Z_POS].X);
//				printf("AG[%2.2f][%2.2f][%2.2f] RPY[%2.2f][%2.2f][%2.2f]\n", mpu9250_handle->data.attitude[X_POS], mpu9250_handle->data.attitude[Y_POS], mpu9250_handle->data.attitude[Z_POS], mpu9250_handle->data.gyro.rpy.xyz.x*(double)360.0f/(double)6.283185307f, mpu9250_handle->data.gyro.rpy.xyz.y*(double)360.0f/(double)6.283185307f, mpu9250_handle->data.gyro.rpy.xyz.z*(double)360.0f/(double)6.283185307f);
//				printf("AA[%2.2f][%2.2f][%2.2f] RPY[%2.2f][%2.2f][%2.2f]\n", mpu9250_handle->data.attitude[X_POS], mpu9250_handle->data.attitude[Y_POS], mpu9250_handle->data.attitude[Z_POS], mpu9250_handle->data.accel.rpy.xyz.x*(double)360.0f/(double)6.283185307f, mpu9250_handle->data.accel.rpy.xyz.y*(double)360.0f/(double)6.283185307f, mpu9250_handle->data.accel.rpy.xyz.z*(double)360.0f/(double)6.283185307f);
//				printf("KG[%1.5f][%1.5f][%1.5f]\n", mpu9250_handle->data.gyro.cal.kalman[X_POS].K, mpu9250_handle->data.gyro.cal.kalman[Y_POS].K, mpu9250_handle->data.gyro.cal.kalman[Z_POS].K );
//			}
	    } else {
	    	ESP_ERROR_CHECK(mpu9250_test_connection(mpu9250_handle));
//			if(counter%100 == 0) {
//		    	printf("SORRY!! Interrupt LOST!\n");
//			}
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
