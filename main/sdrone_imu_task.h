/*
 * sdrone_imu_task.h
 *
 *  Created on: 22 mag 2021
 *      Author: andrea
 */

#ifndef MAIN_SDRONE_IMU_TASK_H_
#define MAIN_SDRONE_IMU_TASK_H_

#include <mpu9250.h>

typedef struct {
	uint32_t driver_id;
	uint32_t controller_driver_id;
	mpu9250_init_t imu;
	TaskHandle_t controller_task_handle;
	TaskHandle_t imu_task_handle;
} sdrone_imu_state_t;
typedef sdrone_imu_state_t* sdrone_imu_state_handle_t;

void sdrone_imu_task(void *arg);


#endif /* MAIN_SDRONE_IMU_TASK_H_ */
