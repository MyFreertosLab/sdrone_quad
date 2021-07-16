/*
 * sdrone_controller_task.c
 *
 *  Created on: 18 mag 2021
 *      Author: andrea
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <sdrone_controller_task.h>
#include <sys/time.h>
#include <time.h>

#ifdef MOTORS_FRAME_HORIZONTAL_HEXACOPTER
esp_err_t sdrone_controller_horizontal_hexacopter_init(sdrone_state_handle_t sdrone_state_handle) {
	// TODO: T.B.D.
	return ESP_OK;
}
#else
#ifdef MOTORS_FRAME_TWO_HORIZONTAL_AXIS
esp_err_t sdrone_controller_two_horizontal_axis_init(
		sdrone_state_handle_t sdrone_state_handle) {
#ifdef SDRONE_MISUREMENT_MODE
	sdrone_state_handle->controller_state.misurement_state.misurement_cycle = 0;
	sdrone_state_handle->controller_state.misurement_state.state =
			SDRONE_STATE_SET_START_TARGET;
	sdrone_state_handle->controller_state.misurement_state.target_initial =
			-0.165 * 4.0;
	sdrone_state_handle->controller_state.misurement_state.target =
			sdrone_state_handle->controller_state.misurement_state.target_initial;
	sdrone_state_handle->controller_state.misurement_state.thrust = 1.8;
	sdrone_state_handle->controller_state.misurement_state.target_increment =
			0.165; // 10 deg
	sdrone_state_handle->controller_state.misurement_state.target_final = 0.165
			* 4.0 + 0.001; // 10 deg
#endif
	sdrone_state_handle->controller_state.ierr[SDRONE_TETA_POS] = 0.0f;
	sdrone_state_handle->controller_state.ierr[SDRONE_OMEGA_POS] = 0.0f;
	sdrone_state_handle->controller_state.ierr[SDRONE_ALFA_POS] = 0.0f;
	sdrone_state_handle->controller_state.ke = SDRONE_KE;
	sdrone_state_handle->controller_state.ki = 0.01f;
	sdrone_state_handle->controller_state.kd = 1.2f;
	sdrone_state_handle->controller_state.prevErr[SDRONE_TETA_POS] = 0.0f;
	sdrone_state_handle->controller_state.prevErr[SDRONE_OMEGA_POS] = 0.0f;
	sdrone_state_handle->controller_state.prevErr[SDRONE_ALFA_POS] = 0.0f;

	return ESP_OK;
}
#endif
#endif

void sdrone_controller_init(sdrone_state_handle_t sdrone_state_handle) {
	printf("sdrone_controller_init init initial state and motors\n");
	memset(sdrone_state_handle, 0, sizeof(*sdrone_state_handle));
	sdrone_state_handle->driver_id = (uint32_t) SDRONE_CONTROLLER_DRIVER_ID;
#ifdef MOTORS_FRAME_HORIZONTAL_HEXACOPTER
	printf("sdrone_controller_init initial state sdrone initialized\n");
#else
#ifdef MOTORS_FRAME_TWO_HORIZONTAL_AXIS
	ESP_ERROR_CHECK(
			sdrone_controller_two_horizontal_axis_init(sdrone_state_handle));
#endif
#endif
}

void sdrone_controller_print_data(sdrone_state_handle_t sdrone_state_handle) {
#ifdef SDRONE_MISUREMENT_MODE
#ifdef SDRONE_PRINT_MISUREMENT_MODE
	struct timeval now;
	gettimeofday(&now, NULL);
	float time = (now.tv_sec) + 1e-6*(now.tv_usec);
	printf("%5.3f, %5.3f, %5.3f, %5.3f, %5.3f, %5.3f, %5.3f, %5.3f, %5.3f, %5.3f, %5.3f, %5.3f, %5.3f, %5.3f, %5.6f\n",
			((float) sdrone_state_handle->rc_state.rc_data.data.norm[RC_ROLL]
					* SDRONE_NORM_ROLL_TO_RADIANS_FACTOR),
			sdrone_state_handle->controller_state.X[SDRONE_TETA_POS],
			sdrone_state_handle->controller_state.X[SDRONE_OMEGA_POS],
			sdrone_state_handle->controller_state.X[SDRONE_ALFA_POS],
			((float) sdrone_state_handle->rc_state.rc_data.data.norm[RC_THROTTLE]
					* SDRONE_NORM_THROTTLE_TO_ACCEL_FACTOR),
			sdrone_state_handle->motors_state.input.data.thrust[1],
			sdrone_state_handle->motors_state.input.data.thrust[0],
			sdrone_state_handle->controller_state.err[SDRONE_TETA_POS],
			sdrone_state_handle->controller_state.err[SDRONE_OMEGA_POS],
			sdrone_state_handle->controller_state.err[SDRONE_ALFA_POS],
			sdrone_state_handle->controller_state.predX[SDRONE_TETA_POS],
			sdrone_state_handle->controller_state.predX[SDRONE_OMEGA_POS],
			sdrone_state_handle->controller_state.predX[SDRONE_ALFA_POS],
			sdrone_state_handle->controller_state.derr[SDRONE_ALFA_POS],
			time
			);
	vTaskDelay(1);
#endif
#else
// do not print anything
#endif
}

#ifdef MOTORS_FRAME_HORIZONTAL_HEXACOPTER
#ifdef SDRONE_MISUREMENT_MODE
esp_err_t sdrone_controller_horizontal_hexacopter_misure(sdrone_state_handle_t sdrone_state_handle) {
	// TODO: T.B.D.
	return ESP_OK;
}
#endif
#else
#ifdef MOTORS_FRAME_TWO_HORIZONTAL_AXIS
#ifdef SDRONE_MISUREMENT_MODE
esp_err_t sdrone_controller_two_horizontal_axis_misure(
		sdrone_state_handle_t sdrone_state_handle) {
	switch (sdrone_state_handle->controller_state.misurement_state.state) {
	case SDRONE_STATE_SET_START_TARGET: {
		sdrone_controller_print_data(sdrone_state_handle);
		sdrone_state_handle->controller_state.misurement_state.state =
				SDRONE_STATE_WAIT_START_TARGET;
		sdrone_state_handle->controller_state.misurement_state.target =
				sdrone_state_handle->controller_state.misurement_state.target_initial;
		break;
	}
	case SDRONE_STATE_WAIT_START_TARGET: {
		sdrone_controller_print_data(sdrone_state_handle);
		if (sdrone_state_handle->controller_state.X[SDRONE_TETA_POS]
				- sdrone_state_handle->controller_state.misurement_state.target
				<= 0.02
				&& sdrone_state_handle->controller_state.X[SDRONE_TETA_POS]
						- sdrone_state_handle->controller_state.misurement_state.target
						>= -0.02
				&& sdrone_state_handle->controller_state.X[SDRONE_OMEGA_POS]
						<= 0.001
				&& sdrone_state_handle->controller_state.X[SDRONE_OMEGA_POS]
						>= -0.001) {
			sdrone_state_handle->controller_state.misurement_state.state =
					SDRONE_STATE_START_MISUREMENT;
		}
		break;
	}
	case SDRONE_STATE_START_MISUREMENT: {
		sdrone_controller_print_data(sdrone_state_handle);
		sdrone_state_handle->controller_state.misurement_state.state =
				SDRONE_STATE_INCREMENT_TARGET;
		break;
	}
	case SDRONE_STATE_INCREMENT_TARGET: {
		sdrone_controller_print_data(sdrone_state_handle);
		sdrone_state_handle->controller_state.misurement_state.target +=
				sdrone_state_handle->controller_state.misurement_state.target_increment;
		if (sdrone_state_handle->controller_state.misurement_state.target
				<= sdrone_state_handle->controller_state.misurement_state.target_final) {
			sdrone_state_handle->controller_state.misurement_state.state =
					SDRONE_STATE_WAIT_INCREMENTED_TARGET;
		} else {
			sdrone_state_handle->controller_state.misurement_state.target =
					sdrone_state_handle->controller_state.misurement_state.target_final;
			sdrone_state_handle->controller_state.misurement_state.state =
					SDRONE_STATE_SET_FINAL_TARGET;
		}
		break;
	}
	case SDRONE_STATE_WAIT_INCREMENTED_TARGET: {
		sdrone_controller_print_data(sdrone_state_handle);
		if ((sdrone_state_handle->controller_state.X[SDRONE_TETA_POS]
				- sdrone_state_handle->controller_state.misurement_state.target)
				<= 0.02
				&& (sdrone_state_handle->controller_state.X[SDRONE_TETA_POS]
						- sdrone_state_handle->controller_state.misurement_state.target)
						>= -0.02
				&& sdrone_state_handle->controller_state.X[SDRONE_OMEGA_POS]
						<= 0.001
				&& sdrone_state_handle->controller_state.X[SDRONE_OMEGA_POS]
						>= -0.001) {
			sdrone_state_handle->controller_state.misurement_state.state =
					SDRONE_STATE_INCREMENT_TARGET;
		}
		break;
	}
	case SDRONE_STATE_SET_FINAL_TARGET: {
		sdrone_controller_print_data(sdrone_state_handle);
		if (sdrone_state_handle->controller_state.misurement_state.target
				< (sdrone_state_handle->controller_state.misurement_state.target_initial
						+ sdrone_state_handle->controller_state.misurement_state.target_increment)) {
			sdrone_state_handle->controller_state.misurement_state.state =
					SDRONE_STATE_END;
		} else {
			sdrone_state_handle->controller_state.misurement_state.target -=
					sdrone_state_handle->controller_state.misurement_state.target_increment;
			sdrone_state_handle->controller_state.misurement_state.state =
					SDRONE_STATE_WAIT_FINAL_TARGET;
		}
		break;
	}
	case SDRONE_STATE_WAIT_FINAL_TARGET: {
		sdrone_controller_print_data(sdrone_state_handle);
		if (sdrone_state_handle->controller_state.X[SDRONE_TETA_POS]
				- sdrone_state_handle->controller_state.misurement_state.target
				<= 0.02
				&& sdrone_state_handle->controller_state.X[SDRONE_TETA_POS]
						- sdrone_state_handle->controller_state.misurement_state.target
						>= -0.02
				&& sdrone_state_handle->controller_state.X[SDRONE_OMEGA_POS]
						<= 0.001
				&& sdrone_state_handle->controller_state.X[SDRONE_OMEGA_POS]
						>= -0.001) {
			sdrone_state_handle->controller_state.misurement_state.state =
					SDRONE_STATE_SET_FINAL_TARGET;
		}
		break;
	}
	case SDRONE_STATE_END: {
		sdrone_controller_print_data(sdrone_state_handle);
		sdrone_state_handle->controller_state.misurement_state.state =
				SDRONE_STATE_SET_START_TARGET;
		break;
	}
	}
	return ESP_OK;
}
#endif
#endif
#endif

#ifdef MOTORS_FRAME_HORIZONTAL_HEXACOPTER
esp_err_t sdrone_controller_horizontal_hexacopter_control(sdrone_state_handle_t sdrone_state_handle) {
    // TODO: T.B.D.
	return ESP_OK;
}
#else
#ifdef MOTORS_FRAME_TWO_HORIZONTAL_AXIS
esp_err_t sdrone_controller_two_horizontal_axis_control(
		sdrone_state_handle_t sdrone_state_handle) {
	float Y[2] = { 0.0f, 0.0f }; // the new response

	// Update X from IMU
	sdrone_state_handle->controller_state.X[SDRONE_TETA_POS] =
			sdrone_state_handle->imu_state.imu.data.gyro.rpy.xyz.x;
	sdrone_state_handle->controller_state.X[SDRONE_OMEGA_POS] =
			(double) sdrone_state_handle->imu_state.imu.data.gyro.cal.kalman[X_POS].X
					/ (double) sdrone_state_handle->imu_state.imu.data.gyro.lsb
					/ (double) 360.0f * (double) PI_2;
	sdrone_state_handle->controller_state.X[SDRONE_ALFA_POS] = sdrone_state_handle->imu_state.imu.data.gyro.alfa[X_POS];
		;

	// Update U from RC
	sdrone_state_handle->controller_state.U[SDRONE_TETA_POS] =
			sdrone_state_handle->rc_state.rc_data.data.norm[RC_ROLL]
					* SDRONE_NORM_ROLL_TO_RADIANS_FACTOR
					- sdrone_state_handle->controller_state.X[SDRONE_TETA_POS];
	sdrone_state_handle->controller_state.U[SDRONE_THRUST_POS] =
			sdrone_state_handle->rc_state.rc_data.data.norm[RC_THROTTLE]
					* SDRONE_NORM_THROTTLE_TO_ACCEL_FACTOR
					* ((float) SDRONE_NUM_MOTORS);

	// Calc error (predX(prev) - X(actual))
#ifdef SDRONE_PRINT_MISUREMENT_MODE
	sdrone_state_handle->controller_state.err[SDRONE_TETA_POS] =
			sdrone_state_handle->controller_state.predX[SDRONE_TETA_POS] - sdrone_state_handle->controller_state.X[SDRONE_TETA_POS];
	sdrone_state_handle->controller_state.err[SDRONE_OMEGA_POS] = sdrone_state_handle->controller_state.predX[SDRONE_OMEGA_POS] - sdrone_state_handle->controller_state.X[SDRONE_OMEGA_POS];
#endif
	sdrone_state_handle->controller_state.err[SDRONE_ALFA_POS] =sdrone_state_handle->controller_state.predX[SDRONE_ALFA_POS] - sdrone_state_handle->controller_state.X[SDRONE_ALFA_POS];

#ifdef SDRONE_PRINT_MISUREMENT_MODE
	// Calc integral error
	sdrone_state_handle->controller_state.ierr[SDRONE_TETA_POS] += sdrone_state_handle->controller_state.err[SDRONE_TETA_POS];
	sdrone_state_handle->controller_state.ierr[SDRONE_OMEGA_POS] += sdrone_state_handle->controller_state.err[SDRONE_OMEGA_POS];
	sdrone_state_handle->controller_state.ierr[SDRONE_ALFA_POS] += sdrone_state_handle->controller_state.err[SDRONE_ALFA_POS];
	if(sdrone_state_handle->controller_state.ierr[SDRONE_ALFA_POS] > 20.0f) {
		sdrone_state_handle->controller_state.ierr[SDRONE_ALFA_POS] = 20.0f;
	} else if(sdrone_state_handle->controller_state.ierr[SDRONE_ALFA_POS] < -20.0f) {
			sdrone_state_handle->controller_state.ierr[SDRONE_ALFA_POS] = -20.0f;
	};

	// Calc derivative error
	sdrone_state_handle->controller_state.derr[SDRONE_TETA_POS] =
			sdrone_state_handle->controller_state.err[SDRONE_TETA_POS] - sdrone_state_handle->controller_state.prevErr[SDRONE_TETA_POS];
	sdrone_state_handle->controller_state.derr[SDRONE_OMEGA_POS] =
			sdrone_state_handle->controller_state.err[SDRONE_OMEGA_POS] - sdrone_state_handle->controller_state.prevErr[SDRONE_OMEGA_POS];

	sdrone_state_handle->controller_state.derr[SDRONE_ALFA_POS] =
			sdrone_state_handle->controller_state.err[SDRONE_ALFA_POS] - sdrone_state_handle->controller_state.prevErr[SDRONE_ALFA_POS];

	// Assign prev errors
	sdrone_state_handle->controller_state.prevErr[SDRONE_TETA_POS] = sdrone_state_handle->controller_state.err[SDRONE_TETA_POS];
	sdrone_state_handle->controller_state.prevErr[SDRONE_OMEGA_POS] = sdrone_state_handle->controller_state.err[SDRONE_OMEGA_POS];
	sdrone_state_handle->controller_state.prevErr[SDRONE_ALFA_POS] = sdrone_state_handle->controller_state.err[SDRONE_ALFA_POS];
#endif

	// LPF on U
	float U = sdrone_state_handle->controller_state.U[SDRONE_TETA_POS]*SDRONE_REACTIVITY_FACTOR;
	if(!(-SDRONE_MAX_RADIANS_PER_SECOND <= U && U <= SDRONE_MAX_RADIANS_PER_SECOND)
	  ) {
		if(U < 0.0f) {
			U = -SDRONE_MAX_RADIANS_PER_SECOND;
		} else {
			U = SDRONE_MAX_RADIANS_PER_SECOND;
		}
	}

	// Calc new prediction
	sdrone_state_handle->controller_state.predX[SDRONE_ALFA_POS] =
						 + (U-1.5f*sdrone_state_handle->controller_state.X[SDRONE_OMEGA_POS]);
#ifdef SDRONE_PRINT_MISUREMENT_MODE
	sdrone_state_handle->controller_state.predX[SDRONE_OMEGA_POS] =
			                sdrone_state_handle->controller_state.X[SDRONE_OMEGA_POS]
			              + sdrone_state_handle->controller_state.X[SDRONE_ALFA_POS]*SDRONE_CONTROLLER_DT;
	sdrone_state_handle->controller_state.predX[SDRONE_TETA_POS] =
			               sdrone_state_handle->controller_state.X[SDRONE_TETA_POS]
						 + sdrone_state_handle->controller_state.X[SDRONE_OMEGA_POS]*SDRONE_CONTROLLER_DT
						 + 0.5f*sdrone_state_handle->controller_state.X[SDRONE_ALFA_POS]*SDRONE_CONTROLLER_DT2;
#endif

	// controller response
	Y[0] = (
			+ SDRONE_AXIS_LENGTH*sdrone_state_handle->controller_state.predX[SDRONE_ALFA_POS]
		    + sdrone_state_handle->controller_state.err[SDRONE_ALFA_POS]*sdrone_state_handle->controller_state.ke
		   )
			;
	Y[1] = (
			 - SDRONE_AXIS_LENGTH*sdrone_state_handle->controller_state.predX[SDRONE_ALFA_POS]
		     - sdrone_state_handle->controller_state.err[SDRONE_ALFA_POS]*sdrone_state_handle->controller_state.ke
		   )
			;

	// from accel to newton
	Y[0] = Y[0]
			+ 0.5f * sdrone_state_handle->controller_state.U[SDRONE_THRUST_POS];
	Y[1] = Y[1]
			+ 0.5f * sdrone_state_handle->controller_state.U[SDRONE_THRUST_POS];

	// constraints
	if (Y[0] < 0.0f) {
		Y[0] = 0.0f;
	} else if (Y[1] < 0.0f) {
		Y[1] = 0.0f;
	}
	if (Y[0] > MOTORS_ACCEL_RANGE) {
		Y[0] = MOTORS_ACCEL_RANGE;
	}
	if (Y[1] > MOTORS_ACCEL_RANGE) {
		Y[1] = MOTORS_ACCEL_RANGE;
	}
	sdrone_state_handle->motors_state.input.data.thrust[1] = Y[0]; // right motor
	sdrone_state_handle->motors_state.input.data.thrust[0] = Y[1]; // left motor

	return ESP_OK;
}
#endif
#endif
void sdrone_controller_cycle(sdrone_state_handle_t sdrone_state_handle) {
	rc_data_t rc_data;

	mpu9250_data_t imu_data;
	while (true) {
		if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100)) == pdPASS) {
			memcpy(&rc_data, &sdrone_state_handle->rc_state.rc_data.data,
					sizeof(rc_data));
			memcpy(&imu_data, &sdrone_state_handle->imu_state.imu.data,
					sizeof(imu_data));
			break;
		} else {
			printf("wait for initialization complete ...\n");
		}
	}
	uint16_t skip_counter = 5000;
	while (true) {
		if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100)) != 0) {
			if (sdrone_state_handle->rc_state.rc_data.data.txrx_signal
					== RC_TXRX_TRANSMITTED) {
				memcpy(&rc_data, &sdrone_state_handle->rc_state.rc_data.data,
						sizeof(rc_data));
				sdrone_state_handle->rc_state.rc_data.data.txrx_signal =
						RC_TXRX_RECEIVED;
			}
			if (sdrone_state_handle->imu_state.imu.data.txrx_signal
					== IMU_TXRX_TRANSMITTED) {
				memcpy(&imu_data, &sdrone_state_handle->imu_state.imu.data,
						sizeof(imu_data));
				sdrone_state_handle->imu_state.imu.data.txrx_signal =
						IMU_TXRX_RECEIVED;
				if (skip_counter > 0) {
					skip_counter--;
				} else {

#ifdef MOTORS_FRAME_HORIZONTAL_HEXACOPTER
					ESP_ERROR_CHECK(sdrone_controller_horizontal_hexacopter_control(sdrone_state_handle));
#else
#ifdef MOTORS_FRAME_TWO_HORIZONTAL_AXIS
#ifdef SDRONE_MISUREMENT_MODE
					ESP_ERROR_CHECK(
							sdrone_controller_two_horizontal_axis_misure(
									sdrone_state_handle));
					sdrone_state_handle->rc_state.rc_data.data.norm[RC_ROLL] =
							sdrone_state_handle->controller_state.misurement_state.target
									* (1.0f / SDRONE_NORM_ROLL_TO_RADIANS_FACTOR);
					sdrone_state_handle->rc_state.rc_data.data.norm[RC_THROTTLE] =
							sdrone_state_handle->controller_state.misurement_state.thrust
									* (1.0f
											/ SDRONE_NORM_THROTTLE_TO_ACCEL_FACTOR);
#endif
					ESP_ERROR_CHECK(
							sdrone_controller_two_horizontal_axis_control(
									sdrone_state_handle));
#endif
#endif
					sdrone_state_handle->motors_state.input.isCommand = false;
					sdrone_state_handle->motors_state.input.data.tx_rx_flag =
							MOTORS_TXRX_TRANSMITTED;
					if (sdrone_state_handle->motors_state.motors_task_handle
							!= NULL) {
						xTaskNotify(
								sdrone_state_handle->motors_state.motors_task_handle,
								sdrone_state_handle->driver_id,
								eSetValueWithOverwrite);
					}
				}
			}
		} else {
			// printf("NOT PASS\n");
		}
	}
}

void sdrone_controller_task(void *arg) {
	sdrone_state_handle_t sdrone_state_handle = (sdrone_state_handle_t) arg;
	sdrone_controller_init(sdrone_state_handle);
	sdrone_controller_cycle(sdrone_state_handle);
	vTaskDelete(NULL);
}
