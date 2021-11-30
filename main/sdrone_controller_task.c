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

void sdrone_controller_init(sdrone_state_handle_t sdrone_state_handle) {
	printf("sdrone_controller_init initial state\n");
	memset(sdrone_state_handle, 0, sizeof(*sdrone_state_handle));
	sdrone_state_handle->driver_id = (uint32_t) SDRONE_CONTROLLER_DRIVER_ID;

	// rotational
	sdrone_state_handle->controller_state[X_POS].pid_alfa.ke = SDRONE_KE_ROLL;
	sdrone_state_handle->controller_state[X_POS].pid_alfa.ki = SDRONE_KI_ROLL;
	sdrone_state_handle->controller_state[Y_POS].pid_alfa.ke = SDRONE_KE_PITCH;
	sdrone_state_handle->controller_state[Y_POS].pid_alfa.ki = SDRONE_KI_PITCH;
	sdrone_state_handle->controller_state[Z_POS].pid_alfa.ke = SDRONE_KE_Z;
	sdrone_state_handle->controller_state[Z_POS].pid_alfa.ki = SDRONE_KI_Z;

	// linear
	sdrone_state_handle->controller_state[X_POS].pid_acc.ke = 0.0f;
	sdrone_state_handle->controller_state[X_POS].pid_acc.ki = 0.0f;
	sdrone_state_handle->controller_state[Y_POS].pid_acc.ke = 0.0f;
	sdrone_state_handle->controller_state[Y_POS].pid_acc.ki = 0.0f;
	sdrone_state_handle->controller_state[Z_POS].pid_acc.ke = SDRONE_KE_ACC_Z;
	sdrone_state_handle->controller_state[Z_POS].pid_acc.ki = SDRONE_KI_ACC_Z;
}

uint16_t counter = 0;
void sdrone_controller_print_data(sdrone_state_handle_t sdrone_state_handle) {
	counter++;
	counter %= 500;
	if (counter == 0) {
//		printf("RC: [%d,%d,%d,%d,%d,%d]\n",
//				sdrone_state_handle->rc_state.rc_data.data.norm[RC_ROLL],
//				sdrone_state_handle->rc_state.rc_data.data.norm[RC_PITCH],
//				sdrone_state_handle->rc_state.rc_data.data.norm[RC_YAW],
//				sdrone_state_handle->rc_state.rc_data.data.norm[RC_THROTTLE],
//				sdrone_state_handle->rc_state.rc_data.data.norm[RC_SWA],
//				sdrone_state_handle->rc_state.rc_data.data.norm[RC_SWB]);
//		printf("U: [%5.5f,%5.5f,%5.5f,%5.5f]\n",
//				sdrone_state_handle->controller_state[0].U[SDRONE_TETA_POS],
//				sdrone_state_handle->controller_state[1].U[SDRONE_TETA_POS],
//				sdrone_state_handle->controller_state[2].U[SDRONE_TETA_POS],
//				sdrone_state_handle->controller_state[2].U[SDRONE_THRUST_POS]);
////		printf("W: [%5.5f,%5.5f,%5.5f,%5.5f]\n",
////				sdrone_state_handle->controller_state[0].W[SDRONE_TETA_POS],
////				sdrone_state_handle->controller_state[1].W[SDRONE_TETA_POS],
////				sdrone_state_handle->controller_state[2].W[SDRONE_TETA_POS],
////				sdrone_state_handle->controller_state[2].W[SDRONE_THRUST_POS]);
//		printf("RPY: [%5.5f,%5.5f,%5.5f]\n",
//				sdrone_state_handle->imu_state.imu.data.gyro.rpy.xyz.x,
//				sdrone_state_handle->imu_state.imu.data.gyro.rpy.xyz.y,
//				sdrone_state_handle->imu_state.imu.data.gyro.rpy.xyz.z);

		printf("ACC: [%5.5f,%5.5f,%5.5f]\n",
				sdrone_state_handle->imu_state.imu.data.accel.mss.array[X_POS],
				sdrone_state_handle->imu_state.imu.data.accel.mss.array[Y_POS],
				sdrone_state_handle->imu_state.imu.data.accel.mss.array[Z_POS]);
		printf("THR: [%d] p=[%5.5f],e=[%5.5f],v=[%5.5f]\n",
				sdrone_state_handle->state,
				sdrone_state_handle->controller_state[Z_POS].dynamic.predX[SDRONE_X_ACC_POS],
				sdrone_state_handle->controller_state[Z_POS].pid_acc.err,
				sdrone_state_handle->imu_state.imu.data.vertical_v);
		printf("DUT: [%5.5f, %5.5f, %5.5f, %5.5f]\n",
				sdrone_state_handle->motors_state.motors.motor[0].duty_cycle,
				sdrone_state_handle->motors_state.motors.motor[1].duty_cycle,
				sdrone_state_handle->motors_state.motors.motor[2].duty_cycle,
				sdrone_state_handle->motors_state.motors.motor[3].duty_cycle);

//		printf("ATT: [%5.5f,%5.5f,%5.5f]\n",
//				sdrone_state_handle->imu_state.imu.data.attitude[X_POS],
//				sdrone_state_handle->imu_state.imu.data.attitude[Y_POS],
//				sdrone_state_handle->imu_state.imu.data.attitude[Z_POS]);
	}
}

bool sdrone_motors_are_armed(sdrone_state_handle_t sdrone_state_handle) {
	return (sdrone_state_handle->motors_state.motors.status == MOTORS_ARMED);
}

bool sdrone_motors_are_disarmed(sdrone_state_handle_t sdrone_state_handle) {
	return (sdrone_state_handle->motors_state.motors.status == MOTORS_DISARMED);
}

void sdrone_update_X_from_IMU(sdrone_state_handle_t sdrone_state_handle) {
	// Update X from IMU
	for (uint8_t i = 0; i < 3; i++) {
		// rotational in body frame
		sdrone_state_handle->controller_state[i].dynamic.X[SDRONE_X_TETA_POS] =
				sdrone_state_handle->imu_state.imu.data.gyro.rpy.array[i];
		sdrone_state_handle->controller_state[i].dynamic.X[SDRONE_X_OMEGA_POS] =
				(double) sdrone_state_handle->imu_state.imu.data.gyro.cal.kalman[i].X
						/ (double) sdrone_state_handle->imu_state.imu.data.gyro.lsb
						/ (double) 360.0f * (double) PI_2;
		sdrone_state_handle->controller_state[i].dynamic.X[SDRONE_X_ALFA_POS] = sdrone_state_handle->imu_state.imu.data.gyro.alfa[i];

		// linear in body frame
		sdrone_state_handle->controller_state[i].dynamic.X[SDRONE_X_SPEED_POS] = sdrone_state_handle->imu_state.imu.data.speed_bf[i];
		sdrone_state_handle->controller_state[i].dynamic.X[SDRONE_X_ACC_POS] = sdrone_state_handle->imu_state.imu.data.accel.mss.array[i];
	}
}
void sdrone_update_U_from_RC_RPY(sdrone_state_handle_t sdrone_state_handle) {
	sdrone_state_handle->controller_state[X_POS].dynamic.U[SDRONE_UW_TETA_POS] =
			sdrone_state_handle->rc_state.rc_data.data.norm[RC_ROLL] * SDRONE_NORM_ROLL_TO_RADIANS_FACTOR;

	sdrone_state_handle->controller_state[Y_POS].dynamic.U[SDRONE_UW_TETA_POS] =
			sdrone_state_handle->rc_state.rc_data.data.norm[RC_PITCH] * SDRONE_NORM_PITCH_TO_RADIANS_FACTOR;

	if (abs(sdrone_state_handle->rc_state.rc_data.data.norm[RC_YAW]) > 10) {
		sdrone_state_handle->imu_state.imu.data.yaw_reference = sdrone_state_handle->controller_state[Z_POS].dynamic.X[SDRONE_UW_TETA_POS] +
				sdrone_state_handle->rc_state.rc_data.data.norm[RC_YAW] * SDRONE_NORM_YAW_TO_RADIANS_FACTOR;
	}
	sdrone_state_handle->controller_state[Z_POS].dynamic.U[SDRONE_UW_TETA_POS] = sdrone_state_handle->imu_state.imu.data.yaw_reference;
}

void sdrone_update_U_from_RC(sdrone_state_handle_t sdrone_state_handle) {
	switch(sdrone_state_handle->state) {
	case SDRONE_OFF: {
		for(uint8_t i = 0; i < 3; i++) {
			sdrone_state_handle->controller_state[i].dynamic.U[SDRONE_UW_TETA_POS] = 0.0f;
			sdrone_state_handle->controller_state[i].dynamic.U[SDRONE_UW_ACC_POS] = 0.0f;
		}
		sdrone_state_handle->controller_state[Z_POS].dynamic.U[SDRONE_UW_ACC_POS] =  0.0f;
		break;
	}
	case SDRONE_ON: {
		for(uint8_t i = 0; i < 3; i++) {
			sdrone_state_handle->controller_state[i].dynamic.U[SDRONE_UW_TETA_POS] = 0.0f;
			sdrone_state_handle->controller_state[i].dynamic.U[SDRONE_UW_ACC_POS] = 0.0f;
		}
		sdrone_state_handle->controller_state[Z_POS].dynamic.U[SDRONE_UW_ACC_POS] =  0.667f*NUM_MOTORS;
		break;
	}
	case SDRONE_TAKE_OFF: {
		for(uint8_t i = 0; i < 3; i++) {
			sdrone_state_handle->controller_state[i].dynamic.U[SDRONE_UW_TETA_POS] = 0.0f;
			sdrone_state_handle->controller_state[i].dynamic.U[SDRONE_UW_ACC_POS] = 0.0f;
		}
		sdrone_state_handle->controller_state[Z_POS].dynamic.U[SDRONE_UW_ACC_POS] =  SDRONE_GRAVITY_ACCELERATION + 0.25f;
		break;
	}
	case SDRONE_TAKEN_OFF: {
		sdrone_update_U_from_RC_RPY(sdrone_state_handle);
		// FIXME: calc ACC for X and Y axis from requested ROLL,PITCH,YAW and g (the result can be like g{bf} x and y coordinates with some contribution from yaw acceleration)
		if(sdrone_state_handle->rc_state.rc_data.data.norm[RC_THROTTLE] < -SDRONE_NORM_THROTTLE_ZERO_RADIUS) {
			sdrone_state_handle->controller_state[Z_POS].dynamic.U[SDRONE_UW_ACC_POS] = SDRONE_GRAVITY_ACCELERATION + (sdrone_state_handle->rc_state.rc_data.data.norm[RC_THROTTLE]+SDRONE_NORM_THROTTLE_ZERO_RADIUS)*SDRONE_NORM_THROTTLE_TO_ACCEL_FACTOR;
		} else if(sdrone_state_handle->rc_state.rc_data.data.norm[RC_THROTTLE] > SDRONE_NORM_THROTTLE_ZERO_RADIUS) {
			sdrone_state_handle->controller_state[Z_POS].dynamic.U[SDRONE_UW_ACC_POS] = SDRONE_GRAVITY_ACCELERATION + (sdrone_state_handle->rc_state.rc_data.data.norm[RC_THROTTLE]-SDRONE_NORM_THROTTLE_ZERO_RADIUS)*SDRONE_NORM_THROTTLE_TO_ACCEL_FACTOR;
		} else {
			sdrone_state_handle->controller_state[Z_POS].dynamic.U[SDRONE_UW_ACC_POS] = SDRONE_GRAVITY_ACCELERATION;
		}
		break;
	}
	case SDRONE_LANDING: {
		sdrone_update_U_from_RC_RPY(sdrone_state_handle);
		for(uint8_t i = 0; i < 3; i++) {
			sdrone_state_handle->controller_state[i].dynamic.U[SDRONE_UW_TETA_POS] = 0.0f;
			sdrone_state_handle->controller_state[i].dynamic.U[SDRONE_UW_ACC_POS] = 0.0f;
		}
		sdrone_state_handle->controller_state[Z_POS].dynamic.U[SDRONE_UW_ACC_POS] = SDRONE_GRAVITY_ACCELERATION-0.25f;
		break;
	}
	case SDRONE_LANDED: {
		for(uint8_t i = 0; i < 3; i++) {
			sdrone_state_handle->controller_state[i].dynamic.U[SDRONE_UW_TETA_POS] = 0.0f;
			sdrone_state_handle->controller_state[i].dynamic.U[SDRONE_UW_ACC_POS] = 0.0f;
		}
		sdrone_state_handle->controller_state[Z_POS].dynamic.U[SDRONE_UW_ACC_POS] = 0.0f;
		break;
	}
	}

}

void sdrone_update_error(sdrone_state_handle_t sdrone_state_handle) {
	// Calc error (predX(prev) - X(actual))
	if (sdrone_state_handle->state == SDRONE_TAKE_OFF || sdrone_state_handle->state == SDRONE_TAKEN_OFF || sdrone_state_handle->state == SDRONE_LANDING) {
		for (uint8_t i = 0; i < 3; i++) {
			sdrone_state_handle->controller_state[i].pid_alfa.err =
					sdrone_state_handle->controller_state[i].dynamic.predX[SDRONE_X_ALFA_POS]
							- sdrone_state_handle->controller_state[i].dynamic.X[SDRONE_X_ALFA_POS];
			sdrone_state_handle->controller_state[i].pid_acc.err =
					sdrone_state_handle->controller_state[i].dynamic.predX[SDRONE_X_ACC_POS]
							- sdrone_state_handle->controller_state[i].dynamic.X[SDRONE_X_ACC_POS];
			sdrone_state_handle->controller_state[i].pid_alfa.ierr +=
					sdrone_state_handle->controller_state[i].pid_alfa.err;
			sdrone_state_handle->controller_state[i].pid_acc.ierr +=
					sdrone_state_handle->controller_state[i].pid_acc.err;

			// limit acc error
			if(sdrone_state_handle->controller_state[i].pid_acc.ierr > 10.0f) {
				sdrone_state_handle->controller_state[i].pid_acc.ierr = 10.0f;
			} else if(sdrone_state_handle->controller_state[i].pid_acc.ierr < -10.0f) {
				sdrone_state_handle->controller_state[i].pid_acc.ierr = -10.0f;
			}
		}
	} else {
		for (uint8_t i = 0; i < 3; i++) {
			sdrone_state_handle->controller_state[i].pid_alfa.err = 0.0f;
			sdrone_state_handle->controller_state[i].pid_alfa.ierr = 0.0f;
			sdrone_state_handle->controller_state[i].pid_acc.err = 0.0f;
			sdrone_state_handle->controller_state[i].pid_acc.ierr = 0.0f;
		}
	}
}

void sdrone_update_W_from_U_and_X(sdrone_state_handle_t sdrone_state_handle) {
	// Update W from U and X
	for (uint8_t i = 0; i < 3; i++) {
		sdrone_state_handle->controller_state[i].dynamic.W[SDRONE_X_TETA_POS] =
				(sdrone_state_handle->controller_state[i].dynamic.U[SDRONE_X_TETA_POS]
						- sdrone_state_handle->controller_state[i].dynamic.X[SDRONE_X_TETA_POS]);

		sdrone_state_handle->controller_state[i].dynamic.W[SDRONE_UW_ACC_POS] =
				(sdrone_state_handle->controller_state[i].dynamic.U[SDRONE_UW_ACC_POS] - sdrone_state_handle->controller_state[i].dynamic.X[SDRONE_X_ACC_POS]);

		// limit W
		// TODO: configurare per ogni asse max_w e togliere i define
		if(i != Z_POS) {
			if (sdrone_state_handle->controller_state[i].dynamic.W[SDRONE_X_TETA_POS]
					>= 0.0f&& sdrone_state_handle->controller_state[i].dynamic.W[SDRONE_X_TETA_POS] > SDRONE_MAX_W) {
				sdrone_state_handle->controller_state[i].dynamic.W[SDRONE_X_TETA_POS] =
						SDRONE_MAX_W;
			} else if (sdrone_state_handle->controller_state[i].dynamic.W[SDRONE_X_TETA_POS]
					< 0.0f&& sdrone_state_handle->controller_state[i].dynamic.W[SDRONE_X_TETA_POS] < -SDRONE_MAX_W) {
				sdrone_state_handle->controller_state[i].dynamic.W[SDRONE_X_TETA_POS] =
						-SDRONE_MAX_W;
			}
		} else {
			if (sdrone_state_handle->controller_state[i].dynamic.W[SDRONE_X_TETA_POS]
					>= 0.0f&& sdrone_state_handle->controller_state[i].dynamic.W[SDRONE_X_TETA_POS] > SDRONE_MAX_W_Z) {
				sdrone_state_handle->controller_state[i].dynamic.W[SDRONE_X_TETA_POS] =
						SDRONE_MAX_W_Z;
			} else if (sdrone_state_handle->controller_state[i].dynamic.W[SDRONE_X_TETA_POS]
					< 0.0f&& sdrone_state_handle->controller_state[i].dynamic.W[SDRONE_X_TETA_POS] < -SDRONE_MAX_W_Z) {
				sdrone_state_handle->controller_state[i].dynamic.W[SDRONE_X_TETA_POS] =
						-SDRONE_MAX_W_Z;
			}
		}
	}
}
void sdrone_update_predX_from_W_and_X(sdrone_state_handle_t sdrone_state_handle) {
	// Calc new prediction
	for (uint8_t i = 0; i < 3; i++) {
		sdrone_state_handle->controller_state[i].dynamic.predX[SDRONE_X_ALFA_POS] =
				+(sdrone_state_handle->controller_state[i].dynamic.W[SDRONE_X_TETA_POS]
						/ SDRONE_CONTROLLER_REACTIVITY_DT
						- 1.5*sdrone_state_handle->controller_state[i].dynamic.X[SDRONE_X_OMEGA_POS])
						* SDRONE_REACTIVITY_FACTOR;
		sdrone_state_handle->controller_state[i].dynamic.predX[SDRONE_X_ACC_POS] = sdrone_state_handle->controller_state[i].dynamic.W[SDRONE_UW_ACC_POS] + sdrone_state_handle->controller_state[i].dynamic.X[SDRONE_X_ACC_POS];
	}
}

void sdrone_update_Y_from_predX(sdrone_state_handle_t sdrone_state_handle) {
	// controller response
	if(sdrone_state_handle->state == SDRONE_TAKE_OFF || sdrone_state_handle->state == SDRONE_TAKEN_OFF || sdrone_state_handle->state == SDRONE_LANDING) {
 	  for (uint8_t i = 0; i < 3; i++) {
  		sdrone_state_handle->controller_state[i].dynamic.Y[SDRONE_Y_TORQUE_POS] =
				(+SDRONE_AXIS_LENGTH
						* sdrone_state_handle->controller_state[i].dynamic.predX[SDRONE_X_ALFA_POS]
						+ sdrone_state_handle->controller_state[i].pid_alfa.err * sdrone_state_handle->controller_state[i].pid_alfa.ke
						+ sdrone_state_handle->controller_state[i].pid_alfa.ierr * sdrone_state_handle->controller_state[i].pid_alfa.ki
				);
	  }
 	  sdrone_state_handle->controller_state[Z_POS].dynamic.Y[SDRONE_Y_ACC_POS] =
 			  sdrone_state_handle->controller_state[Z_POS].dynamic.predX[SDRONE_X_ACC_POS]
 			  + sdrone_state_handle->controller_state[Z_POS].pid_acc.err * sdrone_state_handle->controller_state[Z_POS].pid_acc.ke
			  + sdrone_state_handle->controller_state[Z_POS].pid_acc.ierr * sdrone_state_handle->controller_state[Z_POS].pid_acc.ki;
	} else {
		for(uint8_t i = 0; i < 3; i++) {
		  sdrone_state_handle->controller_state[i].dynamic.Y[SDRONE_Y_TORQUE_POS] = 0.0f;
		}
	  sdrone_state_handle->controller_state[Z_POS].dynamic.Y[SDRONE_Y_ACC_POS] = sdrone_state_handle->controller_state[Z_POS].dynamic.predX[SDRONE_X_ACC_POS];
	}
}

void sdrone_update_motors_input_data_from_Y(
		sdrone_state_handle_t sdrone_state_handle) {

	for(uint8_t i = 0; i< 3; i++) {
		sdrone_state_handle->motors_state.input.data.at[i] = 0.0f;
	}

#ifdef SDRONE_ENABLE_ROLL
		sdrone_state_handle->motors_state.input.data.at[X_POS] = sdrone_state_handle->controller_state[X_POS].dynamic.Y[SDRONE_Y_TORQUE_POS];
#endif
#ifdef SDRONE_ENABLE_PITCH
		sdrone_state_handle->motors_state.input.data.at[Y_POS] = sdrone_state_handle->controller_state[Y_POS].dynamic.Y[SDRONE_Y_TORQUE_POS];
#endif
#ifdef SDRONE_ENABLE_YAW
		sdrone_state_handle->motors_state.input.data.at[Z_POS] = sdrone_state_handle->controller_state[Z_POS].dynamic.Y[SDRONE_Y_TORQUE_POS];
#endif
	sdrone_state_handle->motors_state.input.data.thrust = sdrone_state_handle->controller_state[Z_POS].dynamic.Y[SDRONE_Y_ACC_POS];
}

esp_err_t sdrone_controller_control(sdrone_state_handle_t sdrone_state_handle) {
	// Update X from IMU
	sdrone_update_X_from_IMU(sdrone_state_handle);

	// Update U from RC
	sdrone_update_U_from_RC(sdrone_state_handle);

	// Calc error (predX(prev) - X(actual))
	sdrone_update_error(sdrone_state_handle);

	// Update W from U and X
	sdrone_update_W_from_U_and_X(sdrone_state_handle);

	// Calc new prediction
	sdrone_update_predX_from_W_and_X(sdrone_state_handle);

	// controller response
	sdrone_update_Y_from_predX(sdrone_state_handle);

	// convert response to motors thrust
	sdrone_update_motors_input_data_from_Y(sdrone_state_handle);
	return ESP_OK;
}

esp_err_t sdrone_send_motors_notification(sdrone_state_handle_t sdrone_state_handle) {
	sdrone_state_handle->motors_state.input.data.tx_rx_flag =
			MOTORS_TXRX_TRANSMITTED;
	if (sdrone_state_handle->motors_state.motors_task_handle
			!= NULL) {
		xTaskNotify(
				sdrone_state_handle->motors_state.motors_task_handle,
				sdrone_state_handle->driver_id,
				eSetValueWithOverwrite);
	}
	return ESP_OK;
}

esp_err_t sdrone_disarm_motors(sdrone_state_handle_t sdrone_state_handle) {
	sdrone_state_handle->motors_state.input.data.thrust = 0.0f;
	sdrone_state_handle->motors_state.input.isCommand = true;
	sdrone_state_handle->motors_state.input.command.type = MOTORS_DISARM;
	ESP_ERROR_CHECK(sdrone_send_motors_notification(sdrone_state_handle));
	return ESP_OK;
}

esp_err_t sdrone_arm_motors(sdrone_state_handle_t sdrone_state_handle) {
	sdrone_state_handle->motors_state.input.data.thrust = 0.0f;
	sdrone_state_handle->motors_state.input.isCommand = true;
	sdrone_state_handle->motors_state.input.command.type = MOTORS_ARM;
	ESP_ERROR_CHECK(sdrone_send_motors_notification(sdrone_state_handle));
	return ESP_OK;
}
esp_err_t sdrone_eval_rc_gesture(sdrone_state_handle_t sdrone_state_handle) {
	// connection control
	if(sdrone_state_handle->rc_state.rc_data.state == RC_NOT_CONNECTED) {
		sdrone_state_handle->state = SDRONE_OFF;
		ESP_ERROR_CHECK(sdrone_disarm_motors(sdrone_state_handle));
		return ESP_OK;
	}
	if((sdrone_state_handle->state == SDRONE_OFF) && (sdrone_state_handle->rc_state.rc_data.data.norm[RC_SWA] >= SDRONE_RC_CHANNEL_NORM_MAX - 20)) {
		sdrone_state_handle->state = SDRONE_ON;
		if(sdrone_motors_are_disarmed(sdrone_state_handle)) {
			ESP_ERROR_CHECK(sdrone_arm_motors(sdrone_state_handle));
		}
		return ESP_OK;
	} else if(sdrone_state_handle->rc_state.rc_data.data.norm[RC_SWA] <= SDRONE_RC_CHANNEL_NORM_MIN + 20) {
		sdrone_state_handle->state = SDRONE_OFF;
		if(sdrone_motors_are_armed(sdrone_state_handle)) {
			ESP_ERROR_CHECK(sdrone_disarm_motors(sdrone_state_handle));
		}
		return ESP_OK;
	}

	if((sdrone_state_handle->state == SDRONE_ON) && (sdrone_state_handle->rc_state.rc_data.data.norm[RC_THROTTLE] >= SDRONE_NORM_THROTTLE_ZERO_RADIUS)) {
		sdrone_state_handle->state = SDRONE_TAKE_OFF;
		return ESP_OK;
	}

	if(sdrone_state_handle->state == SDRONE_TAKE_OFF && (sdrone_state_handle->imu_state.imu.data.vertical_v > 0.1f)) {
		sdrone_state_handle->state = SDRONE_TAKEN_OFF;
		return ESP_OK;
	}

	if((sdrone_state_handle->state == SDRONE_TAKEN_OFF) && (sdrone_state_handle->rc_state.rc_data.data.norm[RC_SWD] >= SDRONE_RC_CHANNEL_NORM_MAX - 20)) {
		sdrone_state_handle->state = SDRONE_LANDING;
		return ESP_OK;
	}
	if((sdrone_state_handle->state == SDRONE_LANDING) && (sdrone_state_handle->rc_state.rc_data.data.norm[RC_SWD] <= SDRONE_RC_CHANNEL_NORM_MIN + 20)) {
		sdrone_state_handle->state = SDRONE_TAKEN_OFF;
		return ESP_OK;
	}
	if((sdrone_state_handle->state == SDRONE_LANDING) && (sdrone_state_handle->imu_state.imu.data.vertical_v <= 0.001f)) {
		sdrone_state_handle->state = SDRONE_OFF;
		return ESP_OK;
	}
	return ESP_OK;
}

void sdrone_controller_cycle(sdrone_state_handle_t sdrone_state_handle) {
	rc_data_t rc_data;
	mpu9250_data_t imu_data;

	// wait for start signal (from main)
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

	int16_t skip_counter = SDRONE_IMU_INIT_CYCLES;
	while (true) {
		if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100)) != 0) {
			// RC
			if (sdrone_state_handle->rc_state.rc_data.data.txrx_signal
					== RC_TXRX_TRANSMITTED) {
				memcpy(&rc_data, &sdrone_state_handle->rc_state.rc_data.data,
						sizeof(rc_data));
				sdrone_state_handle->rc_state.rc_data.data.txrx_signal =
						RC_TXRX_RECEIVED;
				ESP_ERROR_CHECK(sdrone_eval_rc_gesture(sdrone_state_handle));
			}

			// IMU
			if (sdrone_state_handle->imu_state.imu.data.txrx_signal
					== IMU_TXRX_TRANSMITTED) {
				memcpy(&imu_data, &sdrone_state_handle->imu_state.imu.data,
						sizeof(imu_data));
				sdrone_state_handle->imu_state.imu.data.txrx_signal =
						IMU_TXRX_RECEIVED;

				if (skip_counter <= 0) {
					ESP_ERROR_CHECK(sdrone_controller_control(sdrone_state_handle));
					sdrone_state_handle->motors_state.input.isCommand = false;
					ESP_ERROR_CHECK(sdrone_send_motors_notification(sdrone_state_handle));
#ifdef SDRONE_DEBUG_LOG
					sdrone_controller_print_data(sdrone_state_handle);
#endif
				} else {
					skip_counter--;
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
