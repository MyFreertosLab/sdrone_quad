/*
 * sdrone_controller_task.h
 *
 *  Created on: 18 mag 2021
 *      Author: andrea
 */

#ifndef MAIN_SDRONE_CONTROLLER_TASK_H_
#define MAIN_SDRONE_CONTROLLER_TASK_H_

#include <sdrone_motors_task.h>
#include <sdrone_rc_task.h>
#include <sdrone_imu_task.h>

#define SDRONE_MAX_ROLL_RADIANS  1.2217f // 70deg
#define SDRONE_MAX_PITCH_RADIANS  1.2217f // 70deg
#define SDRONE_MAX_YAW_RADIANS  1.570796327f // 90deg
#define SDRONE_NORM_THROTTLE_TO_ACCEL_FACTOR ((float)MOTORS_ACCEL_RANGE/(float)SDRONE_RC_CHANNEL_RANGE)
#define SDRONE_NORM_ROLL_TO_RADIANS_FACTOR ((float)2.0*SDRONE_MAX_ROLL_RADIANS/(float)SDRONE_RC_CHANNEL_RANGE)
#define SDRONE_NORM_PITCH_TO_RADIANS_FACTOR ((float)2.0*SDRONE_MAX_PITCH_RADIANS/(float)SDRONE_RC_CHANNEL_RANGE)
#define SDRONE_NORM_YAW_TO_RADIANS_FACTOR ((float)2.0*SDRONE_MAX_YAW_RADIANS/(float)SDRONE_RC_CHANNEL_RANGE)

#define SDRONE_CONTROLLER_FREQ 100.0f
#define SDRONE_CONTROLLER_DT (1.0/SDRONE_CONTROLLER_FREQ)
#define SDRONE_CONTROLLER_DT2 (SDRONE_CONTROLLER_DT*SDRONE_CONTROLLER_DT)
#define SDRONE_CONTROLLER_REACTIVITY_DT (SDRONE_CONTROLLER_DT*10.0f)
#define SDRONE_MAX_RADIANS_PER_SECOND 5.0f
#define SDRONE_MAX_RADIANS_PER_SECOND_Z 2.0f
#define SDRONE_MAX_W SDRONE_MAX_RADIANS_PER_SECOND*SDRONE_CONTROLLER_REACTIVITY_DT
#define SDRONE_MAX_W_Z SDRONE_MAX_RADIANS_PER_SECOND_Z*SDRONE_CONTROLLER_REACTIVITY_DT
#define SDRONE_REACTIVITY_FACTOR 1.5f

#define SDRONE_KE_ROLL  0.4f
#define SDRONE_KI_ROLL  0.002f
#define SDRONE_KE_PITCH 0.4f // 0.7f
#define SDRONE_KI_PITCH 0.002f // 0.008f
#define SDRONE_KE_Z     0.4f // 0.4f
#define SDRONE_KI_Z     0.002f // 0.002f

#define SDRONE_AXIS_LENGTH 0.20f
#define SDRONE_MASS 1.28f

#define SDRONE_TETA_POS      0
#define SDRONE_OMEGA_POS     1
#define SDRONE_ALFA_POS      2
#define SDRONE_UW_THRUST_POS 1
#define SDRONE_X_THRUST_POS  3
#define SDRONE_Y_TORQUE_POS  0
#define SDRONE_Y_THRUST_POS  1

#define SDRONE_AXIS_X_POS    0
#define SDRONE_AXIS_Y_POS    1
#define SDRONE_AXIS_Z_POS    2

#define SDRONE_ENABLE_ROLL
#define SDRONE_ENABLE_PITCH
#define SDRONE_ENABLE_YAW

#define MOTORS_FRAME_X_QUADCOPTER
//#define MOTORS_FRAME_ONE_HORIZONTAL_AXIS
#ifdef MOTORS_FRAME_X_QUADCOPTER
#define SDRONE_NUM_MOTORS 4
#else
#ifdef MOTORS_FRAME_HORIZONTAL_HEXACOPTER
#define SDRONE_NUM_MOTORS 6
#else
#ifdef MOTORS_FRAME_ONE_HORIZONTAL_AXIS
#define SDRONE_NUM_MOTORS 2
#endif
#endif
#endif


typedef enum {
	SDRONE_MOTORS_DRIVER_ID = 1,
	SDRONE_RC_DRIVER_ID,
	SDRONE_IMU_DRIVER_ID,
	SDRONE_CONTROLLER_DRIVER_ID,
	SDRONE_TELEMETRY_DRIVER_ID
} sdrone_drivers_id;
typedef struct {
	float X[4]; // [teta, omega, alfa, thrust] (radians, newton)
	float U[2]; // [teta, thrust] (radians, newton)
	float W[2]; // [dteta,dtrust] (radians, newton)
	float Y[2]; // [torque, thrust](newton)
	float err;
	float ierr;
	float err_thrust;
	float predX[4]; // [teta, omega, alfa, thrust] (radians, newton)
	float ke;
	float ki;
	float prevErr; // previous error
} sdrone_controller_t;

typedef struct {
	sdrone_motors_state_t motors_state;
	sdrone_rc_state_t rc_state;
	sdrone_imu_state_t imu_state;
	uint32_t driver_id;
	sdrone_controller_t controller_state[3]; // one for each axis x,y,z
} sdrone_state_t;

typedef sdrone_state_t* sdrone_state_handle_t;

void sdrone_controller_task(void *arg);

#endif /* MAIN_SDRONE_CONTROLLER_TASK_H_ */
