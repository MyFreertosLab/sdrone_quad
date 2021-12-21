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

//#define SDRONE_DEBUG_LOG
//#define SDRONE_DISABLE_ACC
//#define SDRONE_DISABLE_ROLL
//#define SDRONE_DISABLE_PITCH
//#define SDRONE_DISABLE_YAW
//#define SDRONE_ENABLE_TUNING_ACC
//#define SDRONE_ENABLE_TUNING_ACC_Z


#define SDRONE_MAX_ROLL_RADIANS  1.2217f // 70deg
#define SDRONE_MAX_PITCH_RADIANS  1.2217f // 70deg

#define SDRONE_ACCEL_MAX (3.667f*NUM_MOTORS)
#define SDRONE_ACCEL_MIN (2.0*SDRONE_GRAVITY_ACCELERATION-SDRONE_ACCEL_MAX)
#define SDRONE_ACCEL_RANGE (SDRONE_ACCEL_MAX - SDRONE_ACCEL_MIN)

#define SDRONE_NORM_THROTTLE_ZERO_RADIUS 50
#define SDRONE_NORM_THROTTLE_TO_ACCEL_FACTOR (((float)SDRONE_ACCEL_RANGE)/(float)(SDRONE_RC_CHANNEL_RANGE - SDRONE_NORM_THROTTLE_ZERO_RADIUS*2))
#define SDRONE_NORM_ROLL_TO_ACCEL_FACTOR ((float)SDRONE_ACCEL_RANGE/(float)SDRONE_RC_CHANNEL_RANGE)
#define SDRONE_NORM_PITCH_TO_ACCEL_FACTOR ((float)SDRONE_ACCEL_RANGE/(float)SDRONE_RC_CHANNEL_RANGE)

#define SDRONE_ACC_KE_MAX 2.0f
#define SDRONE_ACC_KE_MIN 0.0f
#define SDRONE_ACC_KI_MAX 0.1f
#define SDRONE_ACC_KI_MIN 0.0f
#define SDRONE_ACC_KE_RANGE  (SDRONE_ACC_KE_MAX - SDRONE_ACC_KE_MIN)
#define SDRONE_ACC_KI_RANGE  (SDRONE_ACC_KI_MAX - SDRONE_ACC_KI_MIN)
#define SDRONE_NORM_ACC_KE_FACTOR (SDRONE_ACC_KE_RANGE/(float)SDRONE_RC_CHANNEL_RANGE)
#define SDRONE_NORM_ACC_KI_FACTOR ((SDRONE_ACC_KI_RANGE)/(float)(SDRONE_RC_CHANNEL_RANGE))

#define SDRONE_ACC_KE_Z_MAX 2.0f
#define SDRONE_ACC_KE_Z_MIN 0.0f
#define SDRONE_ACC_KI_Z_MAX 0.1f
#define SDRONE_ACC_KI_Z_MIN 0.0f
#define SDRONE_ACC_KE_Z_RANGE  (SDRONE_ACC_KE_Z_MAX - SDRONE_ACC_KE_Z_MIN)
#define SDRONE_ACC_KI_Z_RANGE  (SDRONE_ACC_KI_Z_MAX - SDRONE_ACC_KI_Z_MIN)
#define SDRONE_NORM_ACC_KE_Z_FACTOR (SDRONE_ACC_KE_Z_RANGE/(float)SDRONE_RC_CHANNEL_RANGE)
#define SDRONE_NORM_ACC_KI_Z_FACTOR ((SDRONE_ACC_KI_Z_RANGE)/(float)(SDRONE_RC_CHANNEL_RANGE))

#define SDRONE_CONTROLLER_FREQ 100.0f
#define SDRONE_CONTROLLER_DT (1.0/SDRONE_CONTROLLER_FREQ)
#define SDRONE_CONTROLLER_DT2 (SDRONE_CONTROLLER_DT*SDRONE_CONTROLLER_DT)

// FIXME: togliere
#define SDRONE_NORM_ROLL_TO_RADIANS_FACTOR ((float)2.0*SDRONE_MAX_ROLL_RADIANS/(float)SDRONE_RC_CHANNEL_RANGE)
#define SDRONE_NORM_PITCH_TO_RADIANS_FACTOR ((float)2.0*SDRONE_MAX_PITCH_RADIANS/(float)SDRONE_RC_CHANNEL_RANGE)

// FIXME: togliere
#define SDRONE_CONTROLLER_REACTIVITY_DT (SDRONE_CONTROLLER_DT*10.0f)
#define SDRONE_REACTIVITY_FACTOR 1.5f

#define SDRONE_MAX_RADIANS_PER_SECOND 5.0f
#define SDRONE_MAX_W SDRONE_MAX_RADIANS_PER_SECOND*SDRONE_CONTROLLER_REACTIVITY_DT

#define SDRONE_MAX_YAW_RADIANS_PER_SECOND  0.79f // 45deg/s
#define SDRONE_NORM_YAW_TO_RADIANS_FACTOR ((float)2.0*SDRONE_MAX_YAW_RADIANS_PER_SECOND/(float)SDRONE_RC_CHANNEL_RANGE)
#define SDRONE_MAX_W_Z SDRONE_MAX_RADIANS_PER_SECOND*SDRONE_CONTROLLER_REACTIVITY_DT

#define SDRONE_LANDING_SPEED 0.15f

#define SDRONE_KE_ROLL  0.9f//0.7f
#define SDRONE_KI_ROLL  0.004f
#define SDRONE_KE_PITCH 0.7f//0.7f
#define SDRONE_KI_PITCH 0.004f
#define SDRONE_KE_Z     0.4f
#define SDRONE_KI_Z     0.002f

#define SDRONE_KE_ACC_Z     0.68f
#define SDRONE_KI_ACC_Z     0.004f
#define SDRONE_KE_ACC       0.0f
#define SDRONE_KI_ACC       0.0f

#define SDRONE_AXIS_LENGTH 0.20f
#define SDRONE_MASS 1.2f

#define SDRONE_X_TETA_POS    0
#define SDRONE_X_OMEGA_POS   1
#define SDRONE_X_ALFA_POS    2
#define SDRONE_X_SPEED_POS   3
#define SDRONE_X_ACC_POS     4

#define SDRONE_UW_TETA_POS   0
#define SDRONE_UW_ACC_POS    1

#define SDRONE_Y_TORQUE_POS  0
#define SDRONE_Y_ACC_POS     1

#define SDRONE_AXIS_X_POS    0
#define SDRONE_AXIS_Y_POS    1
#define SDRONE_AXIS_Z_POS    2

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

typedef enum {
	SDRONE_OFF = 0,
	SDRONE_ON,
	SDRONE_TAKE_OFF,
	SDRONE_TAKEN_OFF,
	SDRONE_LANDING,
	SDRONE_LANDED
} sdrone_state_enum;

typedef struct {
	float err;
	float ierr;
	float derr;
	float prevErr;
	float correction;
	float ke;
	float ki;
	float kd;
} sdrone_pid_t;

typedef struct {
	float predX[5]; // [teta, omega, alfa, speed_bf, acc_bf] (rad, rad/s, rad/s^2, m/s^2, m/s)
	float X[5]; // [teta, omega, alfa, speed_bf, acc_bf] (rad, rad/s, rad/s^2, m/s, m/s^2)
	float U[2]; // [teta, acc_bf] (rad, m/s^2)
	float W[2]; // [dteta,dacc_bf] (rad, m/s^2)
	float Y[2]; // [torque, acc_bf](m*rad/s^2,m/s^2) NOTA: Y è accelerazione richiesta ai motori (diverso da 0 solo per asse Z)
} sdrone_dynamic_state_t;

typedef struct {
	sdrone_dynamic_state_t dynamic;
	sdrone_pid_t pid_alfa;
	sdrone_pid_t pid_acc;
} sdrone_dynamic_t;

typedef struct {
	sdrone_state_enum state;
	sdrone_motors_state_t motors_state;
	sdrone_rc_state_t rc_state;
	sdrone_imu_state_t imu_state;
	uint32_t driver_id;
	sdrone_dynamic_t controller_state[3]; // one for each axis x,y,z
	mpu9250_cossin_t cossin_target;
	float yaw_reference;
} sdrone_state_t;

typedef sdrone_state_t* sdrone_state_handle_t;

void sdrone_controller_task(void *arg);

#endif /* MAIN_SDRONE_CONTROLLER_TASK_H_ */
