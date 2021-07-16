/*
 * sdrone_motors_task.h
 *
 *  Created on: 19 apr 2021
 *      Author: andrea
 */

#ifndef MAIN_SDRONE_MOTORS_TASK_H_
#define MAIN_SDRONE_MOTORS_TASK_H_
#include <freertos/semphr.h>
#include <motors.h>
#include <ina3221.h>

#define MOTORS_INA3221_I2C_PORT 1

typedef enum {
	MOTORS_TXRX_IGNORE = 0,
	MOTORS_TXRX_TRANSMITTED = 1,
	MOTORS_TXRX_RECEIVED = 2
} sdrone_motors_tx_rx_mask_t;
typedef enum {
	MOTORS_ARM = 1,
	MOTORS_DISARM,
	MOTORS_SWITCHOFF,
	MOTORS_SWITCHON
} sdrone_motors_commands_t;

typedef struct {
	float desired_omega;
	float actual_omega;
	float thrust[MOTORS_MAX_NUM];
	motors_status_t desired_motors_status;
	volatile sdrone_motors_tx_rx_mask_t tx_rx_flag;
} sdrone_motors_input_data_t;
typedef struct {
	sdrone_motors_commands_t type;
} sdrone_motors_command_t;

typedef struct {
	uint8_t isCommand;
	union {
		sdrone_motors_input_data_t data;
		sdrone_motors_command_t command;
	};
} sdrone_motors_input_t;

typedef sdrone_motors_input_t* sdrone_motors_input_handle_t;

typedef struct {
	float kp;
	float ki;
	float kd;
} sdrone_motors_pid_factors_t;

typedef struct {
	uint32_t driver_id;
	uint32_t controller_driver_id;
	motors_t motors;
	sdrone_motors_input_t input;
	TaskHandle_t controller_task_handle;
	TaskHandle_t motors_task_handle;
	ina3221_handle_t ina3221_handle;
} sdrone_motors_state_t;

typedef sdrone_motors_state_t* sdrone_motors_state_handle_t;

void sdrone_motors_task(void *arg);

#endif /* MAIN_SDRONE_MOTORS_TASK_H_ */
