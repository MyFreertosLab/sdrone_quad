/*
 * sdrone_telemetry_task.h
 *
 *  Created on: 19 ott 2021
 *      Author: andrea
 */

#ifndef MAIN_SDRONE_TELEMETRY_TASK_H_
#define MAIN_SDRONE_TELEMETRY_TASK_H_

#include <telemetry.h>
#include <sdrone_controller_task.h>

typedef struct {
	uint32_t driver_id;
	uint32_t controller_driver_id;
	TaskHandle_t controller_task_handle;
	TaskHandle_t telemetry_task_handle;
	sdrone_state_handle_t sdrone_state_handle;
	sdrone_telemetry_state_t telemetry_state;
} sdrone_telemetry_task_state_t;

typedef sdrone_telemetry_task_state_t* sdrone_telemetry_task_state_handle_t;

void sdrone_telemetry_task(void *arg);

#endif /* MAIN_SDRONE_TELEMETRY_TASK_H_ */
