/*
 * sdrone_rc_task.h
 *
 *  Created on: 17 mag 2021
 *      Author: andrea
 */

#ifndef MAIN_SDRONE_RC_TASK_H_
#define MAIN_SDRONE_RC_TASK_H_
#include <freertos/semphr.h>
#include <rc.h>

#define SDRONE_RC_CHANNEL_RANGE 400
#define SDRONE_RC_CHANNEL_RANGE_HALF (SDRONE_RC_CHANNEL_RANGE/2)

typedef struct {
	uint16_t min;
	uint16_t center;
	uint16_t max;
} sdrone_rc_channels_range_t;

typedef struct {
	uint32_t driver_id;
	uint32_t controller_driver_id;
	rc_t rc_data;
	TaskHandle_t controller_task_handle;
	TaskHandle_t rc_task_handle;
} sdrone_rc_state_t;

typedef sdrone_rc_state_t* sdrone_rc_state_handle_t;

void sdrone_rc_task(void *arg);


#endif /* MAIN_SDRONE_RC_TASK_H_ */
