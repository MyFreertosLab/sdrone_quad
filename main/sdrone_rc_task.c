/*
 * sdrone_rc_task.c
 *
 *  Created on: 17 mag 2021
 *      Author: andrea
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <rc.h>
#include <sdrone_rc_task.h>

void sdrone_rc_task(void *arg) {
	sdrone_rc_state_handle_t sdrone_rc_state_handle =
			(sdrone_rc_state_handle_t) arg;
	rc_handle_t rc_handle = &(sdrone_rc_state_handle->rc_data);
	rc_t rc_data_local;
	rc_t* rc_data_local_handle = &rc_data_local;

	ESP_ERROR_CHECK(rc_init(rc_data_local_handle));
	sdrone_rc_state_handle->rc_task_handle = xTaskGetCurrentTaskHandle();
	memcpy(&sdrone_rc_state_handle->rc_data, rc_data_local_handle, sizeof(rc_t));

	float scale_factor_left[RC_MAX_CHANNELS];
	float scale_factor_right[RC_MAX_CHANNELS];

	for(uint8_t i = 0; i < RC_MAX_CHANNELS; i++) {
		if((rc_data_local_handle->rc_channels_range[i].center - rc_data_local_handle->rc_channels_range[i].min) != 0) {
			scale_factor_left[i] = ((float)SDRONE_RC_CHANNEL_RANGE_HALF)/(rc_data_local_handle->rc_channels_range[i].center - rc_data_local_handle->rc_channels_range[i].min);
			scale_factor_right[i] = ((float)SDRONE_RC_CHANNEL_RANGE_HALF)/(rc_data_local_handle->rc_channels_range[i].max - rc_data_local_handle->rc_channels_range[i].center);
		} else {
			scale_factor_left[i] = 0.0;
			scale_factor_right[i] = 0.0;
		}
		printf("channel [%d]; scale: [%2.5f][%2.5f]\n", i, scale_factor_left[i], scale_factor_right[i]);
	}

	while (true) {
		ESP_ERROR_CHECK(rc_start(rc_data_local_handle));
		if (rc_data_local_handle->data.txrx_signal == RC_TXRX_TRANSMITTED) {
			for (uint8_t i = 0; i < RC_MAX_CHANNELS; i++) {
				if (rc_data_local_handle->data.raw[i]
						< rc_data_local_handle->rc_channels_range[i].center) {
					rc_data_local_handle->data.norm[i] = (int16_t) ((rc_data_local_handle->rc_channels_range[i].center
							- rc_data_local_handle->data.raw[i])
							* scale_factor_left[i]);
					rc_data_local_handle->data.norm[i] = -rc_data_local_handle->data.norm[i];
				} else {
					rc_data_local_handle->data.norm[i] =
							(int16_t) ((rc_data_local_handle->data.raw[i]
									- rc_data_local_handle->rc_channels_range[i].center)
									* scale_factor_right[i]);
				}
			}
			if ((sdrone_rc_state_handle->controller_task_handle != NULL) && (rc_handle->data.txrx_signal != RC_TXRX_TRANSMITTED)) {
				memcpy(rc_handle, rc_data_local_handle,
						sizeof(*rc_data_local_handle));
				// Throttle is in range [0,SDRONE_RC_CHANNEL_RANGE]
				rc_handle->data.norm[RC_THROTTLE] = rc_data_local_handle->data.norm[RC_THROTTLE]+SDRONE_RC_CHANNEL_RANGE_HALF;
				xTaskNotify(sdrone_rc_state_handle->controller_task_handle,
						sdrone_rc_state_handle->driver_id,
						eSetValueWithOverwrite);

			}
		}
	}
	vTaskDelete(NULL);

}
