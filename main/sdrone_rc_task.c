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
#include <nvs_flash.h>
#include <nvs.h>

#if CONFIG_ESP_RC_PROTOCOL_IBUS
#include <rc_ibus.h>
#include <driver/uart.h>
#elif  CONFIG_ESP_RC_PROTOCOL_PPM
#include <rc_ppm.h>
#endif
#include <sdrone_rc_task.h>

static esp_err_t rc_load_calibration_data(rc_handle_t rc_handle) {
	nvs_handle_t my_handle;
	uint8_t flashed = 0;
	ESP_ERROR_CHECK(nvs_open("RC_CAL", NVS_READWRITE, &my_handle));
	ESP_ERROR_CHECK(nvs_get_u8(my_handle, "FLASHED", &flashed));

	ESP_ERROR_CHECK(
			nvs_get_u16(my_handle, "RC_MIN_1",
					&rc_handle->rc_channels_range[0].min));
	ESP_ERROR_CHECK(
			nvs_get_u16(my_handle, "RC_MIN_2",
					&rc_handle->rc_channels_range[1].min));
	ESP_ERROR_CHECK(
			nvs_get_u16(my_handle, "RC_MIN_3",
					&rc_handle->rc_channels_range[2].min));
	ESP_ERROR_CHECK(
			nvs_get_u16(my_handle, "RC_MIN_4",
					&rc_handle->rc_channels_range[3].min));
	ESP_ERROR_CHECK(
			nvs_get_u16(my_handle, "RC_MIN_5",
					&rc_handle->rc_channels_range[4].min));
	ESP_ERROR_CHECK(
			nvs_get_u16(my_handle, "RC_MIN_6",
					&rc_handle->rc_channels_range[5].min));
	ESP_ERROR_CHECK(
			nvs_get_u16(my_handle, "RC_MIN_7",
					&rc_handle->rc_channels_range[6].min));
	ESP_ERROR_CHECK(
			nvs_get_u16(my_handle, "RC_MIN_8",
					&rc_handle->rc_channels_range[7].min));
	ESP_ERROR_CHECK(
			nvs_get_u16(my_handle, "RC_MIN_9",
					&rc_handle->rc_channels_range[8].min));
	ESP_ERROR_CHECK(
			nvs_get_u16(my_handle, "RC_MIN_10",
					&rc_handle->rc_channels_range[9].min));

	ESP_ERROR_CHECK(
			nvs_get_u16(my_handle, "RC_MAX_1",
					&rc_handle->rc_channels_range[0].max));
	ESP_ERROR_CHECK(
			nvs_get_u16(my_handle, "RC_MAX_2",
					&rc_handle->rc_channels_range[1].max));
	ESP_ERROR_CHECK(
			nvs_get_u16(my_handle, "RC_MAX_3",
					&rc_handle->rc_channels_range[2].max));
	ESP_ERROR_CHECK(
			nvs_get_u16(my_handle, "RC_MAX_4",
					&rc_handle->rc_channels_range[3].max));
	ESP_ERROR_CHECK(
			nvs_get_u16(my_handle, "RC_MAX_5",
					&rc_handle->rc_channels_range[4].max));
	ESP_ERROR_CHECK(
			nvs_get_u16(my_handle, "RC_MAX_6",
					&rc_handle->rc_channels_range[5].max));
	ESP_ERROR_CHECK(
			nvs_get_u16(my_handle, "RC_MAX_7",
					&rc_handle->rc_channels_range[6].max));
	ESP_ERROR_CHECK(
			nvs_get_u16(my_handle, "RC_MAX_8",
					&rc_handle->rc_channels_range[7].max));
	ESP_ERROR_CHECK(
			nvs_get_u16(my_handle, "RC_MAX_9",
					&rc_handle->rc_channels_range[8].max));
	ESP_ERROR_CHECK(
			nvs_get_u16(my_handle, "RC_MAX_10",
					&rc_handle->rc_channels_range[9].max));

	ESP_ERROR_CHECK(
			nvs_get_u16(my_handle, "RC_CNT_1",
					&rc_handle->rc_channels_range[0].center));
	ESP_ERROR_CHECK(
			nvs_get_u16(my_handle, "RC_CNT_2",
					&rc_handle->rc_channels_range[1].center));
	ESP_ERROR_CHECK(
			nvs_get_u16(my_handle, "RC_CNT_3",
					&rc_handle->rc_channels_range[2].center));
	ESP_ERROR_CHECK(
			nvs_get_u16(my_handle, "RC_CNT_4",
					&rc_handle->rc_channels_range[3].center));
	ESP_ERROR_CHECK(
			nvs_get_u16(my_handle, "RC_CNT_5",
					&rc_handle->rc_channels_range[4].center));
	ESP_ERROR_CHECK(
			nvs_get_u16(my_handle, "RC_CNT_6",
					&rc_handle->rc_channels_range[5].center));
	ESP_ERROR_CHECK(
			nvs_get_u16(my_handle, "RC_CNT_7",
					&rc_handle->rc_channels_range[6].center));
	ESP_ERROR_CHECK(
			nvs_get_u16(my_handle, "RC_CNT_8",
					&rc_handle->rc_channels_range[7].center));
	ESP_ERROR_CHECK(
			nvs_get_u16(my_handle, "RC_CNT_9",
					&rc_handle->rc_channels_range[8].center));
	ESP_ERROR_CHECK(
			nvs_get_u16(my_handle, "RC_CNT_10",
					&rc_handle->rc_channels_range[9].center));

	// Close
	nvs_close(my_handle);
	return ESP_OK;
}

void sdrone_rc_task(void *arg) {
	sdrone_rc_state_handle_t sdrone_rc_state_handle =
			(sdrone_rc_state_handle_t) arg;
	rc_handle_t rc_handle = &(sdrone_rc_state_handle->rc_data);
	rc_t rc_data_local;
	rc_t* rc_data_local_handle = &rc_data_local;
	sdrone_rc_state_handle->rc_task_handle = xTaskGetCurrentTaskHandle();
	memcpy(rc_data_local_handle, &sdrone_rc_state_handle->rc_data, sizeof(rc_t));

	printf("RC: load calibration data from NVS\n");
	ESP_ERROR_CHECK(rc_load_calibration_data(rc_data_local_handle));

#ifdef RC_PROTOCOL_IBUS
	rc_data_local_handle->init = (esp_err_t (*)(void *))rc_ibus_init;
	rc_data_local_handle->start = (esp_err_t (*)(void *))rc_ibus_start;
	rc_data_local_handle->stop = (esp_err_t (*)(void *))rc_ibus_stop;
#else
#ifdef RC_PROTOCOL_PPM
	rc_data_local_handle->init = (esp_err_t (*)(void *))rc_ppm_init;
	rc_data_local_handle->start = (esp_err_t (*)(void *))rc_ppm_start;
	rc_data_local_handle->stop = (esp_err_t (*)(void *))rc_ppm_stop;
#endif
#endif

	ESP_ERROR_CHECK(rc_data_local_handle->init(rc_data_local_handle));

	float scale_factor_left[RC_MAX_CHANNELS];
	float scale_factor_right[RC_MAX_CHANNELS];

	for(uint8_t i = 0; i < RC_MAX_CHANNELS; i++) {
		if((rc_data_local_handle->rc_channels_range[i].center - rc_data_local_handle->rc_channels_range[i].min) != 0) {
			scale_factor_left[i] = ((float)SDRONE_RC_CHANNEL_RANGE_HALF)/(rc_data_local_handle->rc_channels_range[i].center - rc_data_local_handle->rc_channels_range[i].min);
			scale_factor_right[i] = ((float)SDRONE_RC_CHANNEL_RANGE_HALF)/(rc_data_local_handle->rc_channels_range[i].max - rc_data_local_handle->rc_channels_range[i].center);
		} else if((rc_data_local_handle->rc_channels_range[i].max - rc_data_local_handle->rc_channels_range[i].min) != 0) {
			scale_factor_left[i] = ((float)SDRONE_RC_CHANNEL_RANGE_HALF)/((rc_data_local_handle->rc_channels_range[i].max - rc_data_local_handle->rc_channels_range[i].min) >> 1);
			scale_factor_right[i] = scale_factor_left[i];
		} else {
			scale_factor_left[i] = 0.0;
			scale_factor_right[i] = 0.0;
		}
		printf("channel [%d]; scale: [%2.5f][%2.5f]\n", i, scale_factor_left[i], scale_factor_right[i]);
	}
	memcpy(&sdrone_rc_state_handle->rc_data, rc_data_local_handle, sizeof(rc_t));
	while (true) {
		ESP_ERROR_CHECK(rc_data_local_handle->start(rc_data_local_handle));

		// normalize data
		if (rc_data_local_handle->data.txrx_signal == RC_TXRX_TRANSMITTED) {
			rc_data_local_handle->state = RC_CONNECTED;
			for (uint8_t i = 0; i < RC_MAX_CHANNELS; i++) {
				if (rc_data_local_handle->data.raw[i]
						< rc_data_local_handle->rc_channels_range[i].center) {
					rc_data_local_handle->data.norm[i] =
							-(int16_t) ((rc_data_local_handle->rc_channels_range[i].center
									- rc_data_local_handle->data.raw[i])
									* scale_factor_left[i]);
				} else {
					rc_data_local_handle->data.norm[i] =
							(int16_t) ((rc_data_local_handle->data.raw[i]
									- rc_data_local_handle->rc_channels_range[i].center)
									* scale_factor_right[i]);
				}
			}
		} else {
			rc_data_local_handle->state = RC_NOT_CONNECTED;
		}

		// send notify
		if ((sdrone_rc_state_handle->controller_task_handle != NULL) && (rc_handle->data.txrx_signal != RC_TXRX_TRANSMITTED)) {
			memcpy(rc_handle, rc_data_local_handle,
					sizeof(*rc_data_local_handle));

			// Yaw is left positive and right negative
			rc_handle->data.norm[RC_YAW] =-rc_data_local_handle->data.norm[RC_YAW];
			rc_handle->data.txrx_signal = RC_TXRX_TRANSMITTED;
			xTaskNotify(sdrone_rc_state_handle->controller_task_handle,sdrone_rc_state_handle->driver_id, eSetValueWithOverwrite);
		}
		if(rc_data_local_handle->state == RC_CONNECTED) {
			vTaskDelay(pdMS_TO_TICKS(10));
		} else {
			vTaskDelay(pdMS_TO_TICKS(100));
		}
	}
	vTaskDelete(NULL);
}
