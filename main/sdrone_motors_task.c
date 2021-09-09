/*
 * sdrone_motors_task.c
 *
 *  Created on: 19 apr 2021
 *      Author: andrea
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <motors.h>
#include <sdrone_motors_task.h>
#include <driver/i2c.h>
#include <ina3221.h>


void sdrone_motors_controller_init(
		sdrone_motors_state_handle_t sdrone_motors_state_handle) {
	printf("sdrone_motors_controller_init init initial state and motors\n");
	memset(sdrone_motors_state_handle, 0, sizeof(*sdrone_motors_state_handle));
	ESP_ERROR_CHECK(motors_init(&(sdrone_motors_state_handle->motors)));

	sdrone_motors_state_handle->motors_task_handle =
			xTaskGetCurrentTaskHandle();

}

uint8_t motors_counter = 0;
void sdrone_motors_controller_cycle(
		sdrone_motors_state_handle_t sdrone_motors_state_handle) {
	motors_handle_t motors_handle = &(sdrone_motors_state_handle->motors);

	printf("sdrone_motors_task_init::Arm motors\n");
	ESP_ERROR_CHECK(motors_arm(motors_handle));
	vTaskDelay(pdMS_TO_TICKS(10));

	while (true) {
		// Take Notify at end of duty_cycle
		if (ulTaskNotifyTake( pdTRUE, pdMS_TO_TICKS(10))
				== sdrone_motors_state_handle->controller_driver_id) {
			if (sdrone_motors_state_handle->input.isCommand) {
				switch (sdrone_motors_state_handle->input.command.type) {
				case MOTORS_ARM: {
					ESP_ERROR_CHECK(motors_arm(motors_handle));
					break;
				}
				case MOTORS_DISARM: {
					ESP_ERROR_CHECK(motors_disarm(motors_handle));
					break;
				}
				case MOTORS_SWITCHOFF: {
					ESP_ERROR_CHECK(motors_switchoff(motors_handle));
					break;
				}
				case MOTORS_SWITCHON: {
					ESP_ERROR_CHECK(motors_switchon(motors_handle));
					break;
				}
				default: {
					printf(
							"ERROR: sdrone_motors_controller_cycle::command %d unknown for motors",
							sdrone_motors_state_handle->input.command.type);
				}
				}
			} else if (sdrone_motors_state_handle->input.data.tx_rx_flag
					== MOTORS_TXRX_TRANSMITTED) {
				motors_counter++;
				motors_counter %= 100;
				if(motors_counter == 0) {
					  printf("newton: [%5.5f,%5.5f,%5.5f], [%5.5f]\n", sdrone_motors_state_handle->input.data.at[0], sdrone_motors_state_handle->input.data.at[1], sdrone_motors_state_handle->input.data.at[2], sdrone_motors_state_handle->input.data.thrust);
				}
				// set at
				for(uint8_t i = 0; i < 3; i++) {
					motors_handle->at[i] = sdrone_motors_state_handle->input.data.at[i];
				}
				motors_handle->thrust = sdrone_motors_state_handle->input.data.thrust;

			  ESP_ERROR_CHECK(motors_update(motors_handle));
				sdrone_motors_state_handle->input.data.tx_rx_flag =
						MOTORS_TXRX_RECEIVED;
			}
		}
	}
}

void sdrone_motors_task(void *arg) {
	sdrone_motors_state_handle_t sdrone_motors_state_handle =
			(sdrone_motors_state_handle_t) arg;
	sdrone_motors_controller_init(sdrone_motors_state_handle);
	sdrone_motors_controller_cycle(sdrone_motors_state_handle);

}
