/*
 * sdrone_telemetry_task.h
 *
 *  Created on: 19 ott 2021
 *      Author: andrea
 */

#ifndef MAIN_SDRONE_TELEMETRY_TASK_H_
#define MAIN_SDRONE_TELEMETRY_TASK_H_

#include <wsserver.h>
#include <sdrone_controller_task.h>

#define VERTICAL_DATA_ONLY

#define MESSAGE_RC            0x01
#define MESSAGE_RPY           0x02
#define MESSAGE_ACC           0x03
#define MESSAGE_W             0x04
#define MESSAGE_AXIS          0x05
#define MESSAGE_GRAVITY       0x06
#define MESSAGE_V             0x07
#define MESSAGE_X             0x08
#define MESSAGE_U             0x09
#define MESSAGE_Y             0x0A
#define MESSAGE_Z_DATA        0x0B
#define NUM_MESSAGES          0x0B

typedef struct {
	uint16_t m_type;
	uint32_t m_timestamp;
	union {
		struct {
			int16_t rc_t;
			float   x_t;
			float   u_t;
			float   w_t;
			float   y_t;
			float   vertical_v;
		} z_data;
		struct {
			int16_t throttle;
			int16_t roll;
			int16_t pitch;
			int16_t yaw;
			int16_t aux1;
			int16_t aux2;
		} rc;
		struct {
			float roll;
			float pitch;
			float yaw;
		} rpy;
		struct {
			float x;
			float y;
			float z;
		} acc;
		struct {
			float x;
			float y;
			float z;
			float thrust;
		} x;
		struct {
			float x;
			float y;
			float z;
			float thrust;
		} u;
		struct {
			float x;
			float y;
			float z;
			float thrust;
		} w;
		struct {
			float x;
			float y;
			float z;
			float thrust;
		} y;
		struct {
			float x;
			float y;
			float z;
			float thrust;
		} axis;
		struct {
			float x;
			float y;
			float z;
		} gravity;
		float vertical_v;
	};
} sdrone_telemetry_data_t;

typedef struct {
	uint32_t driver_id;
	uint32_t controller_driver_id;
	TaskHandle_t controller_task_handle;
	TaskHandle_t telemetry_task_handle;
	sdrone_state_handle_t sdrone_state_handle;
	sdrone_wsserver_state_t wsserver_state;
} sdrone_telemetry_task_state_t;

typedef sdrone_telemetry_task_state_t* sdrone_telemetry_task_state_handle_t;

void sdrone_telemetry_task(void *arg);

#endif /* MAIN_SDRONE_TELEMETRY_TASK_H_ */
