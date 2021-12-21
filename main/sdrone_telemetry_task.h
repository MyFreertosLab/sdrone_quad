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

#define MESSAGE_UROT          0x01
#define MESSAGE_RPY           0x02
#define MESSAGE_ALFA          0x03
#define MESSAGE_EALFA         0x04
#define MESSAGE_UACC          0x05
#define MESSAGE_SPEED         0x06
#define MESSAGE_ACC           0x07
#define MESSAGE_EACC          0x08
#define NUM_MESSAGES          0x08

typedef struct {
	uint16_t m_type;
	uint32_t m_timestamp;
	union {
		// rotational
		struct {
			float roll;
			float pitch;
			float yaw;
		} urot;
		struct {
			float roll;
			float pitch;
			float yaw;
		} rpy;
		struct {
			float x;
			float y;
			float z;
		} alfa;
		struct {
			float eroll;
			float epitch;
			float eyaw;
		} ealfa;
		// linear
		struct {
			float x;
			float y;
			float z;
		} uacc;
		struct {
			float x;
			float y;
			float z;
		} speed;
		struct {
			float x;
			float y;
			float z;
		} acc;
		struct {
			float ex;
			float ey;
			float ez;
		} eacc;
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
