/*
 * sdrone_telemetry_task.c
 *
 *  Created on: 19 ott 2021
 *      Author: andrea
 */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <esp_log.h>
#include <sdrone_telemetry_task.h>
#include <wsserver.h>

static const char *TAG = "sdrone_telemetry";

static esp_err_t sdrone_telemetry_receive_cb(httpd_ws_frame_t* ws_pkt) {
	ESP_LOGI(TAG, "sdrone_telemetry_receive_cb: received packet of type [%d]", ws_pkt->type);
	return ESP_OK;
}

static sdrone_httpd_uri_t sdrone_telemetry_uri_handle = {
		.handler.uri = "/telemetry",
		.handler.method = HTTP_GET,
		.handler.handler = NULL,
		.handler.user_ctx = NULL,
		.handler.is_websocket = true,
		.handler.handle_ws_control_frames = true,
		.receive_cb = sdrone_telemetry_receive_cb
};

void sdrone_telemetry_task(void *arg) {
	sdrone_telemetry_task_state_handle_t sdrone_telemetry_handle =
			(sdrone_telemetry_task_state_handle_t) arg;
	sdrone_wsserver_handle_t telemetry_handle = &sdrone_telemetry_handle->wsserver_state;

	uint8_t message_num = 0;

	// init telemetry system
	sdrone_wsserver_init(telemetry_handle);
	sdrone_wsserver_add_uri_handle(&sdrone_telemetry_uri_handle);

	sdrone_telemetry_data_t data;
    while(true) {
    	while(sdrone_telemetry_handle->wsserver_state.server == NULL) {
    	    vTaskDelay(pdMS_TO_TICKS(10000));
    	}
    	for(uint8_t i = 1; i <= NUM_MESSAGES; i++) {
    		bool prepared = true;
        	// prepare data
        	switch(i) {
        	// rotational
        	case MESSAGE_UROT: {
            	data.urot.roll = sdrone_telemetry_handle->sdrone_state_handle->controller_state[X_POS].dynamic.U[SDRONE_UW_TETA_POS];
            	data.urot.pitch = sdrone_telemetry_handle->sdrone_state_handle->controller_state[Y_POS].dynamic.U[SDRONE_UW_TETA_POS];
            	data.urot.yaw = sdrone_telemetry_handle->sdrone_state_handle->controller_state[Z_POS].dynamic.U[SDRONE_UW_TETA_POS];
        		break;
        	}
        	case MESSAGE_RPY: {
            	data.rpy.roll = (float)sdrone_telemetry_handle->sdrone_state_handle->imu_state.imu.data.gyro.rpy.xyz.x;
            	data.rpy.pitch = (float)sdrone_telemetry_handle->sdrone_state_handle->imu_state.imu.data.gyro.rpy.xyz.y;
            	data.rpy.yaw = (float)sdrone_telemetry_handle->sdrone_state_handle->imu_state.imu.data.gyro.rpy.xyz.z;
        		break;
        	}
        	case MESSAGE_ALFA: {
            	data.alfa.x = sdrone_telemetry_handle->sdrone_state_handle->controller_state[X_POS].dynamic.X[SDRONE_X_ALFA_POS];
            	data.alfa.y = sdrone_telemetry_handle->sdrone_state_handle->controller_state[Y_POS].dynamic.X[SDRONE_X_ALFA_POS];
            	data.alfa.z = sdrone_telemetry_handle->sdrone_state_handle->controller_state[Z_POS].dynamic.X[SDRONE_X_ALFA_POS];
        		break;
        	}
        	case MESSAGE_EALFA: {
            	data.ealfa.eroll = sdrone_telemetry_handle->sdrone_state_handle->controller_state[X_POS].pid_alfa.err;
            	data.ealfa.epitch = sdrone_telemetry_handle->sdrone_state_handle->controller_state[Y_POS].pid_alfa.err;
            	data.ealfa.eyaw = sdrone_telemetry_handle->sdrone_state_handle->controller_state[Z_POS].pid_alfa.err;
        		break;
        	}

        	// linear
        	case MESSAGE_UACC: {
            	data.uacc.x = sdrone_telemetry_handle->sdrone_state_handle->controller_state[X_POS].dynamic.U[SDRONE_UW_ACC_POS];
            	data.uacc.y = sdrone_telemetry_handle->sdrone_state_handle->controller_state[Y_POS].dynamic.U[SDRONE_UW_ACC_POS];
            	data.uacc.z = sdrone_telemetry_handle->sdrone_state_handle->controller_state[Z_POS].dynamic.U[SDRONE_UW_ACC_POS];
        		break;
        	}
        	case MESSAGE_SPEED: {
                data.speed.x  = sdrone_telemetry_handle->sdrone_state_handle->imu_state.imu.data.speed_if[X_POS];
                data.speed.y  = sdrone_telemetry_handle->sdrone_state_handle->imu_state.imu.data.speed_if[Y_POS];
                data.speed.z  = sdrone_telemetry_handle->sdrone_state_handle->imu_state.imu.data.speed_if[Z_POS];
        		break;
        	}
        	case MESSAGE_ACC: {
                data.acc.x  = sdrone_telemetry_handle->sdrone_state_handle->imu_state.imu.data.accel.mss_if.array[X_POS];
                data.acc.y  = sdrone_telemetry_handle->sdrone_state_handle->imu_state.imu.data.accel.mss_if.array[Y_POS];
                data.acc.z  = sdrone_telemetry_handle->sdrone_state_handle->imu_state.imu.data.accel.mss_if.array[Z_POS];
        		break;
        	}
        	case MESSAGE_EACC: {
            	data.eacc.ex = sdrone_telemetry_handle->sdrone_state_handle->controller_state[X_POS].pid_acc.err;
            	data.eacc.ey = sdrone_telemetry_handle->sdrone_state_handle->controller_state[Y_POS].pid_acc.err;
            	data.eacc.ez = sdrone_telemetry_handle->sdrone_state_handle->controller_state[Z_POS].pid_acc.err;
        		break;
        	}
        	default: {
        		prepared = false;
        		break;
        	}
        	}

        	if(prepared) {
            	// send data
        		data.m_type = i;
        		data.m_timestamp = sdrone_telemetry_handle->sdrone_state_handle->imu_state.imu.data.timestamp;
        		httpd_ws_frame_t frame;
        		frame.final = true;
        		frame.fragmented = false;
        		frame.payload = (uint8_t*)(&data);
        		frame.len = sizeof(data);
        		frame.type = HTTPD_WS_TYPE_BINARY;
            	sdrone_wsserver_send_data(&sdrone_telemetry_uri_handle, &frame);
        	}
    	}
    	// wait some time
	    vTaskDelay(pdMS_TO_TICKS(100));

    }
	vTaskDelete(NULL);

}
