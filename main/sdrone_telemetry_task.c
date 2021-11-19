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
#include <sdrone_telemetry_task.h>

void sdrone_telemetry_task(void *arg) {
	sdrone_telemetry_task_state_handle_t sdrone_telemetry_handle =
			(sdrone_telemetry_task_state_handle_t) arg;
	sdrone_telemetry_handle_t telemetry_handle = &sdrone_telemetry_handle->telemetry_state;
	uint8_t message_num = 0;

	// init telemetry system
	sdrone_telemetry_init(telemetry_handle);
	printf("SDRONE_TELEMETRY SIZE: %d\n", sizeof(sdrone_telemetry_handle->telemetry_state.data));
    while(true) {
    	while(sdrone_telemetry_handle->telemetry_state.server == NULL) {
    	    vTaskDelay(pdMS_TO_TICKS(10000));
    	}
    	message_num %= NUM_MESSAGES;
    	message_num++;

    	memset(&sdrone_telemetry_handle->telemetry_state.data, 0, sizeof(sdrone_telemetry_handle->telemetry_state.data));
    	// prepare data
    	switch(message_num) {
    	case MESSAGE_RC: {
        	sdrone_telemetry_handle->telemetry_state.data.rc.throttle = sdrone_telemetry_handle->sdrone_state_handle->rc_state.rc_data.data.norm[RC_THROTTLE];
        	sdrone_telemetry_handle->telemetry_state.data.rc.roll = sdrone_telemetry_handle->sdrone_state_handle->rc_state.rc_data.data.norm[RC_ROLL];
        	sdrone_telemetry_handle->telemetry_state.data.rc.pitch = sdrone_telemetry_handle->sdrone_state_handle->rc_state.rc_data.data.norm[RC_PITCH];
        	sdrone_telemetry_handle->telemetry_state.data.rc.yaw = sdrone_telemetry_handle->sdrone_state_handle->rc_state.rc_data.data.norm[RC_YAW];
        	sdrone_telemetry_handle->telemetry_state.data.rc.aux1 = sdrone_telemetry_handle->sdrone_state_handle->rc_state.rc_data.data.norm[RC_SWA];
        	sdrone_telemetry_handle->telemetry_state.data.rc.aux2 = sdrone_telemetry_handle->sdrone_state_handle->rc_state.rc_data.data.norm[RC_SWB];
    		break;
    	}
    	case MESSAGE_RPY: {
        	sdrone_telemetry_handle->telemetry_state.data.rpy.roll = (float)sdrone_telemetry_handle->sdrone_state_handle->imu_state.imu.data.gyro.rpy.xyz.x;
        	sdrone_telemetry_handle->telemetry_state.data.rpy.pitch = (float)sdrone_telemetry_handle->sdrone_state_handle->imu_state.imu.data.gyro.rpy.xyz.y;
        	sdrone_telemetry_handle->telemetry_state.data.rpy.yaw = (float)sdrone_telemetry_handle->sdrone_state_handle->imu_state.imu.data.gyro.rpy.xyz.z;
    		break;
    	}
    	case MESSAGE_ACC: {
            sdrone_telemetry_handle->telemetry_state.data.acc.x  = sdrone_telemetry_handle->sdrone_state_handle->imu_state.imu.data.accel_without_g_if[X_POS];
            sdrone_telemetry_handle->telemetry_state.data.acc.y  = sdrone_telemetry_handle->sdrone_state_handle->imu_state.imu.data.accel_without_g_if[Y_POS];
            sdrone_telemetry_handle->telemetry_state.data.acc.z  = sdrone_telemetry_handle->sdrone_state_handle->imu_state.imu.data.accel_without_g_if[Z_POS];
    		break;
    	}
    	case MESSAGE_X: {
        	sdrone_telemetry_handle->telemetry_state.data.x.x = sdrone_telemetry_handle->sdrone_state_handle->controller_state[X_POS].X[SDRONE_TETA_POS];
        	sdrone_telemetry_handle->telemetry_state.data.x.y = sdrone_telemetry_handle->sdrone_state_handle->controller_state[Y_POS].X[SDRONE_TETA_POS];
        	sdrone_telemetry_handle->telemetry_state.data.x.z = sdrone_telemetry_handle->sdrone_state_handle->controller_state[Z_POS].X[SDRONE_TETA_POS];
        	sdrone_telemetry_handle->telemetry_state.data.x.thrust = sdrone_telemetry_handle->sdrone_state_handle->controller_state[Z_POS].X[SDRONE_X_THRUST_POS];
    		break;
    	}
    	case MESSAGE_U: {
        	sdrone_telemetry_handle->telemetry_state.data.u.x = sdrone_telemetry_handle->sdrone_state_handle->controller_state[X_POS].U[SDRONE_TETA_POS];
        	sdrone_telemetry_handle->telemetry_state.data.u.y = sdrone_telemetry_handle->sdrone_state_handle->controller_state[Y_POS].U[SDRONE_TETA_POS];
        	sdrone_telemetry_handle->telemetry_state.data.u.z = sdrone_telemetry_handle->sdrone_state_handle->controller_state[Z_POS].U[SDRONE_TETA_POS];
        	sdrone_telemetry_handle->telemetry_state.data.u.thrust = sdrone_telemetry_handle->sdrone_state_handle->controller_state[Z_POS].U[SDRONE_UW_THRUST_POS];
    		break;
    	}
    	case MESSAGE_W: {
        	sdrone_telemetry_handle->telemetry_state.data.w.x = sdrone_telemetry_handle->sdrone_state_handle->controller_state[X_POS].W[SDRONE_TETA_POS];
        	sdrone_telemetry_handle->telemetry_state.data.w.y = sdrone_telemetry_handle->sdrone_state_handle->controller_state[Y_POS].W[SDRONE_TETA_POS];
        	sdrone_telemetry_handle->telemetry_state.data.w.z = sdrone_telemetry_handle->sdrone_state_handle->controller_state[Z_POS].W[SDRONE_TETA_POS];
        	sdrone_telemetry_handle->telemetry_state.data.w.thrust = sdrone_telemetry_handle->sdrone_state_handle->controller_state[Z_POS].W[SDRONE_UW_THRUST_POS];
    		break;
    	}
    	case MESSAGE_Y: {
        	sdrone_telemetry_handle->telemetry_state.data.y.x = sdrone_telemetry_handle->sdrone_state_handle->controller_state[X_POS].Y[SDRONE_Y_TORQUE_POS];
        	sdrone_telemetry_handle->telemetry_state.data.y.y = sdrone_telemetry_handle->sdrone_state_handle->controller_state[Y_POS].Y[SDRONE_Y_TORQUE_POS];
        	sdrone_telemetry_handle->telemetry_state.data.y.z = sdrone_telemetry_handle->sdrone_state_handle->controller_state[Z_POS].Y[SDRONE_Y_TORQUE_POS];
        	sdrone_telemetry_handle->telemetry_state.data.y.thrust = sdrone_telemetry_handle->sdrone_state_handle->controller_state[Z_POS].Y[SDRONE_Y_THRUST_POS];
    		break;
    	}
    	case MESSAGE_V: {
        	sdrone_telemetry_handle->telemetry_state.data.vertical_v = sdrone_telemetry_handle->sdrone_state_handle->imu_state.imu.data.vertical_v;
    		break;
    	}
    	case MESSAGE_AXIS: {
        	sdrone_telemetry_handle->telemetry_state.data.axis.x = sdrone_telemetry_handle->sdrone_state_handle->motors_state.input.data.at[X_POS];
        	sdrone_telemetry_handle->telemetry_state.data.axis.y = sdrone_telemetry_handle->sdrone_state_handle->motors_state.input.data.at[Y_POS];
        	sdrone_telemetry_handle->telemetry_state.data.axis.z = sdrone_telemetry_handle->sdrone_state_handle->motors_state.input.data.at[Z_POS];
        	sdrone_telemetry_handle->telemetry_state.data.axis.thrust = sdrone_telemetry_handle->sdrone_state_handle->motors_state.motors.thrust_ms;
    		break;
    	}
    	case MESSAGE_GRAVITY: {
        	sdrone_telemetry_handle->telemetry_state.data.gravity.x = (float)sdrone_telemetry_handle->sdrone_state_handle->imu_state.imu.data.gravity_bf[X_POS];
        	sdrone_telemetry_handle->telemetry_state.data.gravity.y = (float)sdrone_telemetry_handle->sdrone_state_handle->imu_state.imu.data.gravity_bf[Y_POS];
        	sdrone_telemetry_handle->telemetry_state.data.gravity.z = (float)sdrone_telemetry_handle->sdrone_state_handle->imu_state.imu.data.gravity_bf[Z_POS];
    		break;
    	}
    	}

    	// send data
		sdrone_telemetry_handle->telemetry_state.data.m_type = message_num;
		sdrone_telemetry_handle->telemetry_state.data.m_timestamp = sdrone_telemetry_handle->sdrone_state_handle->imu_state.imu.data.timestamp;
    	sdrone_telemetry_send_data(telemetry_handle);

    	// wait some time
	    vTaskDelay(pdMS_TO_TICKS(10));

    }
	vTaskDelete(NULL);

}
