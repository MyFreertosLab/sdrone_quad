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

	// init telemetry system
	sdrone_telemetry_init(telemetry_handle);

    while(true) {
    	while(sdrone_telemetry_handle->telemetry_state.server == NULL) {
    	    vTaskDelay(pdMS_TO_TICKS(10000));
    	}

    	// prepare data
    	sdrone_telemetry_handle->telemetry_state.data.acc_x  = (float)sdrone_telemetry_handle->sdrone_state_handle->imu_state.imu.data.accel.cal.kalman[X_POS].X/(float)sdrone_telemetry_handle->sdrone_state_handle->imu_state.imu.data.accel.lsb;
    	sdrone_telemetry_handle->telemetry_state.data.acc_y  = (float)sdrone_telemetry_handle->sdrone_state_handle->imu_state.imu.data.accel.cal.kalman[Y_POS].X/(float)sdrone_telemetry_handle->sdrone_state_handle->imu_state.imu.data.accel.lsb;
    	sdrone_telemetry_handle->telemetry_state.data.acc_z  = (float)sdrone_telemetry_handle->sdrone_state_handle->imu_state.imu.data.accel.cal.kalman[Z_POS].X/(float)sdrone_telemetry_handle->sdrone_state_handle->imu_state.imu.data.accel.lsb;
    	sdrone_telemetry_handle->telemetry_state.data.roll = (float)sdrone_telemetry_handle->sdrone_state_handle->imu_state.imu.data.gyro.rpy.xyz.x;
    	sdrone_telemetry_handle->telemetry_state.data.pitch = (float)sdrone_telemetry_handle->sdrone_state_handle->imu_state.imu.data.gyro.rpy.xyz.y;
    	sdrone_telemetry_handle->telemetry_state.data.yaw = (float)sdrone_telemetry_handle->sdrone_state_handle->imu_state.imu.data.gyro.rpy.xyz.z;
    	sdrone_telemetry_handle->telemetry_state.data.rc_throttle = sdrone_telemetry_handle->sdrone_state_handle->rc_state.rc_data.data.norm[RC_THROTTLE];
    	sdrone_telemetry_handle->telemetry_state.data.rc_roll = sdrone_telemetry_handle->sdrone_state_handle->rc_state.rc_data.data.norm[RC_ROLL];
    	sdrone_telemetry_handle->telemetry_state.data.rc_pitch = sdrone_telemetry_handle->sdrone_state_handle->rc_state.rc_data.data.norm[RC_PITCH];
    	sdrone_telemetry_handle->telemetry_state.data.rc_yaw = sdrone_telemetry_handle->sdrone_state_handle->rc_state.rc_data.data.norm[RC_YAW];
    	sdrone_telemetry_handle->telemetry_state.data.rc_aux1 = sdrone_telemetry_handle->sdrone_state_handle->rc_state.rc_data.data.norm[RC_AUX1];
    	sdrone_telemetry_handle->telemetry_state.data.rc_aux2 = sdrone_telemetry_handle->sdrone_state_handle->rc_state.rc_data.data.norm[RC_AUX2];
    	sdrone_telemetry_handle->telemetry_state.data.w_x = sdrone_telemetry_handle->sdrone_state_handle->controller_state[X_POS].W[SDRONE_TETA_POS];
    	sdrone_telemetry_handle->telemetry_state.data.w_y = sdrone_telemetry_handle->sdrone_state_handle->controller_state[Y_POS].W[SDRONE_TETA_POS];
    	sdrone_telemetry_handle->telemetry_state.data.w_z = sdrone_telemetry_handle->sdrone_state_handle->controller_state[Z_POS].W[SDRONE_TETA_POS];
    	sdrone_telemetry_handle->telemetry_state.data.w_thrust = sdrone_telemetry_handle->sdrone_state_handle->controller_state[Z_POS].U[SDRONE_THRUST_POS];
    	sdrone_telemetry_handle->telemetry_state.data.ax_x = sdrone_telemetry_handle->sdrone_state_handle->motors_state.input.data.at[X_POS];
    	sdrone_telemetry_handle->telemetry_state.data.ax_y = sdrone_telemetry_handle->sdrone_state_handle->motors_state.input.data.at[Y_POS];
    	sdrone_telemetry_handle->telemetry_state.data.ax_z = sdrone_telemetry_handle->sdrone_state_handle->motors_state.input.data.at[Z_POS];
    	sdrone_telemetry_handle->telemetry_state.data.ax_thrust = sdrone_telemetry_handle->sdrone_state_handle->motors_state.input.data.thrust;

    	// send data
    	sdrone_telemetry_send_data(telemetry_handle);

    	// wait some time
	    vTaskDelay(pdMS_TO_TICKS(100));
    }
	vTaskDelete(NULL);

}
