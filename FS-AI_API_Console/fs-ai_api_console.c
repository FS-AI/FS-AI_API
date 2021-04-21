/*********************************************************************
 * Copyright: Preston EV Limited 2018, Rockfort Engineering Ltd. 2019
 * 
 * File:	fs-ai_api_tester.c
 * Author:	Ian Murphy
 * Date:	2018-06-25, 2019-05-14
 * 
 ********************************************************************/


#include <stdio.h>
#include <string.h>

#include <unistd.h>
#include <pthread.h>

#include "../FS-AI_API/fs-ai_api.h"

static pthread_t loop_tid;
static void *loop_thread();
static fs_ai_api_ai2vcu ai2vcu_data;	// assume initialised to zeros
static char inputs[10] = "", outputs[80] = "                                        ";
static int data = 0;
static int timing_us = 5000;

// DEBUG_1
void fs_ai_api_debug_get_data(uint8_t *data);
static uint8_t DEBUG_data[8] = {0,0,0,0,0,0,0,0};


int main(int argc, char** argv) {
	if (argc < 2) {
            printf("Too few arguments!\r\n");
			printf("Usage: fs-ai_api_tester <can>\r\n");
            return(1);
        }
        
        if(fs_ai_api_init(argv[1],1,0)) {	// initialise with Debug & Simulate flags
			printf("fs_ai_api_init() failed\r\n");
			return(1);
		} 

	// spawn thread
	if(pthread_create(&loop_tid, NULL, &loop_thread, NULL) != 0) {
		printf("Can't create loop thread...");
		return(1);
	}
	
	printf("\r\n");
	printf("Ready Player One...\r\n");
	
	while(1) {
		// background loop
		scanf("%s %d", inputs, &data);
		
		if(0 == strcmp(inputs, "s")) {
			sprintf(outputs, "Steer: %d                                   \r\n", data);
			ai2vcu_data.AI2VCU_STEER_ANGLE_REQUEST_deg = (float)data;
		}

		if(0 == strcmp(inputs, "r")) {
			sprintf(outputs, "rpm: %d                                   \r\n", data);
			ai2vcu_data.AI2VCU_WHEEL_SPEED_REQUEST_rpm = (float)data;
		}

		if(0 == strcmp(inputs, "m")) {
			sprintf(outputs, "Mission: %d                                   \r\n", data);
			ai2vcu_data.AI2VCU_MISSION_STATUS = data;
		}

		if(0 == strcmp(inputs, "d")) {
			sprintf(outputs, "Dir: %d                                   \r\n", data);
			ai2vcu_data.AI2VCU_DIRECTION_REQUEST = data;
		}

		if(0 == strcmp(inputs, "e")) {
			sprintf(outputs, "EStop: %d                                   \r\n", data);
			ai2vcu_data.AI2VCU_ESTOP_REQUEST = data;
		}
		
		if(0 == strcmp(inputs, "t")) {
			sprintf(outputs, "Timing: %d                                   \r\n", data);
			timing_us = data;
		}
	}
}

static void *loop_thread() {
	fs_ai_api_vcu2ai vcu2ai_data;
	
	while(1) {
		// get some data
		fs_ai_api_vcu2ai_get_data(&vcu2ai_data);
		fs_ai_api_debug_get_data(DEBUG_data);

		// low pass filter the IMU for better screen display
		static float VCU2AI_Accel_longitudinal_ms2_lpf,VCU2AI_Accel_lateral_ms2_lpf,VCU2AI_Accel_vertical_ms2_lpf;
		static float VCU2AI_Yaw_rate_degps_lpf,VCU2AI_Roll_rate_degps_lpf,VCU2AI_Pitch_rate_degps_lpf;
		
		VCU2AI_Accel_longitudinal_ms2_lpf = 0.95f*VCU2AI_Accel_longitudinal_ms2_lpf + 0.05f*vcu2ai_data.VCU2AI_Accel_longitudinal_ms2;
		VCU2AI_Accel_lateral_ms2_lpf = 0.95f*VCU2AI_Accel_lateral_ms2_lpf + 0.05f*vcu2ai_data.VCU2AI_Accel_lateral_ms2;
		VCU2AI_Accel_vertical_ms2_lpf = 0.95f*VCU2AI_Accel_vertical_ms2_lpf + 0.05f*vcu2ai_data.VCU2AI_Accel_vertical_ms2;
		VCU2AI_Yaw_rate_degps_lpf = 0.95f*VCU2AI_Yaw_rate_degps_lpf + 0.05f*vcu2ai_data.VCU2AI_Yaw_rate_degps;
		VCU2AI_Roll_rate_degps_lpf = 0.95f*VCU2AI_Roll_rate_degps_lpf + 0.05f*vcu2ai_data.VCU2AI_Roll_rate_degps;
		VCU2AI_Pitch_rate_degps_lpf = 0.95f*VCU2AI_Pitch_rate_degps_lpf + 0.05f*vcu2ai_data.VCU2AI_Pitch_rate_degps;
		
		// output
		printf("VCU2AI_AS_STATE                    %u    \r\n",vcu2ai_data.VCU2AI_AS_STATE);
		printf("VCU2AI_AMI_STATE                   %u    \r\n",vcu2ai_data.VCU2AI_AMI_STATE);
		printf("VCU2AI_HANDSHAKE_RECEIVE_BIT       %u    \r\n",vcu2ai_data.VCU2AI_HANDSHAKE_RECEIVE_BIT);
		printf("VCU2AI_STEER_ANGLE_MAX_deg      %4.1f    \r\n",vcu2ai_data.VCU2AI_STEER_ANGLE_MAX_deg);
		printf("VCU2AI_STEER_ANGLE_deg         %+5.1f    \r\n",vcu2ai_data.VCU2AI_STEER_ANGLE_deg);
		printf("VCU2AI_WHEEL_SPEED_MAX_rpm      %4.0f    \r\n",vcu2ai_data.VCU2AI_WHEEL_SPEED_MAX_rpm);
		printf("VCU2AI_FL_WHEEL_SPEED_rpm       %4.0f    \r\n",vcu2ai_data.VCU2AI_FL_WHEEL_SPEED_rpm);
		printf("VCU2AI_FR_WHEEL_SPEED_rpm       %4.0f    \r\n",vcu2ai_data.VCU2AI_FR_WHEEL_SPEED_rpm);
		printf("VCU2AI_RL_WHEEL_SPEED_rpm       %4.0f    \r\n",vcu2ai_data.VCU2AI_RL_WHEEL_SPEED_rpm);
		printf("VCU2AI_RR_WHEEL_SPEED_rpm       %4.0f    \r\n",vcu2ai_data.VCU2AI_RR_WHEEL_SPEED_rpm);
		printf("VCU2AI_FL_PULSE_COUNT          %5u       \r\n",vcu2ai_data.VCU2AI_FL_PULSE_COUNT);
		printf("VCU2AI_FR_PULSE_COUNT          %5u       \r\n",vcu2ai_data.VCU2AI_FR_PULSE_COUNT);
		printf("VCU2AI_RL_PULSE_COUNT          %5u       \r\n",vcu2ai_data.VCU2AI_RL_PULSE_COUNT);
		printf("VCU2AI_RR_PULSE_COUNT          %5u       \r\n",vcu2ai_data.VCU2AI_RR_PULSE_COUNT);
		printf("VCU2AI_Accel_longitudinal_ms2 %+6.2f     \r\n",VCU2AI_Accel_longitudinal_ms2_lpf);
		printf("VCU2AI_Accel_lateral_ms2      %+6.2f     \r\n",VCU2AI_Accel_lateral_ms2_lpf);
		printf("VCU2AI_Accel_vertical_ms2     %+6.2f     \r\n",VCU2AI_Accel_vertical_ms2_lpf);
		printf("VCU2AI_Yaw_rate_degps          %+5.1f    \r\n",VCU2AI_Yaw_rate_degps_lpf);
		printf("VCU2AI_Roll_rate_degps         %+5.1f    \r\n",VCU2AI_Roll_rate_degps_lpf);
		printf("VCU2AI_Pitch_rate_degps        %+5.1f    \r\n",VCU2AI_Pitch_rate_degps_lpf);

		printf("DEBUG_1: %u %u %u %u %u %u %u %u\r\n",DEBUG_data[0],DEBUG_data[1],DEBUG_data[2],DEBUG_data[3],DEBUG_data[4],DEBUG_data[5],DEBUG_data[6],DEBUG_data[7]);
		
		printf("AI2VCU_DIRECTION_REQUEST           %u    \r\n",ai2vcu_data.AI2VCU_DIRECTION_REQUEST);
		printf("AI2VCU_ESTOP_REQUEST               %u    \r\n",ai2vcu_data.AI2VCU_ESTOP_REQUEST);
		printf("AI2VCU_MISSION_STATUS              %u    \r\n",ai2vcu_data.AI2VCU_MISSION_STATUS);
		printf("AI2VCU_STEER_ANGLE_REQUEST_deg %+5.1f    \r\n",ai2vcu_data.AI2VCU_STEER_ANGLE_REQUEST_deg);
		printf("AI2VCU_WHEEL_SPEED_REQUEST_rpm  %4.0f    \r\n",ai2vcu_data.AI2VCU_WHEEL_SPEED_REQUEST_rpm);
		
		printf("%s %d\r\n                 ", inputs, data);
		printf("\033[27A");  // move cursor back up

		// send some data
		if(HANDSHAKE_RECEIVE_BIT_OFF == vcu2ai_data.VCU2AI_HANDSHAKE_RECEIVE_BIT) {
			ai2vcu_data.AI2VCU_HANDSHAKE_SEND_BIT = HANDSHAKE_SEND_BIT_OFF;
		} else if (HANDSHAKE_RECEIVE_BIT_ON == vcu2ai_data.VCU2AI_HANDSHAKE_RECEIVE_BIT) {
			ai2vcu_data.AI2VCU_HANDSHAKE_SEND_BIT = HANDSHAKE_SEND_BIT_ON;
		} else {	// should not be possible
			printf("HANDSHAKE_BIT error\r\n");
		}

//		ai2vcu_data.AI2VCU_DIRECTION_REQUEST = DIRECTION_NEUTRAL;
//		ai2vcu_data.AI2VCU_ESTOP_REQUEST = ESTOP_NO;
//		ai2vcu_data.AI2VCU_MISSION_STATUS = MISSION_SELECTED;
//		ai2vcu_data.AI2VCU_STEER_ANGLE_REQUEST_deg = 9.9f;
//		ai2vcu_data.AI2VCU_WHEEL_SPEED_REQUEST_rpm = 0.0f;

		fs_ai_api_ai2vcu_set_data(&ai2vcu_data);

		// repeat roughly every 5ms
		usleep(timing_us);
	}

	return(0);
}
