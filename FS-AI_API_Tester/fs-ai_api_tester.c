/*********************************************************************
 * Copyright: Preston EV Limited 2018, Rockfort Engineering Ltd. 2019
 * 
 * File:	fs-ai_api_tester.c
 * Author:	Ian Murphy
 * Date:	2018-06-25, 2019-05-14
 * 
 ********************************************************************/


#include <stdio.h>

#include <unistd.h>

#include "../FS-AI_API/fs-ai_api.h"


int main(int argc, char** argv) {
	fs_ai_api_vcu2ai vcu2ai_data;
	fs_ai_api_ai2vcu ai2vcu_data;
	
	if (argc < 2) {
            printf("Too few arguments!\r\n");
			printf("Usage: fs-ai_api_tester <can>\r\n");
            return(1);
        }
        
        if(fs_ai_api_init(argv[1],1,1)) {	// initialise with Debug & Simulate flags
			printf("fs_ai_api_init() failed\r\n");
			return(1);
		} 
	
	while(1) {
		// get some data
		fs_ai_api_vcu2ai_get_data(&vcu2ai_data);

		printf("VCU2AI_AS_STATE                 %u    \r\n",vcu2ai_data.VCU2AI_AS_STATE);
		printf("VCU2AI_AMI_STATE                %u    \r\n",vcu2ai_data.VCU2AI_AMI_STATE);
		printf("VCU2AI_HANDSHAKE_RECEIVE_BIT    %u    \r\n",vcu2ai_data.VCU2AI_HANDSHAKE_RECEIVE_BIT);
		printf("VCU2AI_STEER_ANGLE_MAX_deg      %4.1f    \r\n",vcu2ai_data.VCU2AI_STEER_ANGLE_MAX_deg);
		printf("VCU2AI_STEER_ANGLE_deg         %+5.1f    \r\n",vcu2ai_data.VCU2AI_STEER_ANGLE_deg);
		printf("VCU2AI_WHEEL_SPEED_MAX_rpm      %4.0f    \r\n",vcu2ai_data.VCU2AI_WHEEL_SPEED_MAX_rpm);
		printf("VCU2AI_FL_WHEEL_SPEED_rpm       %4.0f    \r\n",vcu2ai_data.VCU2AI_FL_WHEEL_SPEED_rpm);
		printf("VCU2AI_FR_WHEEL_SPEED_rpm       %4.0f    \r\n",vcu2ai_data.VCU2AI_FR_WHEEL_SPEED_rpm);
		printf("VCU2AI_RL_WHEEL_SPEED_rpm       %4.0f    \r\n",vcu2ai_data.VCU2AI_RL_WHEEL_SPEED_rpm);
		printf("VCU2AI_RR_WHEEL_SPEED_rpm       %4.0f    \r\n",vcu2ai_data.VCU2AI_RR_WHEEL_SPEED_rpm);
		printf("VCU2AI_FL_PULSE_COUNT          %5u    \r\n",vcu2ai_data.VCU2AI_FL_PULSE_COUNT);
		printf("VCU2AI_FR_PULSE_COUNT          %5u    \r\n",vcu2ai_data.VCU2AI_FR_PULSE_COUNT);
		printf("VCU2AI_RL_PULSE_COUNT          %5u    \r\n",vcu2ai_data.VCU2AI_RL_PULSE_COUNT);
		printf("VCU2AI_RR_PULSE_COUNT          %5u    \r\n",vcu2ai_data.VCU2AI_RR_PULSE_COUNT);
		printf("VCU2AI_Accel_longitudinal_ms2  %+5.1f    \r\n",vcu2ai_data.VCU2AI_Accel_longitudinal_ms2);
		printf("VCU2AI_Accel_lateral_ms2       %+5.1f    \r\n",vcu2ai_data.VCU2AI_Accel_lateral_ms2);
		printf("VCU2AI_Accel_vertical_ms2      %+5.1f    \r\n",vcu2ai_data.VCU2AI_Accel_vertical_ms2);
		printf("VCU2AI_Yaw_rate_degps         %+6.1f    \r\n",vcu2ai_data.VCU2AI_Yaw_rate_degps);
		printf("VCU2AI_Roll_rate_degps        %+6.1f    \r\n",vcu2ai_data.VCU2AI_Roll_rate_degps);
		printf("VCU2AI_Pitch_rate_degps       %+6.1f    \r\n",vcu2ai_data.VCU2AI_Pitch_rate_degps);
												
		printf("\033[21A");  // move cursor back up

		// send some data
		if(HANDSHAKE_RECEIVE_BIT_OFF == vcu2ai_data.VCU2AI_HANDSHAKE_RECEIVE_BIT) {
			ai2vcu_data.AI2VCU_HANDSHAKE_SEND_BIT = HANDSHAKE_SEND_BIT_OFF;
		} else if (HANDSHAKE_RECEIVE_BIT_ON == vcu2ai_data.VCU2AI_HANDSHAKE_RECEIVE_BIT) {
			ai2vcu_data.AI2VCU_HANDSHAKE_SEND_BIT = HANDSHAKE_SEND_BIT_ON;
		} else {	// should not be possible
			printf("HANDSHAKE_BIT error\r\n");
		}

		ai2vcu_data.AI2VCU_DIRECTION_REQUEST = DIRECTION_FORWARD;
		ai2vcu_data.AI2VCU_ESTOP_REQUEST = ESTOP_NO;
		ai2vcu_data.AI2VCU_MISSION_STATUS = MISSION_SELECTED;
		ai2vcu_data.AI2VCU_STEER_ANGLE_REQUEST_deg = 9.9f;
		ai2vcu_data.AI2VCU_WHEEL_SPEED_REQUEST_rpm = 999.0f;

		fs_ai_api_ai2vcu_set_data(&ai2vcu_data);

		// repeat roughly every 5ms
		usleep(5000);
	}

	return(0);
}
