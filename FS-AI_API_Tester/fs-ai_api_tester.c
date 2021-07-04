/**************************************************************************
 * Copyright: Preston EV Limited 2018, Rockfort Engineering Ltd. 2019, 2021
 * 
 * File:	fs-ai_api_tester.c
 * Author:	Ian Murphy
 * Date:	2018-06-25, 2019-05-14, 2021-05-22
 * 
 *************************************************************************/


/**************************************************************************
MIT License

Copyright (c) 2021 IMechE Formula Student AI

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*************************************************************************/


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

	printf("\033[2J"); // clear the screen

	while(1) {
		// get some data
		fs_ai_api_vcu2ai_get_data(&vcu2ai_data);

		// output
		printf("\033[H"); // home the cursor
		printf("VCU2AI_HANDSHAKE_RECEIVE_BIT       %u    \r\n",vcu2ai_data.VCU2AI_HANDSHAKE_RECEIVE_BIT);
		printf("VCU2AI_RES_GO_SIGNAL               %u    \r\n",vcu2ai_data.VCU2AI_RES_GO_SIGNAL);
		printf("VCU2AI_AS_STATE                    %u    \r\n",vcu2ai_data.VCU2AI_AS_STATE);
		printf("VCU2AI_AMI_STATE                   %u    \r\n",vcu2ai_data.VCU2AI_AMI_STATE);
		printf("VCU2AI_STEER_ANGLE_deg         %+5.1f    \r\n",vcu2ai_data.VCU2AI_STEER_ANGLE_deg);
		printf("VCU2AI_BRAKE_PRESS_F_pct        %4.1f    \r\n",vcu2ai_data.VCU2AI_BRAKE_PRESS_F_pct);
		printf("VCU2AI_BRAKE_PRESS_R_pct        %4.1f    \r\n",vcu2ai_data.VCU2AI_BRAKE_PRESS_R_pct);
		printf("VCU2AI_FL_WHEEL_SPEED_rpm       %4.0f    \r\n",vcu2ai_data.VCU2AI_FL_WHEEL_SPEED_rpm);
		printf("VCU2AI_FR_WHEEL_SPEED_rpm       %4.0f    \r\n",vcu2ai_data.VCU2AI_FR_WHEEL_SPEED_rpm);
		printf("VCU2AI_RL_WHEEL_SPEED_rpm       %4.0f    \r\n",vcu2ai_data.VCU2AI_RL_WHEEL_SPEED_rpm);
		printf("VCU2AI_RR_WHEEL_SPEED_rpm       %4.0f    \r\n",vcu2ai_data.VCU2AI_RR_WHEEL_SPEED_rpm);
		printf("VCU2AI_FL_PULSE_COUNT          %5u    \r\n",vcu2ai_data.VCU2AI_FL_PULSE_COUNT);
		printf("VCU2AI_FR_PULSE_COUNT          %5u    \r\n",vcu2ai_data.VCU2AI_FR_PULSE_COUNT);
		printf("VCU2AI_RL_PULSE_COUNT          %5u    \r\n",vcu2ai_data.VCU2AI_RL_PULSE_COUNT);
		printf("VCU2AI_RR_PULSE_COUNT          %5u    \r\n",vcu2ai_data.VCU2AI_RR_PULSE_COUNT);

		// TODO: add GPS & IMU data

		// send some data
		if(HANDSHAKE_RECEIVE_BIT_OFF == vcu2ai_data.VCU2AI_HANDSHAKE_RECEIVE_BIT) {
			ai2vcu_data.AI2VCU_HANDSHAKE_SEND_BIT = HANDSHAKE_SEND_BIT_OFF;
		} else if (HANDSHAKE_RECEIVE_BIT_ON == vcu2ai_data.VCU2AI_HANDSHAKE_RECEIVE_BIT) {
			ai2vcu_data.AI2VCU_HANDSHAKE_SEND_BIT = HANDSHAKE_SEND_BIT_ON;
		} else {	// should not be possible
			printf("HANDSHAKE_BIT error\r\n");
		}

		ai2vcu_data.AI2VCU_MISSION_STATUS = MISSION_SELECTED;
		ai2vcu_data.AI2VCU_DIRECTION_REQUEST = DIRECTION_FORWARD;
		ai2vcu_data.AI2VCU_ESTOP_REQUEST = ESTOP_NO;
		ai2vcu_data.AI2VCU_STEER_ANGLE_REQUEST_deg = 9.9f;
		ai2vcu_data.AI2VCU_AXLE_SPEED_REQUEST_rpm = 999.0f;
		ai2vcu_data.AI2VCU_AXLE_TORQUE_REQUEST_Nm = 99.0f;
		ai2vcu_data.AI2VCU_BRAKE_PRESS_REQUEST_pct = 99.0f;

		fs_ai_api_ai2vcu_set_data(&ai2vcu_data);

		// repeat roughly every 10ms
		usleep(10000);
	}

	return(0);
}
