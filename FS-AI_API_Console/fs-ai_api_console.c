/**************************************************************************
 * Copyright: Preston EV Limited 2018, Rockfort Engineering Ltd. 2019, 2021
 * 
 * File:	fs-ai_api_console.c
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
#include <string.h>

#include <unistd.h>
#include <pthread.h>

#include "../FS-AI_API/fs-ai_api.h"

static pthread_t loop_tid;
static void *loop_thread();
static fs_ai_api_ai2vcu ai2vcu_data;	// assume initialised to zeros
static char inputs[10] = "", outputs[80] = "                                        ";
static int data = 0;
static int timing_us = 10000;


int main(int argc, char** argv) {
	if (argc < 2) {
            printf("Too few arguments!\r\n");
			printf("Usage: fs-ai_api_console <can>\r\n");
            return(1);
        }
        
        if(fs_ai_api_init(argv[1],1,0)) {	// initialise with Debug flag
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

	printf("\033[2J"); // clear the screen

	while(1) {
		// background loop
		scanf("%s %d", inputs, &data);
		
		if(0 == strcmp(inputs, "s")) {
			sprintf(outputs, "Steer: %d                                   \r\n", data);
			ai2vcu_data.AI2VCU_STEER_ANGLE_REQUEST_deg = (float)data;
		}

		if(0 == strcmp(inputs, "r")) {
			sprintf(outputs, "rpm: %d                                   \r\n", data);
			ai2vcu_data.AI2VCU_AXLE_SPEED_REQUEST_rpm = (float)data;
		}

		if(0 == strcmp(inputs, "b")) {
			sprintf(outputs, "Brakes: %d                                   \r\n", data);
			ai2vcu_data.AI2VCU_BRAKE_PRESS_REQUEST_pct = (float)data;
		}

		if(0 == strcmp(inputs, "t")) {
			sprintf(outputs, "Torque: %d                                   \r\n", data);
			ai2vcu_data.AI2VCU_AXLE_TORQUE_REQUEST_Nm = (float)data;
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
		
		if(0 == strcmp(inputs, "l")) {
			sprintf(outputs, "Loop_us: %d                                   \r\n", data);
			timing_us = data;
		}
	}
}

static void *loop_thread() {
	fs_ai_api_vcu2ai vcu2ai_data;
	
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
		printf("VCU2AI_FL_PULSE_COUNT          %5u       \r\n",vcu2ai_data.VCU2AI_FL_PULSE_COUNT);
		printf("VCU2AI_FR_PULSE_COUNT          %5u       \r\n",vcu2ai_data.VCU2AI_FR_PULSE_COUNT);
		printf("VCU2AI_RL_PULSE_COUNT          %5u       \r\n",vcu2ai_data.VCU2AI_RL_PULSE_COUNT);
		printf("VCU2AI_RR_PULSE_COUNT          %5u       \r\n",vcu2ai_data.VCU2AI_RR_PULSE_COUNT);

		printf("AI2VCU_DIRECTION_REQUEST           %u    \r\n",ai2vcu_data.AI2VCU_DIRECTION_REQUEST);
		printf("AI2VCU_ESTOP_REQUEST               %u    \r\n",ai2vcu_data.AI2VCU_ESTOP_REQUEST);
		printf("AI2VCU_MISSION_STATUS              %u    \r\n",ai2vcu_data.AI2VCU_MISSION_STATUS);
		printf("AI2VCU_STEER_ANGLE_REQUEST_deg %+5.1f    \r\n",ai2vcu_data.AI2VCU_STEER_ANGLE_REQUEST_deg);
		printf("AI2VCU_AXLE_SPEED_REQUEST_rpm   %4.0f    \r\n",ai2vcu_data.AI2VCU_AXLE_SPEED_REQUEST_rpm);
		printf("AI2VCU_AXLE_TORQUE_REQUEST_Nm   %4.0f    \r\n",ai2vcu_data.AI2VCU_AXLE_TORQUE_REQUEST_Nm);
		printf("AI2VCU_BRAKE_PRESS_REQUEST_pct  %4.0f    \r\n",ai2vcu_data.AI2VCU_BRAKE_PRESS_REQUEST_pct);
		
		printf("                                         \r\n");
		printf("%s                                       \r\n",outputs);

		// send some data
		if(HANDSHAKE_RECEIVE_BIT_OFF == vcu2ai_data.VCU2AI_HANDSHAKE_RECEIVE_BIT) {
			ai2vcu_data.AI2VCU_HANDSHAKE_SEND_BIT = HANDSHAKE_SEND_BIT_OFF;
		} else if (HANDSHAKE_RECEIVE_BIT_ON == vcu2ai_data.VCU2AI_HANDSHAKE_RECEIVE_BIT) {
			ai2vcu_data.AI2VCU_HANDSHAKE_SEND_BIT = HANDSHAKE_SEND_BIT_ON;
		} else {	// should not be possible
			printf("HANDSHAKE_BIT error\r\n");
		}

		fs_ai_api_ai2vcu_set_data(&ai2vcu_data);

		// repeat according to loop timing (default roughly every 10ms)
		usleep(timing_us);
	}

	return(0);
}
