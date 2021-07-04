/**************************************************************************
 * Copyright: Preston EV Limited 2018, Rockfort Engineering Ltd. 2019, 2021
 * 
 * File:	fs-ai_api.c
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
#include <stdlib.h>
#include <stdint.h>

#include <string.h>

#include <unistd.h>
#include <pthread.h>

#include <time.h>

#include "can.h"

#include "fs-ai_api.h"


// typedefs
typedef enum boolean_e {
	FALSE = 0,
	TRUE = 1,
} boolean_e;

typedef struct pack_16_t {
	union {
		volatile uint16_t uword;
		volatile int16_t sword;
		volatile uint8_t bytes[2];
	};
} pack_16_t;

typedef union can_data_t {
	volatile uint8_t ubytes[8];
	volatile int8_t sbytes[8];
	volatile uint16_t uwords[4];
	volatile int16_t swords[4];
	volatile uint32_t ulongs[2];
	volatile int32_t slongs[2];
	volatile float floats[2];
} can_data_t;


// statics
static boolean_e debug_mode = FALSE;
static boolean_e simulate_mode = FALSE;

static const float MOTOR_RATIO = 3.5f;
static const float MOTOR_MAX_RPM = 4000.0f;

static pthread_t can_read_tid;
static pthread_mutex_t can_read_mutex = PTHREAD_MUTEX_INITIALIZER;

static struct timespec last_set, this_set;

// tx frames
static struct can_frame AI2VCU_Status		= {0x510,8};
static struct can_frame AI2VCU_Drive_F		= {0x511,4};
static struct can_frame AI2VCU_Drive_R		= {0x512,4};
static struct can_frame AI2VCU_Steer		= {0x513,2};
static struct can_frame AI2VCU_Brake		= {0x514,2};

// rx frames
#define VCU2AI_STATUS_ID		0x520
#define VCU2AI_DRIVE_F_ID		0x521
#define VCU2AI_DRIVE_R_ID		0x522
#define VCU2AI_STEER_ID			0x523
#define VCU2AI_BRAKE_ID			0x524
#define VCU2AI_WHEEL_SPEEDS_ID	0x525
#define VCU2AI_WHEEL_COUNTS_ID	0x526

static struct can_frame VCU2AI_Status;
static struct can_frame VCU2AI_Drive_F;
static struct can_frame VCU2AI_Drive_R;
static struct can_frame VCU2AI_Steer;
static struct can_frame VCU2AI_Brake;
static struct can_frame VCU2AI_Wheel_speeds;
static struct can_frame VCU2AI_Wheel_counts;

#define PCAN_GPS_BMC_ACCELERATION_ID		0X600
#define PCAN_GPS_BMC_MAGNETICFIELD_ID		0X628	// requires changing in PCAN-GPS firmware to avoid RES ID clash
#define PCAN_GPS_L3GD20_ROTATION_A_ID		0X610
#define PCAN_GPS_L3GD20_ROTATION_B_ID		0X611
#define PCAN_GPS_GPS_STATUS_ID				0X620
#define PCAN_GPS_GPS_COURSESPEED_ID			0X621
#define PCAN_GPS_GPS_POSITIONLONGITUDE_ID	0X622
#define PCAN_GPS_GPS_POSITIONLATITUDE_ID	0X623
#define PCAN_GPS_GPS_POSITIONALTITUDE_ID	0X624
#define PCAN_GPS_GPS_DELUSIONS_A_ID			0X625
#define PCAN_GPS_GPS_DELUSIONS_B_ID			0X626
#define	PCAN_GPS_GPS_DATETIME_ID			0X627

static struct can_frame PCAN_GPS_BMC_Acceleration;
static struct can_frame PCAN_GPS_BMC_MagneticField;
static struct can_frame PCAN_GPS_L3GD20_Rotation_A;
static struct can_frame PCAN_GPS_L3GD20_Rotation_B;
static struct can_frame PCAN_GPS_GPS_Status;
static struct can_frame PCAN_GPS_GPS_CourseSpeed;
static struct can_frame PCAN_GPS_GPS_Longitude;
static struct can_frame PCAN_GPS_GPS_Latitude;
static struct can_frame PCAN_GPS_GPS_Altitude;
static struct can_frame PCAN_GPS_GPS_Delusions_A;
static struct can_frame PCAN_GPS_GPS_Delusions_B;
static struct can_frame PCAN_GPS_GPS_DateTime;

// static local data
static can_stats_t can_stats;

static volatile boolean_e VCU2AI_Status_fresh = FALSE;
static volatile boolean_e VCU2AI_Drive_F_fresh = FALSE;
static volatile boolean_e VCU2AI_Drive_R_fresh = FALSE;
static volatile boolean_e VCU2AI_Steer_fresh = FALSE;
static volatile boolean_e VCU2AI_Brake_fresh = FALSE;
static volatile boolean_e VCU2AI_Wheel_speeds_fresh = FALSE;
static volatile boolean_e VCU2AI_Wheel_counts_fresh = FALSE;
static volatile boolean_e PCAN_GPS_BMC_Acceleration_fresh = FALSE;
static volatile boolean_e PCAN_GPS_BMC_MagneticField_fresh = FALSE;
static volatile boolean_e PCAN_GPS_L3GD20_Rotation_A_fresh = FALSE;
static volatile boolean_e PCAN_GPS_L3GD20_Rotation_B_fresh = FALSE;
static volatile boolean_e PCAN_GPS_GPS_Status_fresh = FALSE;
static volatile boolean_e PCAN_GPS_GPS_CourseSpeed_fresh = FALSE;
static volatile boolean_e PCAN_GPS_GPS_Longitude_fresh = FALSE;
static volatile boolean_e PCAN_GPS_GPS_Latitude_fresh = FALSE;
static volatile boolean_e PCAN_GPS_GPS_Altitude_fresh = FALSE;
static volatile boolean_e PCAN_GPS_GPS_Delusions_A_fresh = FALSE;
static volatile boolean_e PCAN_GPS_GPS_Delusions_B_fresh = FALSE;
static volatile boolean_e PCAN_GPS_GPS_DateTime_fresh = FALSE;

// AI2VCU_Status;
static volatile fs_ai_api_handshake_send_bit_e	AI2VCU_HANDSHAKE_BIT = HANDSHAKE_SEND_BIT_OFF;
static volatile fs_ai_api_estop_request_e		AI2VCU_ESTOP_REQUEST = ESTOP_NO;
static volatile fs_ai_api_mission_status_e		AI2VCU_MISSION_STATUS = MISSION_NOT_SELECTED;
static volatile fs_ai_api_direction_request_e	AI2VCU_DIRECTION_REQUEST = DIRECTION_NEUTRAL;
static volatile uint8_t							AI2VCU_LAP_COUNTER = 0;
static volatile uint8_t							AI2VCU_CONES_COUNT_ACTUAL = 0;
static volatile uint16_t						AI2VCU_CONES_COUNT_ALL = 0;
static volatile uint8_t							AI2VCU_VEH_SPEED_ACTUAL_kmh = 0;
static volatile uint8_t							AI2VCU_VEH_SPEED_DEMAND_kmh = 0;

// AI2VCU_Drive_F
static volatile uint16_t AI2VCU_FRONT_AXLE_TRQ_REQUEST_raw = 0;
static volatile uint16_t AI2VCU_FRONT_MOTOR_SPEED_MAX_rpm = 0;

// AI2VCU_Drive_R
static volatile uint16_t AI2VCU_REAR_AXLE_TRQ_REQUEST_raw = 0;
static volatile uint16_t AI2VCU_REAR_MOTOR_SPEED_MAX_rpm = 0;

// AI2VCU_Steer
static volatile int16_t AI2VCU_STEER_REQUEST_raw = 0;

// AI2VCU_Brake
static volatile uint8_t AI2VCU_HYD_PRESS_F_REQ_raw = 0;
static volatile uint8_t AI2VCU_HYD_PRESS_R_REQ_raw = 0;

// VCU2AI_Status
static volatile fs_ai_api_handshake_receive_bit_e	VCU2AI_HANDSHAKE_RECEIVE_BIT = HANDSHAKE_RECEIVE_BIT_OFF;
static volatile fs_ai_api_res_go_signal_bit_e		VCU2AI_RES_GO_SIGNAL = RES_GO_SIGNAL_NO_GO;
static volatile fs_ai_api_as_state_e				VCU2AI_AS_STATE = AS_OFF;
static volatile fs_ai_api_ami_state_e				VCU2AI_AMI_STATE = AMI_NOT_SELECTED;
// remaining fields not relevant to API

// VCU2AI_Drive_F
static volatile uint16_t	VCU2AI_FRONT_AXLE_TORQUE_MAX_raw = 0;
// remaining fields not relevant to API

// VCU2AI_Drive_R
static volatile uint16_t	VCU2AI_REAR_AXLE_TORQUE_MAX_raw = 0;
// remaining fields not relevant to API

// VCU2AI_Steer
static volatile int16_t		VCU2AI_STEER_ANGLE_raw = 0;
static volatile uint16_t	VCU2AI_STEER_ANGLE_MAX_raw = 0;
// remaining fields not relevant to API

// VCU2AI_Brake
static volatile uint8_t	VCU2AI_BRAKE_PRESS_F_raw = 0;
static volatile uint8_t	VCU2AI_BRAKE_PRESS_R_raw = 0;
// remaining fields not relevant to API

// VCU2AI_Wheel_speeds
static volatile uint16_t	VCU2AI_FL_WHEEL_SPEED_rpm = 0;
static volatile uint16_t	VCU2AI_FR_WHEEL_SPEED_rpm = 0;
static volatile uint16_t	VCU2AI_RL_WHEEL_SPEED_rpm = 0;
static volatile uint16_t	VCU2AI_RR_WHEEL_SPEED_rpm = 0;

// VCU2AI_Wheel_counts
static volatile uint16_t	VCU2AI_FL_PULSE_COUNT = 0;
static volatile uint16_t	VCU2AI_FR_PULSE_COUNT = 0;
static volatile uint16_t	VCU2AI_RL_PULSE_COUNT = 0;
static volatile uint16_t	VCU2AI_RR_PULSE_COUNT = 0;

// PCAN_GPS_BMC_Acceleration
static volatile float IMU_Acceleration_X_mG = 0.0f;
static volatile float IMU_Acceleration_Y_mG = 0.0f;
static volatile float IMU_Acceleration_Z_mG = 0.0f;
static volatile float IMU_Temperature_degC = 0.0f;
static volatile uint8_t IMU_VerticalAxis = 0;
static volatile uint8_t IMU_Orientation = 0;

// PCAN_GPS_BMC_MagneticField
static volatile float IMU_MagneticField_X_uT = 0.0f;
static volatile float IMU_MagneticField_Y_uT = 0.0f;
static volatile float IMU_MagneticField_Z_uT = 0.0f;

// PCAN_GPS_L3GD20_Rotation_A
static volatile float IMU_Rotation_X_degps = 0.0f;
static volatile float IMU_Rotation_Y_degps = 0.0f;

// PCAN_GPS_L3GD20_Rotation_B
static volatile float IMU_Rotation_Z_degps = 0.0f;

// PCAN_GPS_GPS_Status
static volatile uint8_t GPS_AntennaStatus = 0;
static volatile uint8_t GPS_NumSatellites = 0;
static volatile uint8_t GPS_NavigationMethod = 0;

// PCAN_GPS_GPS_CourseSpeed
static volatile float GPS_Course_deg = 0.0f;
static volatile float GPS_Speed_kmh = 0.0f;

// PCAN_GPS_GPS_Longitude
static volatile float GPS_Longitude_Minutes = 0.0f;
static volatile uint16_t GPS_Longitude_Degree = 0;
static volatile uint8_t GPS_Longitude_IndicatorEW = 0;

// PCAN_GPS_GPS_Latitude
static volatile float GPS_Latitude_Minutes = 0.0f;
static volatile uint16_t GPS_Latitude_Degree = 0;
static volatile uint8_t GPS_Latitude_IndicatorNS = 0;

// PCAN_GPS_GPS_Altitude
static volatile float GPS_Altitude = 0.0f;

// PCAN_GPS_GPS_Delusions_A
static volatile float GPS_PDOP = 0.0f;
static volatile float GPS_HDOP = 0.0f;

// PCAN_GPS_GPS_Delusions_B
static volatile float GPS_VDOP = 0.0f;

// PCAN_GPS_GPS_DateTime
static volatile uint8_t GPS_UTC_Year = 0;
static volatile uint8_t GPS_UTC_Month = 0;
static volatile uint8_t GPS_UTC_DayOfMonth = 0;
static volatile uint8_t GPS_UTC_Hour = 0;
static volatile uint8_t GPS_UTC_Minute = 0;
static volatile uint8_t GPS_UTC_Second = 0;


// functions
static void *can_read_thread() {
	struct can_frame read_frame;
	
	while(1)
	{
		if(can_read(&read_frame) < 0) {
			if(debug_mode) { printf("CAN read error!\r\n"); }
		}

		pthread_mutex_lock(&can_read_mutex); // protect the buffers from race conditions

		switch(read_frame.can_id) {
			case VCU2AI_STATUS_ID : {
				VCU2AI_Status.data[0] = read_frame.data[0];
				VCU2AI_Status.data[1] = read_frame.data[1];
				VCU2AI_Status.data[2] = read_frame.data[2];
				VCU2AI_Status.data[3] = read_frame.data[3];
				VCU2AI_Status.data[4] = read_frame.data[4];
				VCU2AI_Status.data[5] = read_frame.data[5];
				VCU2AI_Status.data[6] = read_frame.data[6];
				VCU2AI_Status.data[7] = read_frame.data[7];
				VCU2AI_Status_fresh = TRUE;
				can_stats.VCU2AI_Status_count++;
				break;
			}
			case VCU2AI_DRIVE_F_ID : {
				VCU2AI_Drive_F.data[0] = read_frame.data[0];
				VCU2AI_Drive_F.data[1] = read_frame.data[1];
				VCU2AI_Drive_F.data[2] = read_frame.data[2];
				VCU2AI_Drive_F.data[3] = read_frame.data[3];
				VCU2AI_Drive_F.data[4] = read_frame.data[4];
				VCU2AI_Drive_F.data[5] = read_frame.data[5];
				VCU2AI_Drive_F.data[6] = read_frame.data[6];
				VCU2AI_Drive_F.data[7] = read_frame.data[7];
				VCU2AI_Drive_F_fresh = TRUE;
				can_stats.VCU2AI_Drive_F_count++;
				break;
			}
			case VCU2AI_DRIVE_R_ID : {
				VCU2AI_Drive_R.data[0] = read_frame.data[0];
				VCU2AI_Drive_R.data[1] = read_frame.data[1];
				VCU2AI_Drive_R.data[2] = read_frame.data[2];
				VCU2AI_Drive_R.data[3] = read_frame.data[3];
				VCU2AI_Drive_R.data[4] = read_frame.data[4];
				VCU2AI_Drive_R.data[5] = read_frame.data[5];
				VCU2AI_Drive_R.data[6] = read_frame.data[6];
				VCU2AI_Drive_R.data[7] = read_frame.data[7];
				VCU2AI_Drive_R_fresh = TRUE;
				can_stats.VCU2AI_Drive_R_count++;
				break;
			}
			case VCU2AI_STEER_ID : {
				VCU2AI_Steer.data[0] = read_frame.data[0];
				VCU2AI_Steer.data[1] = read_frame.data[1];
				VCU2AI_Steer.data[2] = read_frame.data[2];
				VCU2AI_Steer.data[3] = read_frame.data[3];
				VCU2AI_Steer.data[4] = read_frame.data[4];
				VCU2AI_Steer.data[5] = read_frame.data[5];
				VCU2AI_Steer.data[6] = read_frame.data[6];
				VCU2AI_Steer.data[7] = read_frame.data[7];
				VCU2AI_Steer_fresh = TRUE;
				can_stats.VCU2AI_Steer_count++;
				break;
			}
			case VCU2AI_BRAKE_ID : {
				VCU2AI_Brake.data[0] = read_frame.data[0];
				VCU2AI_Brake.data[1] = read_frame.data[1];
				VCU2AI_Brake.data[2] = read_frame.data[2];
				VCU2AI_Brake.data[3] = read_frame.data[3];
				VCU2AI_Brake.data[4] = read_frame.data[4];
				VCU2AI_Brake.data[5] = read_frame.data[5];
				VCU2AI_Brake.data[6] = read_frame.data[6];
				VCU2AI_Brake.data[7] = read_frame.data[7];
				VCU2AI_Brake_fresh = TRUE;
				can_stats.VCU2AI_Brake_count++;
				break;
			}
			case VCU2AI_WHEEL_SPEEDS_ID : {
				VCU2AI_Wheel_speeds.data[0] = read_frame.data[0];
				VCU2AI_Wheel_speeds.data[1] = read_frame.data[1];
				VCU2AI_Wheel_speeds.data[2] = read_frame.data[2];
				VCU2AI_Wheel_speeds.data[3] = read_frame.data[3];
				VCU2AI_Wheel_speeds.data[4] = read_frame.data[4];
				VCU2AI_Wheel_speeds.data[5] = read_frame.data[5];
				VCU2AI_Wheel_speeds.data[6] = read_frame.data[6];
				VCU2AI_Wheel_speeds.data[7] = read_frame.data[7];
				VCU2AI_Wheel_speeds_fresh = TRUE;
				can_stats.VCU2AI_Wheel_speeds_count++;
				break;
			}
			case VCU2AI_WHEEL_COUNTS_ID : {
				VCU2AI_Wheel_counts.data[0] = read_frame.data[0];
				VCU2AI_Wheel_counts.data[1] = read_frame.data[1];
				VCU2AI_Wheel_counts.data[2] = read_frame.data[2];
				VCU2AI_Wheel_counts.data[3] = read_frame.data[3];
				VCU2AI_Wheel_counts.data[4] = read_frame.data[4];
				VCU2AI_Wheel_counts.data[5] = read_frame.data[5];
				VCU2AI_Wheel_counts.data[6] = read_frame.data[6];
				VCU2AI_Wheel_counts.data[7] = read_frame.data[7];
				VCU2AI_Wheel_counts_fresh = TRUE;
				can_stats.VCU2AI_Wheel_counts_count++;
				break;
			}
			case PCAN_GPS_BMC_ACCELERATION_ID : {
				PCAN_GPS_BMC_Acceleration.data[0] = read_frame.data[0];
				PCAN_GPS_BMC_Acceleration.data[1] = read_frame.data[1];
				PCAN_GPS_BMC_Acceleration.data[2] = read_frame.data[2];
				PCAN_GPS_BMC_Acceleration.data[3] = read_frame.data[3];
				PCAN_GPS_BMC_Acceleration.data[4] = read_frame.data[4];
				PCAN_GPS_BMC_Acceleration.data[5] = read_frame.data[5];
				PCAN_GPS_BMC_Acceleration.data[6] = read_frame.data[6];
				PCAN_GPS_BMC_Acceleration.data[7] = read_frame.data[7];
				PCAN_GPS_BMC_Acceleration_fresh = TRUE;
				can_stats.PCAN_GPS_BMC_Acceleration_count++;
				break;
			}
			case PCAN_GPS_BMC_MAGNETICFIELD_ID : {
				PCAN_GPS_BMC_MagneticField.data[0] = read_frame.data[0];
				PCAN_GPS_BMC_MagneticField.data[1] = read_frame.data[1];
				PCAN_GPS_BMC_MagneticField.data[2] = read_frame.data[2];
				PCAN_GPS_BMC_MagneticField.data[3] = read_frame.data[3];
				PCAN_GPS_BMC_MagneticField.data[4] = read_frame.data[4];
				PCAN_GPS_BMC_MagneticField.data[5] = read_frame.data[5];
				PCAN_GPS_BMC_MagneticField.data[6] = read_frame.data[6];
				PCAN_GPS_BMC_MagneticField.data[7] = read_frame.data[7];
				PCAN_GPS_BMC_MagneticField_fresh = TRUE;
				can_stats.PCAN_GPS_BMC_MagneticField_count++;
				break;
			}
			case PCAN_GPS_L3GD20_ROTATION_A_ID : {
				PCAN_GPS_L3GD20_Rotation_A.data[0] = read_frame.data[0];
				PCAN_GPS_L3GD20_Rotation_A.data[1] = read_frame.data[1];
				PCAN_GPS_L3GD20_Rotation_A.data[2] = read_frame.data[2];
				PCAN_GPS_L3GD20_Rotation_A.data[3] = read_frame.data[3];
				PCAN_GPS_L3GD20_Rotation_A.data[4] = read_frame.data[4];
				PCAN_GPS_L3GD20_Rotation_A.data[5] = read_frame.data[5];
				PCAN_GPS_L3GD20_Rotation_A.data[6] = read_frame.data[6];
				PCAN_GPS_L3GD20_Rotation_A.data[7] = read_frame.data[7];
				PCAN_GPS_L3GD20_Rotation_A_fresh = TRUE;
				can_stats.PCAN_GPS_L3GD20_Rotation_A_count++;
				break;
			}
			case PCAN_GPS_L3GD20_ROTATION_B_ID : {
				PCAN_GPS_L3GD20_Rotation_B.data[0] = read_frame.data[0];
				PCAN_GPS_L3GD20_Rotation_B.data[1] = read_frame.data[1];
				PCAN_GPS_L3GD20_Rotation_B.data[2] = read_frame.data[2];
				PCAN_GPS_L3GD20_Rotation_B.data[3] = read_frame.data[3];
				PCAN_GPS_L3GD20_Rotation_B.data[4] = read_frame.data[4];
				PCAN_GPS_L3GD20_Rotation_B.data[5] = read_frame.data[5];
				PCAN_GPS_L3GD20_Rotation_B.data[6] = read_frame.data[6];
				PCAN_GPS_L3GD20_Rotation_B.data[7] = read_frame.data[7];
				PCAN_GPS_L3GD20_Rotation_B_fresh = TRUE;
				can_stats.PCAN_GPS_L3GD20_Rotation_B_count++;
				break;
			}
			case PCAN_GPS_GPS_STATUS_ID : {
				PCAN_GPS_GPS_Status.data[0] = read_frame.data[0];
				PCAN_GPS_GPS_Status.data[1] = read_frame.data[1];
				PCAN_GPS_GPS_Status.data[2] = read_frame.data[2];
				PCAN_GPS_GPS_Status.data[3] = read_frame.data[3];
				PCAN_GPS_GPS_Status.data[4] = read_frame.data[4];
				PCAN_GPS_GPS_Status.data[5] = read_frame.data[5];
				PCAN_GPS_GPS_Status.data[6] = read_frame.data[6];
				PCAN_GPS_GPS_Status.data[7] = read_frame.data[7];
				PCAN_GPS_GPS_Status_fresh = TRUE;
				can_stats.PCAN_GPS_GPS_Status_count++;
				break;
			}
			case PCAN_GPS_GPS_COURSESPEED_ID : {
				PCAN_GPS_GPS_CourseSpeed.data[0] = read_frame.data[0];
				PCAN_GPS_GPS_CourseSpeed.data[1] = read_frame.data[1];
				PCAN_GPS_GPS_CourseSpeed.data[2] = read_frame.data[2];
				PCAN_GPS_GPS_CourseSpeed.data[3] = read_frame.data[3];
				PCAN_GPS_GPS_CourseSpeed.data[4] = read_frame.data[4];
				PCAN_GPS_GPS_CourseSpeed.data[5] = read_frame.data[5];
				PCAN_GPS_GPS_CourseSpeed.data[6] = read_frame.data[6];
				PCAN_GPS_GPS_CourseSpeed.data[7] = read_frame.data[7];
				PCAN_GPS_GPS_CourseSpeed_fresh = TRUE;
				can_stats.PCAN_GPS_GPS_CourseSpeed_count++;
				break;
			}
			case PCAN_GPS_GPS_POSITIONLONGITUDE_ID : {
				PCAN_GPS_GPS_Longitude.data[0] = read_frame.data[0];
				PCAN_GPS_GPS_Longitude.data[1] = read_frame.data[1];
				PCAN_GPS_GPS_Longitude.data[2] = read_frame.data[2];
				PCAN_GPS_GPS_Longitude.data[3] = read_frame.data[3];
				PCAN_GPS_GPS_Longitude.data[4] = read_frame.data[4];
				PCAN_GPS_GPS_Longitude.data[5] = read_frame.data[5];
				PCAN_GPS_GPS_Longitude.data[6] = read_frame.data[6];
				PCAN_GPS_GPS_Longitude.data[7] = read_frame.data[7];
				PCAN_GPS_GPS_Longitude_fresh = TRUE;
				can_stats.PCAN_GPS_GPS_Longitude_count++;
				break;
			}
			case PCAN_GPS_GPS_POSITIONLATITUDE_ID : {
				PCAN_GPS_GPS_Latitude.data[0] = read_frame.data[0];
				PCAN_GPS_GPS_Latitude.data[1] = read_frame.data[1];
				PCAN_GPS_GPS_Latitude.data[2] = read_frame.data[2];
				PCAN_GPS_GPS_Latitude.data[3] = read_frame.data[3];
				PCAN_GPS_GPS_Latitude.data[4] = read_frame.data[4];
				PCAN_GPS_GPS_Latitude.data[5] = read_frame.data[5];
				PCAN_GPS_GPS_Latitude.data[6] = read_frame.data[6];
				PCAN_GPS_GPS_Latitude.data[7] = read_frame.data[7];
				PCAN_GPS_GPS_Latitude_fresh = TRUE;
				can_stats.PCAN_GPS_GPS_Latitude_count++;
				break;
			}
			case PCAN_GPS_GPS_POSITIONALTITUDE_ID : {
				PCAN_GPS_GPS_Altitude.data[0] = read_frame.data[0];
				PCAN_GPS_GPS_Altitude.data[1] = read_frame.data[1];
				PCAN_GPS_GPS_Altitude.data[2] = read_frame.data[2];
				PCAN_GPS_GPS_Altitude.data[3] = read_frame.data[3];
				PCAN_GPS_GPS_Altitude.data[4] = read_frame.data[4];
				PCAN_GPS_GPS_Altitude.data[5] = read_frame.data[5];
				PCAN_GPS_GPS_Altitude.data[6] = read_frame.data[6];
				PCAN_GPS_GPS_Altitude.data[7] = read_frame.data[7];
				PCAN_GPS_GPS_Altitude_fresh = TRUE;
				can_stats.PCAN_GPS_GPS_Altitude_count++;
				break;
			}
			case PCAN_GPS_GPS_DELUSIONS_A_ID : {
				PCAN_GPS_GPS_Delusions_A.data[0] = read_frame.data[0];
				PCAN_GPS_GPS_Delusions_A.data[1] = read_frame.data[1];
				PCAN_GPS_GPS_Delusions_A.data[2] = read_frame.data[2];
				PCAN_GPS_GPS_Delusions_A.data[3] = read_frame.data[3];
				PCAN_GPS_GPS_Delusions_A.data[4] = read_frame.data[4];
				PCAN_GPS_GPS_Delusions_A.data[5] = read_frame.data[5];
				PCAN_GPS_GPS_Delusions_A.data[6] = read_frame.data[6];
				PCAN_GPS_GPS_Delusions_A.data[7] = read_frame.data[7];
				PCAN_GPS_GPS_Delusions_A_fresh = TRUE;
				can_stats.PCAN_GPS_GPS_Delusions_A_count++;
				break;
			}
			case PCAN_GPS_GPS_DELUSIONS_B_ID : {
				PCAN_GPS_GPS_Delusions_B.data[0] = read_frame.data[0];
				PCAN_GPS_GPS_Delusions_B.data[1] = read_frame.data[1];
				PCAN_GPS_GPS_Delusions_B.data[2] = read_frame.data[2];
				PCAN_GPS_GPS_Delusions_B.data[3] = read_frame.data[3];
				PCAN_GPS_GPS_Delusions_B.data[4] = read_frame.data[4];
				PCAN_GPS_GPS_Delusions_B.data[5] = read_frame.data[5];
				PCAN_GPS_GPS_Delusions_B.data[6] = read_frame.data[6];
				PCAN_GPS_GPS_Delusions_B.data[7] = read_frame.data[7];
				PCAN_GPS_GPS_Delusions_B_fresh = TRUE;
				can_stats.PCAN_GPS_GPS_Delusions_B_count++;
				break;
			}
			case PCAN_GPS_GPS_DATETIME_ID : {
				PCAN_GPS_GPS_DateTime.data[0] = read_frame.data[0];
				PCAN_GPS_GPS_DateTime.data[1] = read_frame.data[1];
				PCAN_GPS_GPS_DateTime.data[2] = read_frame.data[2];
				PCAN_GPS_GPS_DateTime.data[3] = read_frame.data[3];
				PCAN_GPS_GPS_DateTime.data[4] = read_frame.data[4];
				PCAN_GPS_GPS_DateTime.data[5] = read_frame.data[5];
				PCAN_GPS_GPS_DateTime.data[6] = read_frame.data[6];
				PCAN_GPS_GPS_DateTime.data[7] = read_frame.data[7];
				PCAN_GPS_GPS_DateTime_fresh = TRUE;
				can_stats.PCAN_GPS_GPS_DateTime_count++;
				break;
			}
			default :
			{
				// TODO: set up filters so all unhandled CAN frames don't end up here...
				can_stats.unhandled_frame_count++;
				break;
			}
		}
	
		pthread_mutex_unlock(&can_read_mutex);	// don't forget!
	}
}


int fs_ai_api_init(char* CAN_interface, int debug, int simulate) {
	static boolean_e initialised = FALSE;
	int err;

	fs_ai_api_clear_can_stats();
	
	if(initialised) {
		if(debug_mode) { printf("Already initialised...\r\n"); }
		return(EXIT_FAILURE);
	}
	
	initialised = TRUE;

	if(debug != 0) {
		debug_mode = TRUE;
		printf("Called fs_ai_api_init(%s, %d, %d)\r\n",CAN_interface,debug,simulate);
	}
	
	if(simulate != 0) {
		simulate_mode = TRUE;
		if(debug_mode) { printf("Simulate Mode enabled...\r\n"); }
	}

	if(can_init(CAN_interface) < 0) {
		if(debug_mode) { printf("Can't open [%s]", CAN_interface); }
		return(EXIT_FAILURE);
	}

	// spawn thread
	err = pthread_create(&can_read_tid, NULL, &can_read_thread, NULL);
	if(err != 0) {
		if(debug_mode) { printf("Can't create CAN read thread:[%s]", strerror(err)); }
		return(EXIT_FAILURE);
	}

	clock_gettime(CLOCK_REALTIME,&last_set);
	
	return(EXIT_SUCCESS);
}


void fs_ai_api_vcu2ai_get_data(fs_ai_api_vcu2ai *data) {
	pthread_mutex_lock(&can_read_mutex); // protect the buffers from race conditions

	// decode the CAN buffers if fresh data present
	if(VCU2AI_Status_fresh) {
		VCU2AI_Status_fresh = FALSE;
		VCU2AI_HANDSHAKE_RECEIVE_BIT	= (VCU2AI_Status.data[0] & 0b00000001);
		VCU2AI_RES_GO_SIGNAL			= (VCU2AI_Status.data[1] & 0b00001000) >> 3;
		VCU2AI_AS_STATE					= (VCU2AI_Status.data[2] & 0b00001111);
		VCU2AI_AMI_STATE				= (VCU2AI_Status.data[2] & 0b11110000) >> 4;
		// remaining fields not relevant to API
	}

	if(VCU2AI_Drive_F_fresh) {
		VCU2AI_Drive_F_fresh = FALSE;
		VCU2AI_FRONT_AXLE_TORQUE_MAX_raw		= ((uint16_t)(VCU2AI_Drive_F.data[4] + (VCU2AI_Drive_F.data[5] << 8)));
		// remaining fields not relevant to API
	}

	if(VCU2AI_Drive_R_fresh) {
		VCU2AI_Drive_R_fresh = FALSE;
		VCU2AI_REAR_AXLE_TORQUE_MAX_raw	= ((uint16_t)(VCU2AI_Drive_R.data[4] + (VCU2AI_Drive_R.data[5] << 8)));
		// remaining fields not relevant to API
	}

	if(VCU2AI_Steer_fresh) {
		VCU2AI_Steer_fresh = FALSE;
		VCU2AI_STEER_ANGLE_raw			= ((int16_t)(VCU2AI_Steer.data[0] + (VCU2AI_Steer.data[1] << 8)));
		VCU2AI_STEER_ANGLE_MAX_raw		= ((uint16_t)(VCU2AI_Steer.data[2] + (VCU2AI_Steer.data[3] << 8)));
		// remaining fields not relevant to API
	}

	if(VCU2AI_Brake_fresh) {
		VCU2AI_Brake_fresh = FALSE;
		VCU2AI_BRAKE_PRESS_F_raw		= (VCU2AI_Brake.data[0]);
		VCU2AI_BRAKE_PRESS_R_raw		= (VCU2AI_Brake.data[2]);
		// remaining fields not relevant to API
	}

	if(VCU2AI_Wheel_speeds_fresh) {
		VCU2AI_Wheel_speeds_fresh = FALSE;
		VCU2AI_FL_WHEEL_SPEED_rpm = (VCU2AI_Wheel_speeds.data[0] + (VCU2AI_Wheel_speeds.data[1] << 8));
		VCU2AI_FR_WHEEL_SPEED_rpm = (VCU2AI_Wheel_speeds.data[2] + (VCU2AI_Wheel_speeds.data[3] << 8));
		VCU2AI_RL_WHEEL_SPEED_rpm = (VCU2AI_Wheel_speeds.data[4] + (VCU2AI_Wheel_speeds.data[5] << 8));
		VCU2AI_RR_WHEEL_SPEED_rpm = (VCU2AI_Wheel_speeds.data[6] + (VCU2AI_Wheel_speeds.data[7] << 8));
	}

	if(VCU2AI_Wheel_counts_fresh) {
		VCU2AI_Wheel_counts_fresh = FALSE;
		VCU2AI_FL_PULSE_COUNT = VCU2AI_Wheel_counts.data[0] + (VCU2AI_Wheel_counts.data[1] << 8);
		VCU2AI_FR_PULSE_COUNT = VCU2AI_Wheel_counts.data[2] + (VCU2AI_Wheel_counts.data[3] << 8);
		VCU2AI_RL_PULSE_COUNT = VCU2AI_Wheel_counts.data[4] + (VCU2AI_Wheel_counts.data[5] << 8);
		VCU2AI_RR_PULSE_COUNT = VCU2AI_Wheel_counts.data[6] + (VCU2AI_Wheel_counts.data[7] << 8);
	}

	pthread_mutex_unlock(&can_read_mutex); // don't forget!
	
	if(simulate_mode) {
		static unsigned int counter = 0;
		
		if(0 == (counter % 1000)) {
			if(++VCU2AI_AS_STATE > AS_FINISHED) { VCU2AI_AS_STATE = AS_OFF; }
			if(++VCU2AI_AMI_STATE > AMI_AUTONOMOUS_DEMO) { VCU2AI_AMI_STATE = AMI_NOT_SELECTED; }
		}
		
		VCU2AI_STEER_ANGLE_MAX_raw = 272;
		if(++VCU2AI_STEER_ANGLE_raw > VCU2AI_STEER_ANGLE_MAX_raw) {
			VCU2AI_STEER_ANGLE_raw = -1*VCU2AI_STEER_ANGLE_MAX_raw;
		}

		VCU2AI_FL_WHEEL_SPEED_rpm = 999;
		VCU2AI_FR_WHEEL_SPEED_rpm = 999;
		VCU2AI_RL_WHEEL_SPEED_rpm = 999;
		VCU2AI_RR_WHEEL_SPEED_rpm = 999;

		VCU2AI_FL_PULSE_COUNT++;
		VCU2AI_FR_PULSE_COUNT++;
		VCU2AI_RL_PULSE_COUNT++;
		VCU2AI_RR_PULSE_COUNT++;

		if(debug_mode) { printf("Simulation Count:[%d]\r\n",counter); }

		counter++;
	}

	// output the data, converting where needed
	data->VCU2AI_HANDSHAKE_RECEIVE_BIT = VCU2AI_HANDSHAKE_RECEIVE_BIT;
	data->VCU2AI_RES_GO_SIGNAL = VCU2AI_RES_GO_SIGNAL;
	data->VCU2AI_AS_STATE = VCU2AI_AS_STATE;
	data->VCU2AI_AMI_STATE = VCU2AI_AMI_STATE;
	data->VCU2AI_STEER_ANGLE_deg = 0.1f*VCU2AI_STEER_ANGLE_raw;
	data->VCU2AI_BRAKE_PRESS_F_pct = 0.5f*VCU2AI_BRAKE_PRESS_F_raw;
	data->VCU2AI_BRAKE_PRESS_R_pct = 0.5f*VCU2AI_BRAKE_PRESS_R_raw;
	data->VCU2AI_FL_WHEEL_SPEED_rpm = (float)VCU2AI_FL_WHEEL_SPEED_rpm;
	data->VCU2AI_FR_WHEEL_SPEED_rpm = (float)VCU2AI_FR_WHEEL_SPEED_rpm;
	data->VCU2AI_RL_WHEEL_SPEED_rpm = (float)VCU2AI_RL_WHEEL_SPEED_rpm;
	data->VCU2AI_RR_WHEEL_SPEED_rpm = (float)VCU2AI_RR_WHEEL_SPEED_rpm;
	data->VCU2AI_FL_PULSE_COUNT = VCU2AI_FL_PULSE_COUNT;
	data->VCU2AI_FR_PULSE_COUNT = VCU2AI_FR_PULSE_COUNT;
	data->VCU2AI_RL_PULSE_COUNT = VCU2AI_RL_PULSE_COUNT;
	data->VCU2AI_RR_PULSE_COUNT = VCU2AI_RR_PULSE_COUNT;
}


void fs_ai_api_ai2vcu_set_data(fs_ai_api_ai2vcu *data) {
	// local input data buffers
	fs_ai_api_mission_status_e		t_AI2VCU_MISSION_STATUS = 0;
	fs_ai_api_direction_request_e	t_AI2VCU_DIRECTION_REQUEST = 0;
	fs_ai_api_estop_request_e		t_AI2VCU_ESTOP_REQUEST = 0;
	fs_ai_api_handshake_send_bit_e	t_AI2VCU_HANDSHAKE_SEND_BIT = 0;
	float							t_AI2VCU_STEER_ANGLE_REQUEST_deg = 0;
	float							t_AI2VCU_FRONT_MOTOR_SPEED_REQUEST_rpm = 0;
	float							t_AI2VCU_REAR_MOTOR_SPEED_REQUEST_rpm = 0;
	float							t_AI2VCU_FRONT_AXLE_TORQUE_REQUEST_Nm = 0;
	float							t_AI2VCU_REAR_AXLE_TORQUE_REQUEST_Nm = 0;
	float							t_AI2VCU_FRONT_BRAKE_PRESS_REQUEST_pct = 0;
	float							t_AI2VCU_REAR_BRAKE_PRESS_REQUEST_pct = 0;

	float t_FRONT_AXLE_TORQUE_MAX_Nm = (0.1f*VCU2AI_FRONT_AXLE_TORQUE_MAX_raw);
	float t_REAR_AXLE_TORQUE_MAX_Nm = (0.1f*VCU2AI_REAR_AXLE_TORQUE_MAX_raw);
	float t_STEER_ANGLE_MAX_deg = (0.1f*VCU2AI_STEER_ANGLE_MAX_raw);

	uint16_t t_fastest_wheel_rpm = 0;

	if(VCU2AI_FL_WHEEL_SPEED_rpm > t_fastest_wheel_rpm) { t_fastest_wheel_rpm = VCU2AI_FL_WHEEL_SPEED_rpm; }
	if(VCU2AI_FR_WHEEL_SPEED_rpm > t_fastest_wheel_rpm) { t_fastest_wheel_rpm = VCU2AI_FR_WHEEL_SPEED_rpm; }
	if(VCU2AI_RL_WHEEL_SPEED_rpm > t_fastest_wheel_rpm) { t_fastest_wheel_rpm = VCU2AI_RL_WHEEL_SPEED_rpm; }
	if(VCU2AI_RR_WHEEL_SPEED_rpm > t_fastest_wheel_rpm) { t_fastest_wheel_rpm = VCU2AI_RR_WHEEL_SPEED_rpm; }

	t_AI2VCU_MISSION_STATUS = data->AI2VCU_MISSION_STATUS;
	t_AI2VCU_DIRECTION_REQUEST = data->AI2VCU_DIRECTION_REQUEST;
	t_AI2VCU_ESTOP_REQUEST = data->AI2VCU_ESTOP_REQUEST;
	t_AI2VCU_HANDSHAKE_SEND_BIT = data->AI2VCU_HANDSHAKE_SEND_BIT;
	t_AI2VCU_STEER_ANGLE_REQUEST_deg = data->AI2VCU_STEER_ANGLE_REQUEST_deg;
	t_AI2VCU_FRONT_MOTOR_SPEED_REQUEST_rpm = data->AI2VCU_AXLE_SPEED_REQUEST_rpm * MOTOR_RATIO;
	t_AI2VCU_REAR_MOTOR_SPEED_REQUEST_rpm = data->AI2VCU_AXLE_SPEED_REQUEST_rpm * MOTOR_RATIO;
	t_AI2VCU_FRONT_AXLE_TORQUE_REQUEST_Nm = data->AI2VCU_AXLE_TORQUE_REQUEST_Nm;
	t_AI2VCU_REAR_AXLE_TORQUE_REQUEST_Nm = data->AI2VCU_AXLE_TORQUE_REQUEST_Nm;
	t_AI2VCU_FRONT_BRAKE_PRESS_REQUEST_pct = data->AI2VCU_BRAKE_PRESS_REQUEST_pct;
	t_AI2VCU_REAR_BRAKE_PRESS_REQUEST_pct = data->AI2VCU_BRAKE_PRESS_REQUEST_pct;

	// additional torque limit to maintain constant electrical power and minimise risk of over-current trip
	if(t_fastest_wheel_rpm > 700) {
		if(t_FRONT_AXLE_TORQUE_MAX_Nm > 50.0f) { t_FRONT_AXLE_TORQUE_MAX_Nm = 50.0f; }
		if(t_REAR_AXLE_TORQUE_MAX_Nm > 50.0f) { t_REAR_AXLE_TORQUE_MAX_Nm = 50.0f; }
	} else if(t_fastest_wheel_rpm > 600) {
		if(t_FRONT_AXLE_TORQUE_MAX_Nm > 85.0f) { t_FRONT_AXLE_TORQUE_MAX_Nm = 85.0f; }
		if(t_REAR_AXLE_TORQUE_MAX_Nm > 85.0f) { t_REAR_AXLE_TORQUE_MAX_Nm = 85.0f; }
	} else if(t_fastest_wheel_rpm > 500) {
		if(t_FRONT_AXLE_TORQUE_MAX_Nm > 100.0f) { t_FRONT_AXLE_TORQUE_MAX_Nm = 100.0f; }
		if(t_REAR_AXLE_TORQUE_MAX_Nm > 100.0f) { t_REAR_AXLE_TORQUE_MAX_Nm = 100.0f; }
	} else if(t_fastest_wheel_rpm > 400) {
		if(t_FRONT_AXLE_TORQUE_MAX_Nm > 120.0f) { t_FRONT_AXLE_TORQUE_MAX_Nm = 120.0f; }
		if(t_REAR_AXLE_TORQUE_MAX_Nm > 120.0f) { t_REAR_AXLE_TORQUE_MAX_Nm = 120.0f; }
	} else if(t_fastest_wheel_rpm > 300) {
		if(t_FRONT_AXLE_TORQUE_MAX_Nm > 150.0f) { t_FRONT_AXLE_TORQUE_MAX_Nm = 150.0f; }
		if(t_REAR_AXLE_TORQUE_MAX_Nm > 150.0f) { t_REAR_AXLE_TORQUE_MAX_Nm = 150.0f; }
	}

	// validate the 'float' requests
	if(t_AI2VCU_STEER_ANGLE_REQUEST_deg > t_STEER_ANGLE_MAX_deg) { t_AI2VCU_STEER_ANGLE_REQUEST_deg = t_STEER_ANGLE_MAX_deg; }
	if(t_AI2VCU_STEER_ANGLE_REQUEST_deg < (-1.0f*t_STEER_ANGLE_MAX_deg)) { t_AI2VCU_STEER_ANGLE_REQUEST_deg = (-1.0f*t_STEER_ANGLE_MAX_deg); }

	if(t_AI2VCU_FRONT_MOTOR_SPEED_REQUEST_rpm > MOTOR_MAX_RPM) { t_AI2VCU_FRONT_MOTOR_SPEED_REQUEST_rpm = MOTOR_MAX_RPM; }
	if(t_AI2VCU_FRONT_MOTOR_SPEED_REQUEST_rpm < 0.0f) { t_AI2VCU_FRONT_MOTOR_SPEED_REQUEST_rpm = 0.0f; }

	if(t_AI2VCU_REAR_MOTOR_SPEED_REQUEST_rpm > MOTOR_MAX_RPM) { t_AI2VCU_REAR_MOTOR_SPEED_REQUEST_rpm = MOTOR_MAX_RPM; }
	if(t_AI2VCU_REAR_MOTOR_SPEED_REQUEST_rpm < 0.0f) { t_AI2VCU_REAR_MOTOR_SPEED_REQUEST_rpm = 0.0f; }

	if(t_AI2VCU_FRONT_AXLE_TORQUE_REQUEST_Nm > t_FRONT_AXLE_TORQUE_MAX_Nm) { t_AI2VCU_FRONT_AXLE_TORQUE_REQUEST_Nm = t_FRONT_AXLE_TORQUE_MAX_Nm; }
	if(t_AI2VCU_FRONT_AXLE_TORQUE_REQUEST_Nm < 0.0f) { t_AI2VCU_FRONT_AXLE_TORQUE_REQUEST_Nm = 0.0f; }

	if(t_AI2VCU_REAR_AXLE_TORQUE_REQUEST_Nm > t_REAR_AXLE_TORQUE_MAX_Nm) { t_AI2VCU_REAR_AXLE_TORQUE_REQUEST_Nm = t_REAR_AXLE_TORQUE_MAX_Nm; }
	if(t_AI2VCU_REAR_AXLE_TORQUE_REQUEST_Nm < 0.0f) { t_AI2VCU_REAR_AXLE_TORQUE_REQUEST_Nm = 0.0f; }

	if(t_AI2VCU_FRONT_BRAKE_PRESS_REQUEST_pct > 100.0f) { t_AI2VCU_FRONT_BRAKE_PRESS_REQUEST_pct = 100.0f; }
	if(t_AI2VCU_FRONT_BRAKE_PRESS_REQUEST_pct < 0.0f) { t_AI2VCU_FRONT_BRAKE_PRESS_REQUEST_pct = 0.0f; }

	if(t_AI2VCU_REAR_BRAKE_PRESS_REQUEST_pct > 100.0f) { t_AI2VCU_REAR_BRAKE_PRESS_REQUEST_pct = 100.0f; }
	if(t_AI2VCU_REAR_BRAKE_PRESS_REQUEST_pct < 0.0f) { t_AI2VCU_REAR_BRAKE_PRESS_REQUEST_pct = 0.0f; }

	// validate the 'enum' requests
	if(t_AI2VCU_MISSION_STATUS < MISSION_NOT_SELECTED) { t_AI2VCU_MISSION_STATUS = MISSION_NOT_SELECTED; }
	if(t_AI2VCU_MISSION_STATUS > MISSION_FINISHED) { t_AI2VCU_MISSION_STATUS = MISSION_FINISHED; }

	if(t_AI2VCU_DIRECTION_REQUEST < DIRECTION_NEUTRAL) { t_AI2VCU_DIRECTION_REQUEST = DIRECTION_NEUTRAL; }
	if(t_AI2VCU_DIRECTION_REQUEST > DIRECTION_FORWARD) { t_AI2VCU_DIRECTION_REQUEST = DIRECTION_FORWARD; }

	if(t_AI2VCU_ESTOP_REQUEST < ESTOP_NO) { t_AI2VCU_ESTOP_REQUEST = ESTOP_NO; }
	if(t_AI2VCU_ESTOP_REQUEST > ESTOP_YES) { t_AI2VCU_ESTOP_REQUEST = ESTOP_YES; }

	if(t_AI2VCU_HANDSHAKE_SEND_BIT < HANDSHAKE_SEND_BIT_OFF) { t_AI2VCU_HANDSHAKE_SEND_BIT = HANDSHAKE_SEND_BIT_OFF; }
	if(t_AI2VCU_HANDSHAKE_SEND_BIT > HANDSHAKE_SEND_BIT_ON) { t_AI2VCU_HANDSHAKE_SEND_BIT = HANDSHAKE_SEND_BIT_ON; }

	// braking has priority over torque - reject implausible inputs
	if((t_AI2VCU_FRONT_BRAKE_PRESS_REQUEST_pct > 0.0f) || (t_AI2VCU_REAR_BRAKE_PRESS_REQUEST_pct > 0.0f)) {
		t_AI2VCU_FRONT_AXLE_TORQUE_REQUEST_Nm = 0.0f;
		t_AI2VCU_REAR_AXLE_TORQUE_REQUEST_Nm = 0.0f;
	}

	// set requests, converting where needed
	AI2VCU_HANDSHAKE_BIT = t_AI2VCU_HANDSHAKE_SEND_BIT;
	AI2VCU_ESTOP_REQUEST = t_AI2VCU_ESTOP_REQUEST;
	AI2VCU_MISSION_STATUS = t_AI2VCU_MISSION_STATUS;
	AI2VCU_DIRECTION_REQUEST = t_AI2VCU_DIRECTION_REQUEST;
	AI2VCU_LAP_COUNTER = 0;
	AI2VCU_CONES_COUNT_ACTUAL = 0;
	AI2VCU_CONES_COUNT_ALL = 0;

	AI2VCU_STEER_REQUEST_raw = (int16_t)(10.0f*t_AI2VCU_STEER_ANGLE_REQUEST_deg);
	AI2VCU_FRONT_MOTOR_SPEED_MAX_rpm = (uint16_t)t_AI2VCU_FRONT_MOTOR_SPEED_REQUEST_rpm;
	AI2VCU_REAR_MOTOR_SPEED_MAX_rpm = (uint16_t)t_AI2VCU_REAR_MOTOR_SPEED_REQUEST_rpm;
	AI2VCU_FRONT_AXLE_TRQ_REQUEST_raw = (uint16_t)(10.0f*t_AI2VCU_FRONT_AXLE_TORQUE_REQUEST_Nm);
	AI2VCU_REAR_AXLE_TRQ_REQUEST_raw = (uint16_t)(10.0f*t_AI2VCU_REAR_AXLE_TORQUE_REQUEST_Nm);
	AI2VCU_HYD_PRESS_F_REQ_raw = (uint8_t)(2.0f*t_AI2VCU_FRONT_BRAKE_PRESS_REQUEST_pct);
	AI2VCU_HYD_PRESS_R_REQ_raw = (uint8_t)(2.0f*t_AI2VCU_REAR_BRAKE_PRESS_REQUEST_pct);

	// load the CAN frames with the validated data
	volatile pack_16_t temp;

	AI2VCU_Status.data[0] = (uint8_t)(AI2VCU_HANDSHAKE_BIT & 0x01);
	AI2VCU_Status.data[1] = (uint8_t)(((AI2VCU_DIRECTION_REQUEST & 0x03) << 6) + ((AI2VCU_MISSION_STATUS & 0x03) << 4) + (AI2VCU_ESTOP_REQUEST & 0x01));
	AI2VCU_Status.data[2] = (AI2VCU_LAP_COUNTER & 0x0F);
	AI2VCU_Status.data[3] = AI2VCU_CONES_COUNT_ACTUAL;
	AI2VCU_Status.data[4] = (uint8_t)(AI2VCU_CONES_COUNT_ALL & 0x00FF);
	AI2VCU_Status.data[5] = (uint8_t)((AI2VCU_CONES_COUNT_ALL & 0xFF00) >> 8);
	AI2VCU_Status.data[6] = 0;
	AI2VCU_Status.data[7] = 0;

	temp.uword = AI2VCU_FRONT_AXLE_TRQ_REQUEST_raw;
	AI2VCU_Drive_F.data[0] = temp.bytes[0];
	AI2VCU_Drive_F.data[1] = temp.bytes[1];
	temp.uword = AI2VCU_FRONT_MOTOR_SPEED_MAX_rpm;	
	AI2VCU_Drive_F.data[2] = temp.bytes[0];
	AI2VCU_Drive_F.data[3] = temp.bytes[1];
	AI2VCU_Drive_F.data[4] = 0;
	AI2VCU_Drive_F.data[5] = 0;
	AI2VCU_Drive_F.data[6] = 0;
	AI2VCU_Drive_F.data[7] = 0;

	temp.uword = AI2VCU_REAR_AXLE_TRQ_REQUEST_raw;
	AI2VCU_Drive_R.data[0] = temp.bytes[0];
	AI2VCU_Drive_R.data[1] = temp.bytes[1];
	temp.uword = AI2VCU_REAR_MOTOR_SPEED_MAX_rpm;
	AI2VCU_Drive_R.data[2] = temp.bytes[0];
	AI2VCU_Drive_R.data[3] = temp.bytes[1];
	AI2VCU_Drive_R.data[4] = 0;
	AI2VCU_Drive_R.data[5] = 0;
	AI2VCU_Drive_R.data[6] = 0;
	AI2VCU_Drive_R.data[7] = 0;

	temp.sword = AI2VCU_STEER_REQUEST_raw;
	AI2VCU_Steer.data[0] = temp.bytes[0];
	AI2VCU_Steer.data[1] = temp.bytes[1];
	AI2VCU_Steer.data[2] = 0;
	AI2VCU_Steer.data[3] = 0;
	AI2VCU_Steer.data[4] = 0;
	AI2VCU_Steer.data[5] = 0;
	AI2VCU_Steer.data[6] = 0;
	AI2VCU_Steer.data[7] = 0;

	AI2VCU_Brake.data[0] = AI2VCU_HYD_PRESS_F_REQ_raw;
	AI2VCU_Brake.data[1] = AI2VCU_HYD_PRESS_R_REQ_raw;
	AI2VCU_Brake.data[2] = 0;
	AI2VCU_Brake.data[3] = 0;
	AI2VCU_Brake.data[4] = 0;
	AI2VCU_Brake.data[5] = 0;
	AI2VCU_Brake.data[6] = 0;
	AI2VCU_Brake.data[7] = 0;

	clock_gettime(CLOCK_REALTIME,&this_set);

	long int interval_ns = ((this_set.tv_sec-last_set.tv_sec)* 1000000000) + (this_set.tv_nsec-last_set.tv_nsec);
	
	if(interval_ns > 8000000) // enforce maximum call rate of approx. 8ms
	{
		// send the CAN frames
		can_send(&AI2VCU_Status);
		can_send(&AI2VCU_Drive_F);
		can_send(&AI2VCU_Drive_R);
		can_send(&AI2VCU_Steer);
		can_send(&AI2VCU_Brake);
		clock_gettime(CLOCK_REALTIME,&last_set);
	}
}


void fs_ai_api_imu_get_data(fs_ai_api_imu *data) {
	pthread_mutex_lock(&can_read_mutex); // protect the buffers from race conditions

	can_data_t* temp;

	// decode the CAN buffers if fresh data present
	if(PCAN_GPS_BMC_Acceleration_fresh) {
		PCAN_GPS_BMC_Acceleration_fresh = FALSE;
		temp = (can_data_t*)&PCAN_GPS_BMC_Acceleration.data[0];
		IMU_Acceleration_X_mG = 3.91f*temp->swords[0];
		IMU_Acceleration_Y_mG = 3.91f*temp->swords[1];
		IMU_Acceleration_Z_mG = 3.91f*temp->swords[2];
		IMU_Temperature_degC = 0.5f*temp->sbytes[6];
		IMU_VerticalAxis = temp->sbytes[7] & 0b00000011;
		IMU_Orientation = (temp->sbytes[7] & 0b00011100) >> 2;
	}

	if(PCAN_GPS_BMC_MagneticField_fresh) {
		PCAN_GPS_BMC_MagneticField_fresh = FALSE;
		temp = (can_data_t*)&PCAN_GPS_BMC_MagneticField.data[0];
		IMU_MagneticField_X_uT = 0.3f*temp->swords[0];
		IMU_MagneticField_Y_uT = 0.3f*temp->swords[1];
		IMU_MagneticField_Z_uT = 0.3f*temp->swords[2];
	}

	if(PCAN_GPS_L3GD20_Rotation_A_fresh) {
		PCAN_GPS_L3GD20_Rotation_A_fresh = FALSE;
		temp = (can_data_t*)&PCAN_GPS_L3GD20_Rotation_A.data[0];
		IMU_Rotation_X_degps = temp->floats[0];
		IMU_Rotation_Y_degps = temp->floats[1];
	}

	if(PCAN_GPS_L3GD20_Rotation_B_fresh) {
		temp = (can_data_t*)&PCAN_GPS_L3GD20_Rotation_B.data[0];
		PCAN_GPS_L3GD20_Rotation_B_fresh = FALSE;
		IMU_Rotation_Z_degps = temp->floats[0];
	}

	pthread_mutex_unlock(&can_read_mutex); // don't forget!

	// output the data, conversions done above
	data->IMU_Acceleration_X_mG = IMU_Acceleration_X_mG;
	data->IMU_Acceleration_Y_mG = IMU_Acceleration_Y_mG;
	data->IMU_Acceleration_Z_mG = IMU_Acceleration_Z_mG;
	data->IMU_Temperature_degC = IMU_Temperature_degC;
	data->IMU_VerticalAxis = IMU_VerticalAxis;
	data->IMU_Orientation = IMU_Orientation;
	data->IMU_MagneticField_X_uT = IMU_MagneticField_X_uT;
	data->IMU_MagneticField_Y_uT = IMU_MagneticField_Y_uT;
	data->IMU_MagneticField_Z_uT = IMU_MagneticField_Z_uT;
	data->IMU_Rotation_X_degps = IMU_Rotation_X_degps;
	data->IMU_Rotation_Y_degps = IMU_Rotation_Y_degps;
	data->IMU_Rotation_Z_degps = IMU_Rotation_Z_degps;
}


void fs_ai_api_gps_get_data(fs_ai_api_gps *data) {
	pthread_mutex_lock(&can_read_mutex); // protect the buffers from race conditions

	can_data_t* temp;

	// decode the CAN buffers if fresh data present
	if(PCAN_GPS_GPS_Status_fresh) {
		PCAN_GPS_GPS_Status_fresh = FALSE;
		GPS_AntennaStatus = PCAN_GPS_GPS_Status.data[0];
		GPS_NumSatellites = PCAN_GPS_GPS_Status.data[1];
		GPS_NavigationMethod = PCAN_GPS_GPS_Status.data[2];
	}

	if(PCAN_GPS_GPS_CourseSpeed_fresh) {
		PCAN_GPS_GPS_CourseSpeed_fresh = FALSE;
		temp = (can_data_t*)&PCAN_GPS_GPS_CourseSpeed.data[0];
		GPS_Course_deg = temp->floats[0];
		GPS_Speed_kmh = temp->floats[1];	}

	if(PCAN_GPS_GPS_Longitude_fresh) {
		PCAN_GPS_GPS_Longitude_fresh = FALSE;
		temp = (can_data_t*)&PCAN_GPS_GPS_Longitude.data[0];
		GPS_Longitude_Minutes = temp->floats[0];
		GPS_Longitude_Degree = temp->uwords[2];
		GPS_Longitude_IndicatorEW = temp->ubytes[6];
	}

	if(PCAN_GPS_GPS_Latitude_fresh) {
		PCAN_GPS_GPS_Latitude_fresh = FALSE;
		temp = (can_data_t*)&PCAN_GPS_GPS_Latitude.data[0];
		GPS_Latitude_Minutes = temp->floats[0];
		GPS_Latitude_Degree = temp->uwords[2];
		GPS_Latitude_IndicatorNS = temp->ubytes[6];
	}

	if(PCAN_GPS_GPS_Altitude_fresh) {
		PCAN_GPS_GPS_Altitude_fresh = FALSE;
		temp = (can_data_t*)&PCAN_GPS_GPS_Altitude.data[0];
		GPS_Altitude = temp->floats[0];
	}

	if(PCAN_GPS_GPS_Delusions_A_fresh) {
		PCAN_GPS_GPS_Delusions_A_fresh = FALSE;
		temp = (can_data_t*)&PCAN_GPS_GPS_Delusions_A.data[0];
		GPS_PDOP = temp->floats[0];
		GPS_HDOP = temp->floats[0];
	}

	if(PCAN_GPS_GPS_Delusions_B_fresh) {
		PCAN_GPS_GPS_Delusions_B_fresh = FALSE;
		temp = (can_data_t*)&PCAN_GPS_GPS_Delusions_B.data[0];
		GPS_VDOP = temp->floats[0];
	}

	if(PCAN_GPS_GPS_DateTime_fresh) {
		PCAN_GPS_GPS_DateTime_fresh = FALSE;
		GPS_UTC_Year = PCAN_GPS_GPS_DateTime.data[0];
		GPS_UTC_Month = PCAN_GPS_GPS_DateTime.data[1];
		GPS_UTC_DayOfMonth = PCAN_GPS_GPS_DateTime.data[2];
		GPS_UTC_Hour = PCAN_GPS_GPS_DateTime.data[3];
		GPS_UTC_Minute = PCAN_GPS_GPS_DateTime.data[4];
		GPS_UTC_Second = PCAN_GPS_GPS_DateTime.data[5];
	}

	pthread_mutex_unlock(&can_read_mutex); // don't forget!

	// output the data, no conversions needed
	data->GPS_AntennaStatus = GPS_AntennaStatus;
	data->GPS_NumSatellites = GPS_NumSatellites;
	data->GPS_NavigationMethod = GPS_NavigationMethod;
	data->GPS_Course_deg = GPS_Course_deg;
	data->GPS_Speed_kmh = GPS_Speed_kmh;
	data->GPS_Longitude_Minutes = GPS_Longitude_Minutes;
	data->GPS_Longitude_Degree = GPS_Longitude_Degree;
	data->GPS_Longitude_IndicatorEW = GPS_Longitude_IndicatorEW;
	data->GPS_Latitude_Minutes = GPS_Latitude_Minutes;
	data->GPS_Latitude_Degree = GPS_Latitude_Degree;
	data->GPS_Latitude_IndicatorNS = GPS_Latitude_IndicatorNS;
	data->GPS_Altitude = GPS_Altitude;
	data->GPS_PDOP = GPS_PDOP;
	data->GPS_HDOP = GPS_HDOP;
	data->GPS_VDOP = GPS_VDOP;
	data->GPS_UTC_Year = GPS_UTC_Year;
	data->GPS_UTC_Month = GPS_UTC_Month;
	data->GPS_UTC_DayOfMonth = GPS_UTC_DayOfMonth;
	data->GPS_UTC_Hour = GPS_UTC_Hour;
	data->GPS_UTC_Minute = GPS_UTC_Minute;
	data->GPS_UTC_Second = GPS_UTC_Second;
}


void fs_ai_api_get_can_stats(can_stats_t *data) {
	memcpy(data,&can_stats,sizeof(can_stats_t));
}


void fs_ai_api_clear_can_stats() {
	memset(&can_stats, 0, sizeof(can_stats_t));
}
