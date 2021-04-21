/*********************************************************************
 * Copyright: Preston EV Limited 2018, Rockfort Engineering Ltd. 2019
 * 
 * File:	fs-ai_api.c
 * Author:	Ian Murphy
 * Date:	2018-06-25, 2019-05-14
 * 
 ********************************************************************/


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

typedef struct pack_int16_t {
	union {
		volatile int16_t word;
		volatile uint8_t bytes[2];
	};
} pack_int16_t;


// statics
static boolean_e debug_mode = FALSE;
static boolean_e simulate_mode = FALSE;

static volatile uint32_t loop_count = 0;

static const float MOTOR_RATIO = 3.5f;
static const float MOTOR_MAX_RPM = 4000.0f;

static const float ACCEL_X_OFFSET = 0.0f;
static const float ACCEL_Y_OFFSET = -9.80665f;
static const float ACCEL_Z_OFFSET = 0.0f;
static const float GYRO_X_OFFSET = -1.63f;
static const float GYRO_Y_OFFSET = +0.38f;
static const float GYRO_Z_OFFSET = +0.43f;

static pthread_t can_read_tid;
static pthread_mutex_t can_read_mutex = PTHREAD_MUTEX_INITIALIZER;

static struct timespec last_set, this_set;


// tx frames
static struct can_frame AI2LOG_Dynamics2	= {0x501,6};
static struct can_frame AI2VCU_Status		= {0x510,8};
static struct can_frame AI2VCU_Drive_F		= {0x511,8};
static struct can_frame AI2VCU_Drive_R		= {0x512,8};
static struct can_frame AI2VCU_Steer		= {0x513,8};
static struct can_frame AI2VCU_Brake		= {0x514,8};


// rx frames
static struct can_frame VCU2AI_Status;
static struct can_frame VCU2AI_Drive_F;
static struct can_frame VCU2AI_Drive_R;
static struct can_frame VCU2AI_Steer;
static struct can_frame VCU2AI_Brake;
static struct can_frame VCU2AI_Wheel_speeds;
static struct can_frame VCU2AI_Wheel_counts;
static struct can_frame PCAN_GPS_Accels;
static struct can_frame PCAN_GPS_Gyro;
static struct can_frame DEBUG_1;

static volatile uint16_t VCU2AI_Status_count = 0;
static volatile uint16_t VCU2AI_Drive_F_count = 0;
static volatile uint16_t VCU2AI_Drive_R_count = 0;
static volatile uint16_t VCU2AI_Steer_count = 0;
static volatile uint16_t VCU2AI_Brake_count = 0;
static volatile uint16_t VCU2AI_Wheel_speeds_count = 0;
static volatile uint16_t VCU2AI_Wheel_counts_count = 0;
static volatile uint16_t PCAN_GPS_Accels_count = 0;
static volatile uint16_t PCAN_GPS_Gyro_count = 0;
static volatile uint16_t DEBUG_1_count = 0;

static volatile boolean_e VCU2AI_Status_fresh = FALSE;
static volatile boolean_e VCU2AI_Drive_F_fresh = FALSE;
static volatile boolean_e VCU2AI_Drive_R_fresh = FALSE;
static volatile boolean_e VCU2AI_Steer_fresh = FALSE;
static volatile boolean_e VCU2AI_Brake_fresh = FALSE;
static volatile boolean_e VCU2AI_Wheel_speeds_fresh = FALSE;
static volatile boolean_e VCU2AI_Wheel_counts_fresh = FALSE;
static volatile boolean_e PCAN_GPS_Accels_fresh = FALSE;
static volatile boolean_e PCAN_GPS_Gyro_fresh = FALSE;
static volatile boolean_e DEBUG_1_fresh = FALSE;


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
static volatile int16_t AI2VCU_FRONT_AXLE_TRQ_REQUEST_Nm = 0;
static volatile uint16_t AI2VCU_FRONT_MOTOR_SPEED_MAX_rpm = 0;

// AI2VCU_Drive_R
static volatile int16_t AI2VCU_REAR_AXLE_TRQ_REQUEST_Nm = 0;
static volatile uint16_t AI2VCU_REAR_MOTOR_SPEED_MAX_rpm = 0;

// AI2VCU_Steer
static volatile int16_t AI2VCU_STEER_REQUEST_deg = 0;

// AI2VCU_Brake
static volatile uint8_t AI2VCU_HYD_PRESSURE_REQUEST_pct = 0;


// VCU2AI_Status
static volatile fs_ai_api_handshake_receive_bit_e	VCU2AI_HANDSHAKE_RECEIVE_BIT = HANDSHAKE_RECEIVE_BIT_OFF;
static volatile boolean_e							VCU2AI_SHUTDOWN_REQUEST = 0;	// TODO: remove?
static volatile boolean_e							VCU2AI_AS_SWITCH_STATUS = 0;
static volatile boolean_e							VCU2AI_TS_SWITCH_STATUS = 0;
static volatile boolean_e							VCU2AI_RES_GO_SIGNAL = 0;
static volatile uint8_t								VCU2AI_STEERING_STATE = 0;		// TODO: replace with STATUS
static volatile fs_ai_api_as_state_e				VCU2AI_AS_STATE = AS_OFF;
static volatile fs_ai_api_ami_state_e				VCU2AI_AMI_STATE = AMI_NOT_SELECTED;
static volatile boolean_e							VCU2AI_FAULT_STATUS = 0;
static volatile boolean_e							VCU2AI_WARNING_STATUS = 0;
static volatile uint8_t								VCU2AI_WARNING_CAUSE = 0;		// TODO: replace with flags
static volatile uint8_t								VCU2AI_SHUTDOWN_CAUSE = 0;		// TODO: replace with flags

// VCU2AI_Drive_F
static volatile int16_t		VCU2AI_FRONT_AXLE_TORQUE_Nm = 0;
static volatile int16_t		VCU2AI_FRONT_AXLE_TRQ_REQUEST_Nm = 0;
static volatile uint16_t	VCU2AI_FRONT_AXLE_TORQUE_MAX_Nm = 0;
static volatile int16_t		VCU2AI_FRONT_AXLE_TORQUE_MIN_Nm = 0;

// VCU2AI_Drive_R
static volatile int16_t		VCU2AI_REAR_AXLE_TORQUE_Nm = 0;
static volatile int16_t		VCU2AI_REAR_AXLE_TRQ_REQUEST_Nm = 0;
static volatile uint16_t	VCU2AI_REAR_AXLE_TORQUE_MAX_Nm = 0;
static volatile int16_t		VCU2AI_REAR_AXLE_TORQUE_MIN_Nm = 0;

// VCU2AI_Steer
static volatile int16_t		VCU2AI_STEER_ANGLE_deg = 0;
static volatile uint16_t	VCU2AI_STEER_ANGLE_MAX_deg = 0;
static volatile int16_t		VCU2AI_STEER_ANGLE_REQUEST_deg = 0;

// VCU2AI_Brake
static volatile uint8_t	VCU2AI_HYD_PRESSURE_pct = 0;
static volatile uint8_t	VCU2AI_HYD_PRESSURE_REQUEST_pct = 0;

// VCU2AI_Wheel_speeds
static volatile uint16_t VCU2AI_FL_WHEEL_SPEED_rpm = 0;
static volatile uint16_t VCU2AI_FR_WHEEL_SPEED_rpm = 0;
static volatile uint16_t VCU2AI_RL_WHEEL_SPEED_rpm = 0;
static volatile uint16_t VCU2AI_RR_WHEEL_SPEED_rpm = 0;

// VCU2AI_Wheel_counts
static volatile uint16_t VCU2AI_FL_PULSE_COUNT = 0;
static volatile uint16_t VCU2AI_FR_PULSE_COUNT = 0;
static volatile uint16_t VCU2AI_RL_PULSE_COUNT = 0;
static volatile uint16_t VCU2AI_RR_PULSE_COUNT = 0;

// PCAN-GPS
static volatile int16_t VCU2AI_Accel_X = 0.0;
static volatile int16_t VCU2AI_Accel_Y = 0.0;
static volatile int16_t VCU2AI_Accel_Z = 0.0;
static volatile float VCU2AI_Gyro_X = 0.0f;
static volatile float VCU2AI_Gyro_Y = 0.0f;
static volatile float VCU2AI_Gyro_Z = 0.0f;


static void *can_read_thread() {
	struct can_frame read_frame;
	
	while(1)
	{
		if(can_read(&read_frame) < 0) {
			if(debug_mode) { printf("CAN read error!\r\n"); }
		}

		pthread_mutex_lock(&can_read_mutex); // protect the buffers from race conditions

		switch(read_frame.can_id) {
			case 0x520 : {	// VCU2AI_Status
				VCU2AI_Status.data[0] = read_frame.data[0];
				VCU2AI_Status.data[1] = read_frame.data[1];
				VCU2AI_Status.data[2] = read_frame.data[2];
				VCU2AI_Status.data[3] = read_frame.data[3];
				VCU2AI_Status.data[4] = read_frame.data[4];
				VCU2AI_Status.data[5] = read_frame.data[5];
				VCU2AI_Status.data[6] = read_frame.data[6];
				VCU2AI_Status.data[7] = read_frame.data[7];
				VCU2AI_Status_fresh = TRUE;
				VCU2AI_Status_count++;
				break;
			}
			case 0x521 : {	// VCU2AI_Drive_F
				VCU2AI_Drive_F.data[0] = read_frame.data[0];
				VCU2AI_Drive_F.data[1] = read_frame.data[1];
				VCU2AI_Drive_F.data[2] = read_frame.data[2];
				VCU2AI_Drive_F.data[3] = read_frame.data[3];
				VCU2AI_Drive_F.data[4] = read_frame.data[4];
				VCU2AI_Drive_F.data[5] = read_frame.data[5];
				VCU2AI_Drive_F.data[6] = read_frame.data[6];
				VCU2AI_Drive_F.data[7] = read_frame.data[7];
				VCU2AI_Drive_F_fresh = TRUE;
				VCU2AI_Drive_F_count++;
				break;
			}
			case 0x522 : {	// VCU2AI_Drive_R
				VCU2AI_Drive_R.data[0] = read_frame.data[0];
				VCU2AI_Drive_R.data[1] = read_frame.data[1];
				VCU2AI_Drive_R.data[2] = read_frame.data[2];
				VCU2AI_Drive_R.data[3] = read_frame.data[3];
				VCU2AI_Drive_R.data[4] = read_frame.data[4];
				VCU2AI_Drive_R.data[5] = read_frame.data[5];
				VCU2AI_Drive_R.data[6] = read_frame.data[6];
				VCU2AI_Drive_R.data[7] = read_frame.data[7];
				VCU2AI_Drive_R_fresh = TRUE;
				VCU2AI_Drive_R_count++;
				break;
			}
			case 0x523 : {	// VCU2AI_Steer
				VCU2AI_Steer.data[0] = read_frame.data[0];
				VCU2AI_Steer.data[1] = read_frame.data[1];
				VCU2AI_Steer.data[2] = read_frame.data[2];
				VCU2AI_Steer.data[3] = read_frame.data[3];
				VCU2AI_Steer.data[4] = read_frame.data[4];
				VCU2AI_Steer.data[5] = read_frame.data[5];
				VCU2AI_Steer.data[6] = read_frame.data[6];
				VCU2AI_Steer.data[7] = read_frame.data[7];
				VCU2AI_Steer_fresh = TRUE;
				VCU2AI_Steer_count++;
				break;
			}
			case 0x524 : {	// VCU2AI_Brake
				VCU2AI_Brake.data[0] = read_frame.data[0];
				VCU2AI_Brake.data[1] = read_frame.data[1];
				VCU2AI_Brake.data[2] = read_frame.data[2];
				VCU2AI_Brake.data[3] = read_frame.data[3];
				VCU2AI_Brake.data[4] = read_frame.data[4];
				VCU2AI_Brake.data[5] = read_frame.data[5];
				VCU2AI_Brake.data[6] = read_frame.data[6];
				VCU2AI_Brake.data[7] = read_frame.data[7];
				VCU2AI_Brake_fresh = TRUE;
				VCU2AI_Brake_count++;
				break;
			}
			case 0x525 : {	// VCU2AI_Wheel_speeds
				VCU2AI_Wheel_speeds.data[0] = read_frame.data[0];
				VCU2AI_Wheel_speeds.data[1] = read_frame.data[1];
				VCU2AI_Wheel_speeds.data[2] = read_frame.data[2];
				VCU2AI_Wheel_speeds.data[3] = read_frame.data[3];
				VCU2AI_Wheel_speeds.data[4] = read_frame.data[4];
				VCU2AI_Wheel_speeds.data[5] = read_frame.data[5];
				VCU2AI_Wheel_speeds.data[6] = read_frame.data[6];
				VCU2AI_Wheel_speeds.data[7] = read_frame.data[7];
				VCU2AI_Wheel_speeds_fresh = TRUE;
				VCU2AI_Wheel_speeds_count++;
				break;
			}
			case 0x526 : {	// VCU2AI_Wheel_counts
				VCU2AI_Wheel_counts.data[0] = read_frame.data[0];
				VCU2AI_Wheel_counts.data[1] = read_frame.data[1];
				VCU2AI_Wheel_counts.data[2] = read_frame.data[2];
				VCU2AI_Wheel_counts.data[3] = read_frame.data[3];
				VCU2AI_Wheel_counts.data[4] = read_frame.data[4];
				VCU2AI_Wheel_counts.data[5] = read_frame.data[5];
				VCU2AI_Wheel_counts.data[6] = read_frame.data[6];
				VCU2AI_Wheel_counts.data[7] = read_frame.data[7];
				VCU2AI_Wheel_counts_fresh = TRUE;
				VCU2AI_Wheel_counts_count++;
				break;
			}
			case 0x620 : {	// PCAN_GPS_Accels
				PCAN_GPS_Accels.data[0] = read_frame.data[0];
				PCAN_GPS_Accels.data[1] = read_frame.data[1];
				PCAN_GPS_Accels.data[2] = read_frame.data[2];
				PCAN_GPS_Accels.data[3] = read_frame.data[3];
				PCAN_GPS_Accels.data[4] = read_frame.data[4];
				PCAN_GPS_Accels.data[5] = read_frame.data[5];
				PCAN_GPS_Accels.data[6] = read_frame.data[6];
				PCAN_GPS_Accels.data[7] = read_frame.data[7];
				PCAN_GPS_Accels_fresh = TRUE;
				PCAN_GPS_Accels_count++;
				break;
			}
			case 0x622 : {	// PCAN_GPS_GyroXY
				PCAN_GPS_Gyro.data[0] = read_frame.data[0];
				PCAN_GPS_Gyro.data[1] = read_frame.data[1];
				PCAN_GPS_Gyro.data[2] = read_frame.data[2];
				PCAN_GPS_Gyro.data[3] = read_frame.data[3];
				PCAN_GPS_Gyro.data[4] = read_frame.data[4];
				PCAN_GPS_Gyro.data[5] = read_frame.data[5];
				PCAN_GPS_Gyro.data[6] = read_frame.data[6];
				PCAN_GPS_Gyro.data[7] = read_frame.data[7];
				PCAN_GPS_Gyro_fresh = TRUE;
				PCAN_GPS_Gyro_count++;
				break;
			}
			case 0x150 : {	// DEBUG_1
				DEBUG_1.data[0] = read_frame.data[0];
				DEBUG_1.data[1] = read_frame.data[1];
				DEBUG_1.data[2] = read_frame.data[2];
				DEBUG_1.data[3] = read_frame.data[3];
				DEBUG_1.data[4] = read_frame.data[4];
				DEBUG_1.data[5] = read_frame.data[5];
				DEBUG_1.data[6] = read_frame.data[6];
				DEBUG_1.data[7] = read_frame.data[7];
				DEBUG_1_fresh = TRUE;
				DEBUG_1_count++;
				break;
			}
			default :
			{
				// TODO: set up filters so all unhandled CAN frames don't end up here...
				break;
			}
		}
	
		pthread_mutex_unlock(&can_read_mutex);	// don't forget!
	}
}


int fs_ai_api_init(char* CAN_interface, int debug, int simulate) {
	static boolean_e initialised = FALSE;
	int err;
	
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
		VCU2AI_SHUTDOWN_REQUEST			= (VCU2AI_Status.data[1] & 0b00000001);
		VCU2AI_AS_SWITCH_STATUS			= (VCU2AI_Status.data[1] & 0b00000010) >> 1;
		VCU2AI_TS_SWITCH_STATUS			= (VCU2AI_Status.data[1] & 0b00000100) >> 2;
		VCU2AI_RES_GO_SIGNAL			= (VCU2AI_Status.data[1] & 0b00001000) >> 3;
		VCU2AI_STEERING_STATE			= (VCU2AI_Status.data[1] & 0b00110000) >> 4;
		VCU2AI_AS_STATE					= (VCU2AI_Status.data[2] & 0b00001111);
		VCU2AI_AMI_STATE				= (VCU2AI_Status.data[2] & 0b11110000) >> 4;
		VCU2AI_FAULT_STATUS				= (VCU2AI_Status.data[3] & 0b00000001);
		VCU2AI_WARNING_STATUS			= (VCU2AI_Status.data[3] & 0b00000010) >> 1;
		VCU2AI_WARNING_CAUSE			= (VCU2AI_Status.data[4] & 0b00001111);
		VCU2AI_SHUTDOWN_CAUSE			= VCU2AI_Status.data[5];
	}

	if(VCU2AI_Drive_F_fresh) {
		VCU2AI_Drive_F_fresh = FALSE;
		VCU2AI_FRONT_AXLE_TORQUE_Nm			= ((int16_t)(VCU2AI_Drive_F.data[0] + (VCU2AI_Drive_F.data[1] << 8)));
		VCU2AI_FRONT_AXLE_TRQ_REQUEST_Nm	= ((int16_t)(VCU2AI_Drive_F.data[2] + (VCU2AI_Drive_F.data[3] << 8)));
		VCU2AI_FRONT_AXLE_TORQUE_MAX_Nm		= ((uint16_t)(VCU2AI_Drive_F.data[4] + (VCU2AI_Drive_F.data[5] << 8)));
		VCU2AI_FRONT_AXLE_TORQUE_MIN_Nm		= ((int16_t)(VCU2AI_Drive_F.data[6] + (VCU2AI_Drive_F.data[7] << 8)));
	}

	if(VCU2AI_Drive_R_fresh) {
		VCU2AI_Drive_R_fresh = FALSE;
		VCU2AI_REAR_AXLE_TORQUE_Nm		= ((int16_t)(VCU2AI_Drive_R.data[0] + (VCU2AI_Drive_R.data[1] << 8)));
		VCU2AI_REAR_AXLE_TRQ_REQUEST_Nm	= ((int16_t)(VCU2AI_Drive_R.data[2] + (VCU2AI_Drive_R.data[3] << 8)));
		VCU2AI_REAR_AXLE_TORQUE_MAX_Nm	= ((uint16_t)(VCU2AI_Drive_R.data[4] + (VCU2AI_Drive_R.data[5] << 8)));
		VCU2AI_REAR_AXLE_TORQUE_MIN_Nm	= ((int16_t)(VCU2AI_Drive_R.data[6] + (VCU2AI_Drive_R.data[7] << 8)));
	}

	if(VCU2AI_Steer_fresh) {
		VCU2AI_Steer_fresh = FALSE;
		VCU2AI_STEER_ANGLE_deg			= ((int16_t)(VCU2AI_Steer.data[0] + (VCU2AI_Steer.data[1] << 8)));
		VCU2AI_STEER_ANGLE_MAX_deg		= ((uint16_t)(VCU2AI_Steer.data[2] + (VCU2AI_Steer.data[3] << 8)));
		VCU2AI_STEER_ANGLE_REQUEST_deg	= ((int16_t)(VCU2AI_Steer.data[4] + (VCU2AI_Steer.data[5] << 8)));
	}

	if(VCU2AI_Brake_fresh) {
		VCU2AI_Brake_fresh = FALSE;
		VCU2AI_HYD_PRESSURE_pct			= (VCU2AI_Brake.data[0]);
		VCU2AI_HYD_PRESSURE_REQUEST_pct	= (VCU2AI_Brake.data[1]);
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

	if(PCAN_GPS_Accels_fresh) {
		PCAN_GPS_Accels_fresh = FALSE;
		VCU2AI_Accel_X = ((int16_t)(PCAN_GPS_Accels.data[0] + (PCAN_GPS_Accels.data[1] << 8)));
		VCU2AI_Accel_Y = ((int16_t)(PCAN_GPS_Accels.data[2] + (PCAN_GPS_Accels.data[3] << 8)));
		VCU2AI_Accel_Z = ((int16_t)(PCAN_GPS_Accels.data[4] + (PCAN_GPS_Accels.data[5] << 8)));
	}

	if(PCAN_GPS_Gyro_fresh) {
		PCAN_GPS_Gyro_fresh = FALSE;
		VCU2AI_Gyro_X = ((int16_t)(PCAN_GPS_Gyro.data[0] + (PCAN_GPS_Gyro.data[1] << 8)));
		VCU2AI_Gyro_Y = ((int16_t)(PCAN_GPS_Gyro.data[2] + (PCAN_GPS_Gyro.data[3] << 8)));
		VCU2AI_Gyro_Z = ((int16_t)(PCAN_GPS_Gyro.data[4] + (PCAN_GPS_Gyro.data[5] << 8)));
	}

	pthread_mutex_unlock(&can_read_mutex); // don't forget!

	
	if(simulate_mode) {
		static unsigned int counter = 0;
		
		if(0 == (counter % 1000)) {
			if(++VCU2AI_AS_STATE > AS_FINISHED) { VCU2AI_AS_STATE = AS_OFF; }
			if(++VCU2AI_AMI_STATE > AMI_MANUAL) { VCU2AI_AMI_STATE = AMI_NOT_SELECTED; }
		}
		
		VCU2AI_STEER_ANGLE_MAX_deg = 272;
		if(++VCU2AI_STEER_ANGLE_deg > VCU2AI_STEER_ANGLE_MAX_deg) {
			VCU2AI_STEER_ANGLE_deg = -1*VCU2AI_STEER_ANGLE_MAX_deg;
		}
		
		VCU2AI_FL_PULSE_COUNT++;
		VCU2AI_FR_PULSE_COUNT++;
		VCU2AI_RL_PULSE_COUNT++;
		VCU2AI_RR_PULSE_COUNT++;

		if(debug_mode) { printf("Simulation Count:[%d]\r\n",counter); }

		counter++;
	}


	// output the data, converting where needed
	data->VCU2AI_AS_STATE = VCU2AI_AS_STATE;
	data->VCU2AI_AMI_STATE = VCU2AI_AMI_STATE;
	data->VCU2AI_HANDSHAKE_RECEIVE_BIT = VCU2AI_HANDSHAKE_RECEIVE_BIT;
	data->VCU2AI_STEER_ANGLE_MAX_deg = 0.1f*VCU2AI_STEER_ANGLE_MAX_deg;
	data->VCU2AI_STEER_ANGLE_deg = 0.1f*VCU2AI_STEER_ANGLE_deg;
	data->VCU2AI_WHEEL_SPEED_MAX_rpm = MOTOR_MAX_RPM / MOTOR_RATIO;
	data->VCU2AI_FL_WHEEL_SPEED_rpm = (float)VCU2AI_FL_WHEEL_SPEED_rpm;
	data->VCU2AI_FR_WHEEL_SPEED_rpm = (float)VCU2AI_FR_WHEEL_SPEED_rpm;
	data->VCU2AI_RL_WHEEL_SPEED_rpm = (float)VCU2AI_RL_WHEEL_SPEED_rpm;
	data->VCU2AI_RR_WHEEL_SPEED_rpm = (float)VCU2AI_RR_WHEEL_SPEED_rpm;
	data->VCU2AI_FL_PULSE_COUNT = VCU2AI_FL_PULSE_COUNT;
	data->VCU2AI_FR_PULSE_COUNT = VCU2AI_FR_PULSE_COUNT;
	data->VCU2AI_RL_PULSE_COUNT = VCU2AI_RL_PULSE_COUNT;
	data->VCU2AI_RR_PULSE_COUNT = VCU2AI_RR_PULSE_COUNT;
	data->VCU2AI_Accel_longitudinal_ms2 = -1.0f*((9.80665f*0.001f*3.91f*(float)VCU2AI_Accel_Z)+ACCEL_Z_OFFSET);
	data->VCU2AI_Accel_lateral_ms2 = -1.0f*((9.80665f*0.001f*3.91f*(float)VCU2AI_Accel_X)+ACCEL_X_OFFSET);
	data->VCU2AI_Accel_vertical_ms2 = (9.80665f*0.001f*3.91f*(float)VCU2AI_Accel_Y)+ACCEL_Y_OFFSET;
	data->VCU2AI_Yaw_rate_degps = -1.0f*((0.00875f*(float)VCU2AI_Gyro_Y)+GYRO_Y_OFFSET);
	data->VCU2AI_Pitch_rate_degps = (0.00875f*(float)VCU2AI_Gyro_X)+GYRO_X_OFFSET;
	data->VCU2AI_Roll_rate_degps = -1.0f*((0.00875f*(float)VCU2AI_Gyro_Z)+GYRO_Z_OFFSET);
}


void fs_ai_api_ai2vcu_set_data(fs_ai_api_ai2vcu *data) {
	// temporary input data buffers
	float							AI_FRONT_MOTOR_SPEED_REQUEST_rpm = 0;
	float							AI_REAR_MOTOR_SPEED_REQUEST_rpm = 0;
	float							AI_STEER_REQUEST_deg = 0;
	fs_ai_api_mission_status_e		AI_MISSION_STATUS = 0;
	fs_ai_api_direction_request_e	AI_DIRECTION_REQUEST = 0;
	fs_ai_api_estop_request_e		AI_ESTOP_REQUEST = 0;
	fs_ai_api_handshake_send_bit_e	AI_HANDSHAKE_SEND_BIT = 0;

	AI_FRONT_MOTOR_SPEED_REQUEST_rpm = data->AI2VCU_WHEEL_SPEED_REQUEST_rpm * MOTOR_RATIO;
	AI_REAR_MOTOR_SPEED_REQUEST_rpm = data->AI2VCU_WHEEL_SPEED_REQUEST_rpm * MOTOR_RATIO;
	AI_STEER_REQUEST_deg = data->AI2VCU_STEER_ANGLE_REQUEST_deg;
	AI_MISSION_STATUS = data->AI2VCU_MISSION_STATUS;
	AI_DIRECTION_REQUEST = data->AI2VCU_DIRECTION_REQUEST;
	AI_ESTOP_REQUEST = data->AI2VCU_ESTOP_REQUEST;
	AI_HANDSHAKE_SEND_BIT = data->AI2VCU_HANDSHAKE_SEND_BIT;

	// validate the 'float' requests
	if(AI_FRONT_MOTOR_SPEED_REQUEST_rpm > MOTOR_MAX_RPM) { AI_FRONT_MOTOR_SPEED_REQUEST_rpm = MOTOR_MAX_RPM; }
	if(AI_FRONT_MOTOR_SPEED_REQUEST_rpm < 0.0f) { AI_FRONT_MOTOR_SPEED_REQUEST_rpm = 0.0f; }

	if(AI_REAR_MOTOR_SPEED_REQUEST_rpm > MOTOR_MAX_RPM) { AI_REAR_MOTOR_SPEED_REQUEST_rpm = MOTOR_MAX_RPM; }
	if(AI_REAR_MOTOR_SPEED_REQUEST_rpm < 0.0f) { AI_REAR_MOTOR_SPEED_REQUEST_rpm = 0.0f; }

	if(AI_STEER_REQUEST_deg > (0.1f*(float)VCU2AI_STEER_ANGLE_MAX_deg)) { AI_STEER_REQUEST_deg = (0.1f*(float)VCU2AI_STEER_ANGLE_MAX_deg); }
	if(AI_STEER_REQUEST_deg < (-0.1f*(float)VCU2AI_STEER_ANGLE_MAX_deg)) { AI_STEER_REQUEST_deg = (-0.1f*(float)VCU2AI_STEER_ANGLE_MAX_deg); }

	// validate the 'enum' requests
	if(AI_MISSION_STATUS < MISSION_NOT_SELECTED) { AI_MISSION_STATUS = MISSION_NOT_SELECTED; }
	if(AI_MISSION_STATUS > MISSION_FINISHED) { AI_MISSION_STATUS = MISSION_FINISHED; }

	if(AI_DIRECTION_REQUEST < DIRECTION_NEUTRAL) { AI_DIRECTION_REQUEST = DIRECTION_NEUTRAL; }
	if(AI_DIRECTION_REQUEST > DIRECTION_FORWARD) { AI_DIRECTION_REQUEST = DIRECTION_FORWARD; }

	if(AI_ESTOP_REQUEST < ESTOP_NO) { AI_ESTOP_REQUEST = ESTOP_NO; }
	if(AI_ESTOP_REQUEST > ESTOP_YES) { AI_ESTOP_REQUEST = ESTOP_YES; }

	if(AI_HANDSHAKE_SEND_BIT < HANDSHAKE_SEND_BIT_OFF) { AI_HANDSHAKE_SEND_BIT = HANDSHAKE_SEND_BIT_OFF; }
	if(AI_HANDSHAKE_SEND_BIT > HANDSHAKE_SEND_BIT_ON) { AI_HANDSHAKE_SEND_BIT = HANDSHAKE_SEND_BIT_ON; }


	// set the torque values appropriate to the AS_STATE
	switch(VCU2AI_AS_STATE) {
		case 1:			// AS_OFF
		case 2:			// AS_READY
		case 4:			// EMERGENCY_BRAKE
		case 5:			// AS_FINISHED
		default: {		// UNDEFINED
			AI2VCU_FRONT_AXLE_TRQ_REQUEST_Nm = 0;
			AI2VCU_REAR_AXLE_TRQ_REQUEST_Nm = 0;
			break;
		}
		case 3: {		// AS_DRIVING
			// TODO: establish if decel requires this to be set -ve
			if(VCU2AI_FL_WHEEL_SPEED_rpm > 700) {
				AI2VCU_FRONT_AXLE_TRQ_REQUEST_Nm = 500;
				AI2VCU_REAR_AXLE_TRQ_REQUEST_Nm = 500;
			} else if(VCU2AI_FL_WHEEL_SPEED_rpm > 600) {
				AI2VCU_FRONT_AXLE_TRQ_REQUEST_Nm = 850;
				AI2VCU_REAR_AXLE_TRQ_REQUEST_Nm = 850;				
			} else if(VCU2AI_FL_WHEEL_SPEED_rpm > 500) {
				AI2VCU_FRONT_AXLE_TRQ_REQUEST_Nm = 1000;
				AI2VCU_REAR_AXLE_TRQ_REQUEST_Nm = 1000;				
			} else if(VCU2AI_FL_WHEEL_SPEED_rpm > 400) {
				AI2VCU_FRONT_AXLE_TRQ_REQUEST_Nm = 1200;
				AI2VCU_REAR_AXLE_TRQ_REQUEST_Nm = 1200;				
			} else if(VCU2AI_FL_WHEEL_SPEED_rpm > 300) {
				AI2VCU_FRONT_AXLE_TRQ_REQUEST_Nm = 1500;
				AI2VCU_REAR_AXLE_TRQ_REQUEST_Nm = 1500;				
			} else {
				AI2VCU_FRONT_AXLE_TRQ_REQUEST_Nm = VCU2AI_FRONT_AXLE_TORQUE_MAX_Nm;
				AI2VCU_REAR_AXLE_TRQ_REQUEST_Nm = VCU2AI_REAR_AXLE_TORQUE_MAX_Nm;
			}
			break;
		}
	}

	// set requests, converting where needed
	AI2VCU_HANDSHAKE_BIT = AI_HANDSHAKE_SEND_BIT;
	AI2VCU_ESTOP_REQUEST = AI_ESTOP_REQUEST;
	AI2VCU_MISSION_STATUS = AI_MISSION_STATUS;
	AI2VCU_DIRECTION_REQUEST = AI_DIRECTION_REQUEST;
	AI2VCU_LAP_COUNTER = 0;
	AI2VCU_CONES_COUNT_ACTUAL = 0;
	AI2VCU_CONES_COUNT_ALL = 0;

	AI2VCU_FRONT_MOTOR_SPEED_MAX_rpm = (uint16_t)AI_FRONT_MOTOR_SPEED_REQUEST_rpm;
	AI2VCU_REAR_MOTOR_SPEED_MAX_rpm = (uint16_t)AI_REAR_MOTOR_SPEED_REQUEST_rpm;
	AI2VCU_STEER_REQUEST_deg = (int16_t)(10.0f*AI_STEER_REQUEST_deg);

	AI2VCU_HYD_PRESSURE_REQUEST_pct = 0;

	// load the CAN frames with the validated data
	volatile pack_int16_t temp;
	
	temp.word = (int16_t)((1.0f/0.00195313f)*((9.80665f*0.001f*3.91f*(float)VCU2AI_Accel_X)+ACCEL_X_OFFSET));
	AI2LOG_Dynamics2.data[0] = temp.bytes[0];
	AI2LOG_Dynamics2.data[1] = temp.bytes[1];
	temp.word = (int16_t)((1.0f/0.00195313f)*((9.80665f*0.001f*3.91f*(float)VCU2AI_Accel_Y)+ACCEL_Y_OFFSET));
	AI2LOG_Dynamics2.data[2] = temp.bytes[0];
	AI2LOG_Dynamics2.data[3] = temp.bytes[1];
	temp.word = (int16_t)((1.0f/0.0078125f)*((0.00875f*(float)VCU2AI_Gyro_Z)+GYRO_X_OFFSET));
	AI2LOG_Dynamics2.data[4] = temp.bytes[0];
	AI2LOG_Dynamics2.data[5] = temp.bytes[1];
	
	AI2VCU_Status.data[0] = (uint8_t)(AI2VCU_HANDSHAKE_BIT & 0x01);
	AI2VCU_Status.data[1] = (uint8_t)(((AI2VCU_DIRECTION_REQUEST & 0x03) << 6) + ((AI2VCU_MISSION_STATUS & 0x03) << 4) + (AI2VCU_ESTOP_REQUEST & 0x01));
	AI2VCU_Status.data[2] = (AI2VCU_LAP_COUNTER & 0x0F);
	AI2VCU_Status.data[3] = AI2VCU_CONES_COUNT_ACTUAL;
	AI2VCU_Status.data[4] = (uint8_t)(AI2VCU_CONES_COUNT_ALL & 0x00FF);
	AI2VCU_Status.data[5] = (uint8_t)((AI2VCU_CONES_COUNT_ALL & 0xFF00) >> 8);
	AI2VCU_Status.data[6] = 0;
	AI2VCU_Status.data[7] = 0;

	temp.word = AI2VCU_FRONT_AXLE_TRQ_REQUEST_Nm;
	AI2VCU_Drive_F.data[0] = temp.bytes[0];
	AI2VCU_Drive_F.data[1] = temp.bytes[1];
	temp.word = AI2VCU_FRONT_MOTOR_SPEED_MAX_rpm;	
	AI2VCU_Drive_F.data[2] = temp.bytes[0];
	AI2VCU_Drive_F.data[3] = temp.bytes[1];
	AI2VCU_Drive_F.data[4] = 0;
	AI2VCU_Drive_F.data[5] = 0;
	AI2VCU_Drive_F.data[6] = 0;
	AI2VCU_Drive_F.data[7] = 0;

	temp.word = AI2VCU_REAR_AXLE_TRQ_REQUEST_Nm;
	AI2VCU_Drive_R.data[0] = temp.bytes[0];
	AI2VCU_Drive_R.data[1] = temp.bytes[1];
	temp.word = AI2VCU_REAR_MOTOR_SPEED_MAX_rpm;
	AI2VCU_Drive_R.data[2] = temp.bytes[0];
	AI2VCU_Drive_R.data[3] = temp.bytes[1];
	AI2VCU_Drive_R.data[4] = 0;
	AI2VCU_Drive_R.data[5] = 0;
	AI2VCU_Drive_R.data[6] = 0;
	AI2VCU_Drive_R.data[7] = 0;

	temp.word = AI2VCU_STEER_REQUEST_deg;
	AI2VCU_Steer.data[0] = temp.bytes[0];
	AI2VCU_Steer.data[1] = temp.bytes[1];
	AI2VCU_Steer.data[2] = 0;
	AI2VCU_Steer.data[3] = 0;
	AI2VCU_Steer.data[4] = 0;
	AI2VCU_Steer.data[5] = 0;
	AI2VCU_Steer.data[6] = 0;
	AI2VCU_Steer.data[7] = 0;

	temp.word = AI2VCU_HYD_PRESSURE_REQUEST_pct;
	AI2VCU_Brake.data[0] = (uint8_t)temp.word;
	AI2VCU_Brake.data[1] = 0;
	AI2VCU_Brake.data[2] = 0;
	AI2VCU_Brake.data[3] = 0;
	AI2VCU_Brake.data[4] = 0;
	AI2VCU_Brake.data[5] = 0;
	AI2VCU_Brake.data[6] = 0;
	AI2VCU_Brake.data[7] = 0;

	clock_gettime(CLOCK_REALTIME,&this_set);

	long int interval_ns = ((this_set.tv_sec-last_set.tv_sec)* 1000000000) + (this_set.tv_nsec-last_set.tv_nsec);
	
	if(interval_ns > 4000000) // enforce maximum call rate of approx. 4ms
	{
		// send the CAN frames
		can_send(&AI2LOG_Dynamics2);
		can_send(&AI2VCU_Status);
		can_send(&AI2VCU_Drive_F);
		can_send(&AI2VCU_Drive_R);
		can_send(&AI2VCU_Steer);
		can_send(&AI2VCU_Brake);
		clock_gettime(CLOCK_REALTIME,&last_set);
	}
}


void fs_ai_api_debug_get_data(uint8_t *data) {
	pthread_mutex_lock(&can_read_mutex); // protect the buffers from race conditions

	if(DEBUG_1_fresh) {
		DEBUG_1_fresh = FALSE;
	}

	// always copy
	data[0] = DEBUG_1.data[0];
	data[1] = DEBUG_1.data[1];
	data[2] = DEBUG_1.data[2];
	data[3] = DEBUG_1.data[3];
	data[4] = DEBUG_1.data[4];
	data[5] = DEBUG_1.data[5];
	data[6] = DEBUG_1.data[6];
	data[7] = DEBUG_1.data[7];

	pthread_mutex_unlock(&can_read_mutex); // don't forget!
}