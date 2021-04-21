/*********************************************************************
 * Copyright: Preston EV Limited 2018, Rockfort Engineering Ltd. 2019
 * 
 * File:	fs-ai_api.h
 * Author:	Ian Murphy
 * Date:	2018-06-25, 2019-05-14
 * 
 ********************************************************************/


#ifndef FS_AI_API_H
#define FS_AI_API_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>


typedef enum fs_ai_api_as_state_e {
	AS_OFF = 1,
	AS_READY = 2,
	AS_DRIVING = 3,
	AS_EMERGENCY_BRAKE = 4,
	AS_FINISHED = 5,
} fs_ai_api_as_state_e;


typedef enum fs_ai_api_ami_state_e {
	AMI_NOT_SELECTED = 0,
	AMI_ACCELERATION = 1,
	AMI_SKIDPAD = 2,
	AMI_AUTOCROSS = 3,
	AMI_TRACK_DRIVE = 4,
	AMI_BRAKE_TEST = 5,
	AMI_INSPECTION = 6,
	AMI_MANUAL = 7,
} fs_ai_api_ami_state_e;


typedef enum fs_ai_api_handshake_receive_bit_e {
	HANDSHAKE_RECEIVE_BIT_OFF = 0,
	HANDSHAKE_RECEIVE_BIT_ON = 1,
} fs_ai_api_handshake_receive_bit_e;


#ifdef __cplusplus
typedef volatile struct alignas(4) fs_ai_api_vcu2ai_struct {
	volatile fs_ai_api_as_state_e				VCU2AI_AS_STATE;
	volatile fs_ai_api_ami_state_e				VCU2AI_AMI_STATE;
	volatile fs_ai_api_handshake_receive_bit_e	VCU2AI_HANDSHAKE_RECEIVE_BIT;
	volatile float								VCU2AI_STEER_ANGLE_MAX_deg;
	volatile float								VCU2AI_STEER_ANGLE_deg;
	volatile float								VCU2AI_WHEEL_SPEED_MAX_rpm;
	volatile float								VCU2AI_FL_WHEEL_SPEED_rpm;
	volatile float								VCU2AI_FR_WHEEL_SPEED_rpm;
	volatile float								VCU2AI_RL_WHEEL_SPEED_rpm;
	volatile float								VCU2AI_RR_WHEEL_SPEED_rpm;
	volatile uint16_t							VCU2AI_FL_PULSE_COUNT;
	volatile uint16_t							VCU2AI_FR_PULSE_COUNT;
	volatile uint16_t							VCU2AI_RL_PULSE_COUNT;
	volatile uint16_t							VCU2AI_RR_PULSE_COUNT;
	volatile float								VCU2AI_Accel_longitudinal_ms2;
	volatile float								VCU2AI_Accel_lateral_ms2;
	volatile float								VCU2AI_Accel_vertical_ms2;
	volatile float								VCU2AI_Yaw_rate_degps;
	volatile float								VCU2AI_Roll_rate_degps;
	volatile float								VCU2AI_Pitch_rate_degps;
} fs_ai_api_vcu2ai;
#else
typedef volatile struct fs_ai_api_vcu2ai_struct {
	volatile _Alignas(4) fs_ai_api_as_state_e				VCU2AI_AS_STATE;
	volatile _Alignas(4) fs_ai_api_ami_state_e				VCU2AI_AMI_STATE;
	volatile _Alignas(4) fs_ai_api_handshake_receive_bit_e	VCU2AI_HANDSHAKE_RECEIVE_BIT;
	volatile _Alignas(4) float								VCU2AI_STEER_ANGLE_MAX_deg;
	volatile _Alignas(4) float								VCU2AI_STEER_ANGLE_deg;
	volatile _Alignas(4) float								VCU2AI_WHEEL_SPEED_MAX_rpm;
	volatile _Alignas(4) float								VCU2AI_FL_WHEEL_SPEED_rpm;
	volatile _Alignas(4) float								VCU2AI_FR_WHEEL_SPEED_rpm;
	volatile _Alignas(4) float								VCU2AI_RL_WHEEL_SPEED_rpm;
	volatile _Alignas(4) float								VCU2AI_RR_WHEEL_SPEED_rpm;
	volatile _Alignas(4) uint16_t							VCU2AI_FL_PULSE_COUNT;
	volatile _Alignas(4) uint16_t							VCU2AI_FR_PULSE_COUNT;
	volatile _Alignas(4) uint16_t							VCU2AI_RL_PULSE_COUNT;
	volatile _Alignas(4) uint16_t							VCU2AI_RR_PULSE_COUNT;
	volatile _Alignas(4) float								VCU2AI_Accel_longitudinal_ms2;
	volatile _Alignas(4) float								VCU2AI_Accel_lateral_ms2;
	volatile _Alignas(4) float								VCU2AI_Accel_vertical_ms2;
	volatile _Alignas(4) float								VCU2AI_Yaw_rate_degps;
	volatile _Alignas(4) float								VCU2AI_Roll_rate_degps;
	volatile _Alignas(4) float								VCU2AI_Pitch_rate_degps;
} fs_ai_api_vcu2ai;
#endif


typedef enum fs_ai_api_mission_status_e {
	MISSION_NOT_SELECTED = 0,
	MISSION_SELECTED = 1,
	MISSION_RUNNING = 2,
	MISSION_FINISHED = 3,
} fs_ai_api_mission_status_e;


typedef enum fs_ai_api_direction_request_e {
	DIRECTION_NEUTRAL = 0,
	DIRECTION_FORWARD = 1,
} fs_ai_api_direction_request_e;


typedef enum fs_ai_api_estop_request_e {
	ESTOP_NO = 0,
	ESTOP_YES = 1,
} fs_ai_api_estop_request_e;


typedef enum fs_ai_api_handshake_send_bit_e {
	HANDSHAKE_SEND_BIT_OFF = 0,
	HANDSHAKE_SEND_BIT_ON = 1,
} fs_ai_api_handshake_send_bit_e;


#ifdef __cplusplus
typedef volatile struct alignas(4) fs_ai_api_ai2vcu_struct {
	volatile fs_ai_api_mission_status_e		AI2VCU_MISSION_STATUS;
	volatile fs_ai_api_direction_request_e	AI2VCU_DIRECTION_REQUEST;
	volatile fs_ai_api_estop_request_e		AI2VCU_ESTOP_REQUEST;
	volatile fs_ai_api_handshake_send_bit_e	AI2VCU_HANDSHAKE_SEND_BIT;
	volatile float							AI2VCU_STEER_ANGLE_REQUEST_deg;
	volatile float							AI2VCU_WHEEL_SPEED_REQUEST_rpm;
} fs_ai_api_ai2vcu;
#else
typedef volatile struct fs_ai_api_ai2vcu_struct {
	volatile _Alignas(4) fs_ai_api_mission_status_e		AI2VCU_MISSION_STATUS;
	volatile _Alignas(4) fs_ai_api_direction_request_e	AI2VCU_DIRECTION_REQUEST;
	volatile _Alignas(4) fs_ai_api_estop_request_e		AI2VCU_ESTOP_REQUEST;
	volatile _Alignas(4) fs_ai_api_handshake_send_bit_e	AI2VCU_HANDSHAKE_SEND_BIT;
	volatile _Alignas(4) float							AI2VCU_STEER_ANGLE_REQUEST_deg;
	volatile _Alignas(4) float							AI2VCU_WHEEL_SPEED_REQUEST_rpm;
} fs_ai_api_ai2vcu;
#endif


int fs_ai_api_init(char *CAN_interface, int debug, int simulate);

void fs_ai_api_vcu2ai_get_data(fs_ai_api_vcu2ai *data);
void fs_ai_api_ai2vcu_set_data(fs_ai_api_ai2vcu *data);


#ifdef __cplusplus
}
#endif

#endif /* FS_AI_API_H */

