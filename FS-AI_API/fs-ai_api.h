/**************************************************************************
 * Copyright: Preston EV Limited 2018, Rockfort Engineering Ltd. 2019, 2021
 * 
 * File:	fs-ai_api.h
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
	AMI_STATIC_INSPECTION_A = 5,
	AMI_STATIC_INSPECTION_B = 6,
	AMI_AUTONOMOUS_DEMO = 7,
} fs_ai_api_ami_state_e;


typedef enum fs_ai_api_handshake_receive_bit_e {
	HANDSHAKE_RECEIVE_BIT_OFF = 0,
	HANDSHAKE_RECEIVE_BIT_ON = 1,
} fs_ai_api_handshake_receive_bit_e;


typedef enum fs_ai_api_res_go_signal_bit_e {
	RES_GO_SIGNAL_NO_GO = 0,
	RES_GO_SIGNAL_GO = 1,
} fs_ai_api_res_go_signal_bit_e;


#ifdef __cplusplus
typedef volatile struct alignas(4) fs_ai_api_vcu2ai_struct {
	volatile fs_ai_api_handshake_receive_bit_e	VCU2AI_HANDSHAKE_RECEIVE_BIT;
	volatile fs_ai_api_res_go_signal_bit_e		VCU2AI_RES_GO_SIGNAL;
	volatile fs_ai_api_as_state_e				VCU2AI_AS_STATE;
	volatile fs_ai_api_ami_state_e				VCU2AI_AMI_STATE;
	volatile float								VCU2AI_STEER_ANGLE_deg;
	volatile float								VCU2AI_BRAKE_PRESS_F_pct;
	volatile float								VCU2AI_BRAKE_PRESS_R_pct;
	volatile float								VCU2AI_FL_WHEEL_SPEED_rpm;
	volatile float								VCU2AI_FR_WHEEL_SPEED_rpm;
	volatile float								VCU2AI_RL_WHEEL_SPEED_rpm;
	volatile float								VCU2AI_RR_WHEEL_SPEED_rpm;
	volatile uint16_t							VCU2AI_FL_PULSE_COUNT;
	volatile uint16_t							VCU2AI_FR_PULSE_COUNT;
	volatile uint16_t							VCU2AI_RL_PULSE_COUNT;
	volatile uint16_t							VCU2AI_RR_PULSE_COUNT;
} fs_ai_api_vcu2ai;
#else
typedef volatile struct fs_ai_api_vcu2ai_struct {
	volatile _Alignas(4) fs_ai_api_handshake_receive_bit_e	VCU2AI_HANDSHAKE_RECEIVE_BIT;
	volatile _Alignas(4) fs_ai_api_res_go_signal_bit_e		VCU2AI_RES_GO_SIGNAL;
	volatile _Alignas(4) fs_ai_api_as_state_e				VCU2AI_AS_STATE;
	volatile _Alignas(4) fs_ai_api_ami_state_e				VCU2AI_AMI_STATE;
	volatile _Alignas(4) float								VCU2AI_STEER_ANGLE_deg;
	volatile _Alignas(4) float								VCU2AI_BRAKE_PRESS_F_pct;
	volatile _Alignas(4) float								VCU2AI_BRAKE_PRESS_R_pct;
	volatile _Alignas(4) float								VCU2AI_FL_WHEEL_SPEED_rpm;
	volatile _Alignas(4) float								VCU2AI_FR_WHEEL_SPEED_rpm;
	volatile _Alignas(4) float								VCU2AI_RL_WHEEL_SPEED_rpm;
	volatile _Alignas(4) float								VCU2AI_RR_WHEEL_SPEED_rpm;
	volatile _Alignas(4) uint16_t							VCU2AI_FL_PULSE_COUNT;
	volatile _Alignas(4) uint16_t							VCU2AI_FR_PULSE_COUNT;
	volatile _Alignas(4) uint16_t							VCU2AI_RL_PULSE_COUNT;
	volatile _Alignas(4) uint16_t							VCU2AI_RR_PULSE_COUNT;
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
	volatile float							AI2VCU_AXLE_SPEED_REQUEST_rpm;
	volatile float							AI2VCU_AXLE_TORQUE_REQUEST_Nm;
	volatile float							AI2VCU_BRAKE_PRESS_REQUEST_pct;
} fs_ai_api_ai2vcu;
#else
typedef volatile struct fs_ai_api_ai2vcu_struct {
	volatile _Alignas(4) fs_ai_api_mission_status_e		AI2VCU_MISSION_STATUS;
	volatile _Alignas(4) fs_ai_api_direction_request_e	AI2VCU_DIRECTION_REQUEST;
	volatile _Alignas(4) fs_ai_api_estop_request_e		AI2VCU_ESTOP_REQUEST;
	volatile _Alignas(4) fs_ai_api_handshake_send_bit_e	AI2VCU_HANDSHAKE_SEND_BIT;
	volatile _Alignas(4) float							AI2VCU_STEER_ANGLE_REQUEST_deg;
	volatile _Alignas(4) float							AI2VCU_AXLE_SPEED_REQUEST_rpm;
	volatile _Alignas(4) float							AI2VCU_AXLE_TORQUE_REQUEST_Nm;
	volatile _Alignas(4) float							AI2VCU_BRAKE_PRESS_REQUEST_pct;
} fs_ai_api_ai2vcu;
#endif


#ifdef __cplusplus
typedef volatile struct alignas(4) fs_ai_api_gps_struct {
	volatile uint8_t	GPS_AntennaStatus;
	volatile uint8_t	GPS_NumSatellites;
	volatile uint8_t	GPS_NavigationMethod;
	volatile float		GPS_Course_deg;
	volatile float		GPS_Speed_kmh;
	volatile float		GPS_Longitude_Minutes;
	volatile uint16_t	GPS_Longitude_Degree;
	volatile uint8_t	GPS_Longitude_IndicatorEW;
	volatile float		GPS_Latitude_Minutes;
	volatile uint16_t	GPS_Latitude_Degree;
	volatile uint8_t	GPS_Latitude_IndicatorNS;
	volatile float		GPS_Altitude;
	volatile float		GPS_PDOP;
	volatile float		GPS_HDOP;
	volatile float		GPS_VDOP;
	volatile uint8_t	GPS_UTC_Year;
	volatile uint8_t	GPS_UTC_Month;
	volatile uint8_t	GPS_UTC_DayOfMonth;
	volatile uint8_t	GPS_UTC_Hour;
	volatile uint8_t	GPS_UTC_Minute;
	volatile uint8_t	GPS_UTC_Second;
} fs_ai_api_gps;
#else
typedef volatile struct fs_ai_api_gps_struct {
	volatile _Alignas(4) uint8_t	GPS_AntennaStatus;
	volatile _Alignas(4) uint8_t	GPS_NumSatellites;
	volatile _Alignas(4) uint8_t	GPS_NavigationMethod;
	volatile _Alignas(4) float		GPS_Course_deg;
	volatile _Alignas(4) float		GPS_Speed_kmh;
	volatile _Alignas(4) float		GPS_Longitude_Minutes;
	volatile _Alignas(4) uint16_t	GPS_Longitude_Degree;
	volatile _Alignas(4) uint8_t	GPS_Longitude_IndicatorEW;
	volatile _Alignas(4) float		GPS_Latitude_Minutes;
	volatile _Alignas(4) uint16_t	GPS_Latitude_Degree;
	volatile _Alignas(4) uint8_t	GPS_Latitude_IndicatorNS;
	volatile _Alignas(4) float		GPS_Altitude;
	volatile _Alignas(4) float		GPS_PDOP;
	volatile _Alignas(4) float		GPS_HDOP;
	volatile _Alignas(4) float		GPS_VDOP;
	volatile _Alignas(4) uint8_t	GPS_UTC_Year;
	volatile _Alignas(4) uint8_t	GPS_UTC_Month;
	volatile _Alignas(4) uint8_t	GPS_UTC_DayOfMonth;
	volatile _Alignas(4) uint8_t	GPS_UTC_Hour;
	volatile _Alignas(4) uint8_t	GPS_UTC_Minute;
	volatile _Alignas(4) uint8_t	GPS_UTC_Second;
} fs_ai_api_gps;
#endif


#ifdef __cplusplus
typedef volatile struct alignas(4) fs_ai_api_imu_struct {
	volatile float		IMU_Acceleration_X_mG;
	volatile float		IMU_Acceleration_Y_mG;
	volatile float		IMU_Acceleration_Z_mG;
	volatile float		IMU_Temperature_degC;
	volatile uint8_t	IMU_VerticalAxis;
	volatile uint8_t	IMU_Orientation;
	volatile float		IMU_MagneticField_X_uT;
	volatile float		IMU_MagneticField_Y_uT;
	volatile float		IMU_MagneticField_Z_uT;
	volatile float		IMU_Rotation_X_degps;
	volatile float		IMU_Rotation_Y_degps;
	volatile float		IMU_Rotation_Z_degps;
} fs_ai_api_imu;
#else
typedef volatile struct fs_ai_api_imu_struct {
	volatile _Alignas(4) float		IMU_Acceleration_X_mG;
	volatile _Alignas(4) float		IMU_Acceleration_Y_mG;
	volatile _Alignas(4) float		IMU_Acceleration_Z_mG;
	volatile _Alignas(4) float		IMU_Temperature_degC;
	volatile _Alignas(4) uint8_t	IMU_VerticalAxis;
	volatile _Alignas(4) uint8_t	IMU_Orientation;
	volatile _Alignas(4) float		IMU_MagneticField_X_uT;
	volatile _Alignas(4) float		IMU_MagneticField_Y_uT;
	volatile _Alignas(4) float		IMU_MagneticField_Z_uT;
	volatile _Alignas(4) float		IMU_Rotation_X_degps;
	volatile _Alignas(4) float		IMU_Rotation_Y_degps;
	volatile _Alignas(4) float		IMU_Rotation_Z_degps;
} fs_ai_api_imu;
#endif

typedef struct can_stats_struct {
	volatile uint32_t VCU2AI_Status_count;
	volatile uint32_t VCU2AI_Drive_F_count;
	volatile uint32_t VCU2AI_Drive_R_count;
	volatile uint32_t VCU2AI_Steer_count;
	volatile uint32_t VCU2AI_Brake_count;
	volatile uint32_t VCU2AI_Wheel_speeds_count;
	volatile uint32_t VCU2AI_Wheel_counts_count;
	volatile uint32_t PCAN_GPS_BMC_Acceleration_count;
	volatile uint32_t PCAN_GPS_BMC_MagneticField_count;
	volatile uint32_t PCAN_GPS_L3GD20_Rotation_A_count;
	volatile uint32_t PCAN_GPS_L3GD20_Rotation_B_count;
	volatile uint32_t PCAN_GPS_GPS_Status_count;
	volatile uint32_t PCAN_GPS_GPS_CourseSpeed_count;
	volatile uint32_t PCAN_GPS_GPS_Longitude_count;
	volatile uint32_t PCAN_GPS_GPS_Latitude_count;
	volatile uint32_t PCAN_GPS_GPS_Altitude_count;
	volatile uint32_t PCAN_GPS_GPS_Delusions_A_count;
	volatile uint32_t PCAN_GPS_GPS_Delusions_B_count;
	volatile uint32_t PCAN_GPS_GPS_DateTime_count;
	volatile uint32_t unhandled_frame_count;
} can_stats_t;

int fs_ai_api_init(char *CAN_interface, int debug, int simulate);

void fs_ai_api_vcu2ai_get_data(fs_ai_api_vcu2ai *data);
void fs_ai_api_ai2vcu_set_data(fs_ai_api_ai2vcu *data);

void fs_ai_api_imu_get_data(fs_ai_api_imu *data);
void fs_ai_api_gps_get_data(fs_ai_api_gps *data);

void fs_ai_api_get_can_stats(can_stats_t *data);
void fs_ai_api_clear_can_stats();

#ifdef __cplusplus
}
#endif

#endif /* FS_AI_API_H */

