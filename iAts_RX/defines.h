#ifndef DEFINES_H
#define DEFINES_H

#include <inttypes.h>
#include "config.h"

#define OLED_I2C_ADDRESS    0x3C

#define T_GPS_RX_PIN        8
#define T_GPS_TX_PIN        7
#define VOLTAGEDIVIDER      A0
#define LED_DIGITAL_PIN     13
#define LED_SERIAL_PIN      2
#define LED_ENCODE_PIN      4
#define EN_33V_PIN          6

#define PARAM_EPPROM_POS  	6
#define PARAM_EPPROM_POS_PID_P  	              PARAM_EPPROM_POS + 0
#define PARAM_EPPROM_POS_PID_I  	              PARAM_EPPROM_POS + 2
#define PARAM_EPPROM_POS_PID_D  	              PARAM_EPPROM_POS + 4
#define PARAM_EPPROM_POS_PID_DIVIDER  	        PARAM_EPPROM_POS + 5
#define PARAM_EPPROM_POS_PID_MAX_ERROR 	        PARAM_EPPROM_POS + 6
#define PARAM_EPPROM_POS_TILT_0  	              PARAM_EPPROM_POS + 8
#define PARAM_EPPROM_POS_TILT_90   	            PARAM_EPPROM_POS + 10
#define PARAM_EPPROM_POS_PAN_CENTER  	          PARAM_EPPROM_POS + 12
#define PARAM_EPPROM_POS_PAN_MIN_SPEED 	        PARAM_EPPROM_POS + 14
#define PARAM_EPPROM_POS_COMPASS_OFFSET	        PARAM_EPPROM_POS + 15
#define PARAM_EPPROM_POS_COMPASS_DECLINATION  	PARAM_EPPROM_POS + 17
#define PARAM_EPPROM_POS_START_TRACKING_DISTANCE  	PARAM_EPPROM_POS + 18

#define TRACKER_MODE_MANUAL 0
#define TRACKER_MODE_AUTO   1
#define TRACKER_MODE_DEBUG  2

//lat and lon required in units of millionths of a degree -> precision of 5 digits after '.'
// for example 52.52081 -> 5252081
//             13.40945 -> 1340945
struct Airplane
{
  // latitude in units of millionths of a degree
  uint32_t lat;
  // longitude in units of millionths of a degree
  uint32_t lon;
  // altitude ranging from -32.768m to 32.767m
  uint32_t alt;
  // pitch in 0-359° *10
  uint16_t pitch;
  // roll in 0-359° *10
  uint16_t roll;
  // heading in 0-359° *10
  uint16_t heading;
  // distance from 0 ... 64km
  uint16_t distance;
  // speed 0-255
  uint16_t speed;
  // sats
  uint8_t sats;
  // fix_type
  uint8_t fix_type;
};

struct Tracker
{
  uint32_t lat;         //纬度
  uint32_t lon;         //经度
  uint32_t alt;         //高度
  uint16_t heading;     //朝向
  uint8_t pitching;     //俯仰
  uint16_t voltage;     //电压
  uint8_t mode;         //跟踪模式
  uint16_t course;		//飞机指向
  uint16_t outpwm;		//输出脉冲
  int8_t declination;	//本地磁偏角
};

struct Parameter
{
	uint16_t pid_p;
	uint16_t pid_i;
	uint16_t pid_d;
  uint8_t pid_divider;
  uint8_t pid_max_error;
	uint16_t tilt_0;
	uint16_t tilt_90;
	uint16_t pan_center;
  uint8_t pan_min_speed;
	int16_t compass_offset;
  int8_t compass_declination;
	uint8_t start_tracking_distance;
};

//typedef struct
//{
//	uint16_t pid_p;
//	uint16_t pid_i;
//	uint16_t pid_d;
//	uint16_t titl_0;
//	uint16_t titl_90;
//	uint16_t pan_0;
//	uint8_t compass_offset;
//	uint8_t start_tracking_distance;
//	uint8_t max_pid_error;
//	uint8_t max_pan_speed;
//	uint8_t min_pan_speed;
//} TrackerTParameter;
//
//TrackerTParameter *TParam;

//#define PID_P						((volatile TrackerTParameter *)(&TParam))->pid_p
//#define PID_I						((volatile TrackerTParameter *)(&TParam))->pid_i
//#define PID_D						((volatile TrackerTParameter *)(&TParam))->pid_d
//#define TITL_0						((volatile TrackerTParameter *)(&TParam))->titl_0
//#define TITL_90						((volatile TrackerTParameter *)(&TParam))->titl_90
//#define PAN_0						((volatile TrackerTParameter *)(&TParam))->pan_0
//#define COMPASS_OFFSET				((volatile TrackerTParameter *)(&TParam))->compass_offset
//#define START_TRACKING_DISTANCE		((volatile TrackerTParameter *)(&TParam))->start_tracking_distance
//#define MAX_PID_ERROR				((volatile TrackerTParameter *)(&TParam))->max_pid_error
//#define MAX_PAN_SPEED				((volatile TrackerTParameter *)(&TParam))->max_pan_speed
//#define MIN_PAN_SPEED				((volatile TrackerTParameter *)(&TParam))->min_pan_speed

typedef struct
{
  bool f0 : 1;
  bool f1 : 1;
  bool f2 : 1;
  bool f3 : 1;
  bool f4 : 1;
  bool f5 : 1;
  bool f6 : 1;
  bool f7 : 1;
} PackedBool;

#define HOME_SETED		   ((volatile PackedBool *)(&GPIOR0))->f0
#define TRACKING_STARTED ((volatile PackedBool *)(&GPIOR0))->f1
#define CURRENT_STATE    ((volatile PackedBool *)(&GPIOR0))->f2
#define PREVIOUS_STATE   ((volatile PackedBool *)(&GPIOR0))->f3
#define HAS_FIX          ((volatile PackedBool *)(&GPIOR0))->f4
#define HAS_ALT          ((volatile PackedBool *)(&GPIOR0))->f5
#define BT_CONNECTED		 ((volatile PackedBool *)(&GPIOR0))->f6
#define NEW_HEADING      ((volatile PackedBool *)(&GPIOR0))->f7

typedef struct
{
	bool f0 : 1;
	bool f1 : 1;
	bool f2 : 1;
	bool f3 : 1;
	bool f4 : 1;
	bool f5 : 1;
	bool f6 : 1;
	bool f7 : 1;
} Status;


#define BT_DATA_SENDING			    ((volatile Status *)(&GPIOR1))->f0
#define AUTO_POINT_TO_NORTH			((volatile Status *)(&GPIOR1))->f2
#define SERVO_TILT_HAS_ARRIVED  ((volatile Status *)(&GPIOR1))->f3

#define toRad(val) val * PI/180.0f
#define toDeg(val) val * 180.0f/PI

#define meter2feet(value) value * 3.2808399
#define feet2meter(value) value * 0.3048

#endif