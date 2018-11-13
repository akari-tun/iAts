#ifndef TPROTOCOL_H
#define TPROTOCOL_H

#include "defines.h"

#define MAX_TAG_COUNT							10        //每帧数据最大TAG数
#define MAX_CMD_COUNT							5         //最大缓存等待发送的指令数

#define TP_PACKET_LEAD							0x24      //引导码 $
#define TP_PACKET_START							0x54      //协议头 T

#define TAG_COUNT								37        //TAG数量，定义了新的TAG需要增加这个值
//-----------------基础协议---------------------------------------------------
#define TAG_BASE_ACK							0x00      //应答结果 L:1 V:0 失败 非0成功，成功应答命令的INDEX
#define TAG_BASE_HEARTBEAT						0x01	  //设备心跳 L:1 V:0空闲 1正在跟踪 2调试模式 3手动模式
#define TAG_BASE_QUERY							0x02	  //请求数据 L:1 V:CMD 请求的指令的CMD值
//-----------------飞机数据---------------------------------------------------
#define TAG_PLANE_LONGITUDE						0x10      //飞机经度 L:4 
#define TAG_PLANE_LATITUDE						0x11      //飞机纬度 L:4
#define TAG_PLANE_ALTITUDE						0x12      //飞机高度 L:4
#define TAG_PLANE_SPEED							0x13      //飞机速度 L:2
#define TAG_PLANE_DISTANCE						0x14      //飞机距离 L:4
#define TAG_PLANE_STAR							0x15      //定位星数 L:1
#define TAG_PLANE_FIX							0x16      //定位类型 L:1
#define TAG_PLANE_PITCH							0x17      //俯仰角度 L:2
#define TAG_PLANE_ROLL							0x18      //横滚角度 L:2
#define TAG_PLANE_HEADING						0x19      //飞机方向 L:2
//-----------------家的数据---------------------------------------------------
#define TAG_HOME_LONGITUDE						0x20      //家的经度 L:4
#define TAG_HOME_LATITUDE						0x21      //家的纬度 L:4
#define TAG_HOME_ALTITUDE						0x22      //家的高度 L:4
#define TAG_HOME_HEADING						0x23      //家的朝向 L:2
#define TAG_HOME_PITCHING						0x24      //家的俯仰 L:1
#define TAG_HOME_VOLTAGE						0x25      //家的电压 L:2
#define TAG_HOME_MODE							0x26      //家的模式 L:1
#define TAG_HOME_DECLINATION					0x27      //磁偏角 L:1
//-----------------配置参数---------------------------------------------------
#define TAG_PARAM_PID_P							0x50      //PID_P L:2
#define TAG_PARAM_PID_I							0x51      //PID_I L:2
#define TAG_PARAM_PID_D							0x52      //PID_D L:2
#define TAG_PARAM_TITL_0						0x53      //俯仰零度 L:2
#define TAG_PARAM_TITL_90						0x54      //俯仰90度 L:2
#define TAG_PARAM_PAN_0  						0x55      //水平中立点 L:2
#define TAG_PARAM_OFFSET						0x56      //罗盘偏移 L:2
#define TAG_PARAM_START_TRACKING_DISTANCE		0x57      //开始跟踪距离 L:1
#define TAG_PARAM_MAX_PID_ERROR					0x58      //跟踪偏移度数 L:1
#define TAG_PARAM_MIN_PAN_SPEED					0x59      //最小舵机速度 L:1
#define TAG_PARAM_DECLINATION					0x5A      //磁偏角 L:1
//-----------------控制参数---------------------------------------------------
#define TAG_CTR_MODE							0x60      //模式 L:1 0：手动模式 1：自动跟踪 2：调试模式
#define TAG_CTR_AUTO_POINT_TO_NORTH				0x61      //自动指北 L:1 0：不启用 1:启用
#define TAG_CTR_CALIBRATE				        0x62      //校准 L:1 >0：开始校准
#define TAG_CTR_HEADING				            0x63      //指向 L:2 0~359
#define TAG_CTR_TILT				            0x64      //俯仰 L:1 0~90

//-----------------命令字---------------------------------------------------
#define CMD_U_HEARTBEAT							0x00      //心跳
#define CMD_U_AIRPLANE							0x30      //上传飞机状态
#define CMD_U_TRACKER							0x31      //上传设备状态
#define CMD_U_PARAM								0x32      //上传参数
#define CMD_U_ACK                               0x33      //应答结果

#define CMD_D_HEARTBEAT							0x00      //心跳
#define CMD_D_SET_HOME							0x40      //设置家
#define CMD_D_SET_PARAM							0x41      //下载参数
#define CMD_D_QUERY_PARAM						0x42      //请求参数
#define CMD_D_CONTROL 		    				0x43      //控制指令

void tprotocol(Airplane *target, Tracker *tracker, Parameter *param);
boolean encodeTrackerData(uint8_t c);
void analysis();
void setSendCmd(uint8_t cmd);
void setSendCmd(uint8_t cmd);
void setSendCmdFirst(uint8_t cmd);
void getSendData(uint8_t* &p_data, uint8_t &p_len);
uint8_t getCmdCount();
void setReceiveCmd(uint8_t cmd);
uint8_t getReceiveCmd();
unsigned long getLastHeartBeetAck();
uint8_t tagread_u8();
uint16_t tagread_u16();
uint32_t tagread_u32();

#endif