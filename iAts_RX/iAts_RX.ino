/*
作者：TanJun
用途：蓝牙版AAT

部分代码来源于开源项目 amv-open360tracker_RX_v1_user 项目地址 https://github.com/raul-ortega/amv-open360tracker/
在此基础上做了一些修改
1.重新构建主流程，使执行过程更合理
2.蓝牙调参，蓝牙上传数
3.蓝牙传输GPS数据，在手机上显示飞机位置
4.加入了PID变积分算法，使之能更快的修正误差
*/

#include <Arduino.h>
#include <Wire.h>			  //I2C
#include <SSD1306Ascii.h>	  //OLED屏
#include <SSD1306AsciiWire.h> //OLED屏
#include <SoftwareSerial.h>   //软串口
#include "defines.h"		  //基础定义
#include "config.h"			  //配置定义
#include "telemetry.h"		  //通讯解析
#include "tprotocol.h"		  //协议
#include "battery.h"		  //电压测量
#include "IMU.h"		  	  //惯性测量单元
#include "servos.h"			  //舵机控制
#include "calc.h"			  //计算数据

SoftwareSerial digitalSerial(T_GPS_RX_PIN, T_GPS_TX_PIN);
SSD1306AsciiWire oled;
IMU imu;

char oled_str[24];

struct Airplane airplane;
struct Tracker tracker;
struct Parameter TParam;

long Error[11];
long Accumulator;
long PID;
// double AccGain;
int16_t _servo_tilt_must_move = -1;

int softserial_delay = (int)round(10000000.0f / (BLUETOOTH_SERIAL_BAUDRATE)); // time to wait between each byte sent.

// uint16_t last_course;
// unsigned long airplane_pos_time;
unsigned long airplane_upload_time;
unsigned long tracker_upload_time;
unsigned long heart_upload_time;
unsigned long compass_time;
unsigned long oled_time;

boolean is_bt_conn;

uint8_t *p_send_data;
uint8_t send_len;
uint8_t send_index;
uint32_t frame_timer;

void getError();
void calculatePID();
boolean sendToBluetooth();
void calcTilt();

void setup()
{
	pinMode(VOLTAGEDIVIDER, INPUT);
	pinMode(LED_DIGITAL_PIN, OUTPUT);
	pinMode(LED_SERIAL_PIN, OUTPUT);
	pinMode(LED_ENCODE_PIN, OUTPUT);

	/* 初始化一些变量 */
	tracker.mode = 1;
	p_send_data = 0;
	send_len = 0;
	send_index = 0;
	airplane_upload_time = 0;
	tracker_upload_time = 0;
	heart_upload_time = 0;
	compass_time = 0;
	oled_time = 0;
	SERVO_TILT_HAS_ARRIVED = true;
	//AccGain = 0;

	//测试代码初始化
	// tracker.mode = 1;
	// TParam.pid_p = 6000;
	// TParam.pid_i = 50;
	// TParam.pid_d = 200;
	// TParam.pid_divider = 15;
	// TParam.pid_max_error = 40;
	// TParam.tilt_min = 1100;
	// TParam.tilt_max = 2150;
	// TParam.pan_center = 1505;
	// TParam.pan_min_speed = 50;
	// TParam.compass_offset = 180;
	// TParam.compass_declination = 0;
	// TParam.start_tracking_distance = 10;
	/* ------------------------------------------ */

	//设置参考电压为默认值
	analogReference(BATTERYMONITORING_VREF_SOURCE);

	/* OLED屏初始化 */
	Wire.begin();
	oled.begin(&Adafruit128x64, OLED_I2C_ADDRESS);
	oled.set400kHz();
	oled.setFont(font8x8);
	oled.println("T Auto Tracker");
	oled.println("Version 1.0.0");
	oled.println("    ");
	delay(1000);
	/* ------------------------------------------ */

	HAS_ALT = false;
	HAS_FIX = false;
	HOME_SETED = false;
	BT_CONNECTED = false;
	BT_DATA_SENDING = false;

	/* 初始化电子指南针 */
	Wire.setClock(uint32_t(400000L));
	Wire.begin();
	oled.println("IMU Init...");
	imu.init();
	/* ------------------------------------------ */

	/* 初始化传输协议 */
	oled.println("Protocol Init...");
	tprotocol(&airplane, &tracker, &TParam);
	telemetry(&airplane);
	/* ------------------------------------------ */

	/* 初始化传输协议 */
	oled.println("Servos Init...");
	//initServos(&tracker_param);
	initServos(&TParam);
	/* ------------------------------------------ */

	/* 蓝牙串口 */
	digitalSerial.begin(BLUETOOTH_SERIAL_BAUDRATE);
	/* ------------------------------------------ */

	/* 远端GPS信号串口 */
	Serial.begin(DIGITAL_SERIAL_BAUDRATE);
	/* ------------------------------------------ */

	TParam.pid_divider = 15;
	// airplane_pos_time = millis();
	airplane_upload_time = millis();
	tracker_upload_time = millis() + 500;
	heart_upload_time = millis();
	compass_time = millis();
	oled_time = millis();
}

void loop()
{
	//int8_t i;

	digitalWrite(LED_DIGITAL_PIN, LOW);
	digitalWrite(LED_ENCODE_PIN, LOW);
	digitalWrite(LED_SERIAL_PIN, LOW);

	if (Serial.available())
	{
		uint8_t c = Serial.read();

		if (encodeTargetData(c) && HAS_ALT && HAS_FIX) digitalWrite(LED_DIGITAL_PIN, HIGH);

		// else if (tracker.mode == TRACKER_MODE_DEBUG)
		// {
		// 	//target heading in degree
		// 	if (c == 'H' || c == 'h') tracker.course = Serial.parseInt();
		// 	else if (c == 'T' || c == 't')
		// 	{
		// 		//tilt angle in degree
		// 		int value = Serial.parseInt();
		// 		if (value > 90) value = 90;
		// 		else if (value < 0)	value = 0;

		// 		oled.clear();
		// 		sprintf(oled_str, "Tilt set to:%03u", value);
		// 		oled.println(oled_str);

		// 		_servo_tilt_must_move = value;
		// 		SERVO_TILT_HAS_ARRIVED = false;
		// 	}
		// 	else if (c == 'M' || c == 'm')
		// 	{
		// 		//tilt angle in ms
		// 		tracker.pitching = Serial.parseInt();
		// 		SET_TILT_SERVO_SPEED(tracker.pitching);
		// 	}
		// 	else if (c == 'L' || c == 'l') TParam.pan_center = Serial.parseInt();
		// 	else if (c == 'P' || c == 'p') TParam.pid_p = Serial.parseInt();
		// 	else if (c == 'I' || c == 'i') TParam.pid_i = Serial.parseInt();
		// 	else if (c == 'D' || c == 'd') TParam.pid_d = Serial.parseInt();
		// 	else if (c == 'C' || c == 'c') calibrate_compass(TParam.pan_center);
		// 	else if (c == 'S' || c == 's') TParam.pan_min_speed = Serial.parseInt();
		// 	else if (c == 'O' || c == 'o') TParam.compass_offset = Serial.parseInt();
		// 	else if (c == 'E' || c == 'e') TParam.pid_max_error = Serial.parseInt();
		// 	else if (c == 'G' || c == 'g') TRACKING_STARTED = Serial.parseInt();
		// 	else if (c == 'W' || c == 'w') tracker.outpwm = Serial.parseInt();

		// 	digitalWrite(LED_DIGITAL_PIN, HIGH);
		// }
	}

	if (digitalSerial.available())
	{
		uint8_t c = digitalSerial.read();
		if (encodeTrackerData(c))
			digitalWrite(LED_ENCODE_PIN, HIGH);
	}

	//根据当前的模式处理对应的数据
	switch (tracker.mode)
	{
		case TRACKER_MODE_MANUAL:
			break;
		case TRACKER_MODE_AUTO:
			//Only track if home is seted.
			if (HAS_FIX && HOME_SETED)
			{
				airplane.distance = distance_between(tracker.lat / 10000000.0f, tracker.lon / 10000000.0f,
													airplane.lat / 10000000.0f, airplane.lon / 10000000.0f);
				tracker.course = course_to(tracker.lat / 10000000.0f, tracker.lon / 10000000.0f,
										airplane.lat / 10000000.0f, airplane.lon / 10000000.0f) *
								10.0f;

				HAS_FIX = false;

				//Only track if tracking process started.
				if (!TRACKING_STARTED)
				{
					//if plane is START_TRACKING_DISTANCE meter away from tracker start tracking process.
					if (airplane.distance >= TParam.start_tracking_distance)
						TRACKING_STARTED = true;
				}

				// unsigned long dt = (millis() - airplane_pos_time);

				// if (dt > 0) {
				// 	uint16_t er = abs(tracker.course - last_course);

				// 	if (er > 1800) er = 3600 - er;

				// 	AccGain = (er / (dt / 100.0)) * 1.1;

				// 	last_course = tracker.course;
				// 	airplane_pos_time = millis();
				// }
			}

			if (SERVO_TILT_HAS_ARRIVED && HAS_ALT && TRACKING_STARTED && airplane.distance < 120000 && airplane.alt < 500000)
			{
				calcTilt();
				tracker.pitching = _servo_tilt_must_move;
				HAS_ALT = false;

				// if(_servo_tilt_must_move > 88) 
				// {
				// 	Serial.print("T LAT:");
				// 	dtostrf(airplane.lat / 10000000.0f, 10, 5, oled_str);
				// 	Serial.println(oled_str);
				// 	Serial.print("T LON:");
				// 	dtostrf(airplane.lon / 10000000.0f, 10, 5, oled_str);
				// 	Serial.println(oled_str);
				// 	sprintf(oled_str, "PLAN_ALT:%05d  DST:%05u", (int)(airplane.alt / 100.0f), airplane.distance);
				// 	Serial.println(oled_str);
				// 	sprintf(oled_str, "TKER_ALT:%05u  S:%05u", (int)(tracker.alt / 100.0f), airplane.speed);
				// 	Serial.println(oled_str);
				// }
			}
			break;
		case TRACKER_MODE_DEBUG:
			if (TRACKING_STARTED) TRACKING_STARTED = false;
			//处理下发下来要执行的命令
			uint8_t cmd = getReceiveCmd();

			if (cmd != 0)
			{
				switch (cmd)
				{
					case TAG_CTR_CALIBRATE:
						oled.clear();
						oled.setCursor(0, 0);
						oled.println("Calibrating");
						imu.calibrate_compass(TParam.pan_center);
						oled.clear();
						break;
					case TAG_CTR_MODE:
						oled.clear();
						break;
					case TAG_CTR_TILT:
						_servo_tilt_must_move = tracker.pitching;
						SERVO_TILT_HAS_ARRIVED = false;
						#ifdef DEBUG
							Serial.print("CTR_TILT:");
							Serial.println(_servo_tilt_must_move);
						#endif
						break;
				}
			}
			break;
	}

	// only update pan value if there is new data
	if ((TRACKING_STARTED || AUTO_POINT_TO_NORTH) && NEW_HEADING)
	{
		getError();		// Get position error
		calculatePID(); // Calculate the PID output from the error
		NEW_HEADING = false;
	}

	SET_PAN_SERVO_SPEED(tracker.outpwm);
	servo_tilt_update(_servo_tilt_must_move);

	// refresh rate of compass is 75Hz -> 13.333ms to refresh the data
	// we update the heading every 14ms to get as many samples into the smooth array as possible
	if (millis() > compass_time)
	{
		compass_time = millis() + 14;

		// double heading = (imu.getHeading() * 10) + TParam.compass_declination - (TParam.compass_offset) * 10;

		// if (heading < 0)
		// 	tracker.heading = (uint16_t)(3600 + heading);
		// else
		// 	tracker.heading = (uint16_t)heading;

		double heading = imu.getHeading();
		if (heading < 0) heading += 360;

		int16_t tmp = (heading * 10) + TParam.compass_declination + (TParam.compass_offset * 10);
		if (tmp < 0) tmp += 3600;
		tracker.heading = (uint16_t)tmp;

		NEW_HEADING = true;
	}

	// 通过蓝牙上报飞机状态
	if (BT_CONNECTED && millis() > airplane_upload_time)
	{
		airplane_upload_time = millis() + 500;
		setSendCmd(CMD_U_AIRPLANE);
	}

	// 通过蓝牙上报设备状态
	if (BT_CONNECTED && millis() > tracker_upload_time)
	{
		tracker_upload_time = millis() + 500;
		setSendCmd(CMD_U_TRACKER);
	}

	if (millis() > heart_upload_time)
	{
		heart_upload_time = millis() + 25000;
		setSendCmd(CMD_U_HEARTBEAT);

		if (millis() > getLastHeartBeetAck() + 30000)
			BT_CONNECTED = false;
	}

	//获取需要发送的数据
	if (send_index <= 0)
	{
		getSendData(p_send_data, send_len);
		send_index = send_len;
	}

	if (send_index > 0)
	{
		//发送数据
		if (sendToBluetooth())
			digitalWrite(LED_SERIAL_PIN, HIGH);
	}

	if (millis() > oled_time)
	{
		long t = millis() - oled_time;

		getBatVoltage(&tracker.voltage);

		if (tracker.mode == TRACKER_MODE_DEBUG)
		{
			oled.setCursor(0, 0);
			sprintf(oled_str, "H:%03u A:%03u", tracker.heading / 10, tracker.course / 10);
			oled.println(oled_str);
			sprintf(oled_str, "PWM:%04d PID:%03d", tracker.outpwm, PID);
			oled.println(oled_str);
			sprintf(oled_str, "T0:%04u T90:%04u", TParam.tilt_0, TParam.tilt_90);
			oled.println(oled_str);
			sprintf(oled_str, "C:%04d E:%03u", TParam.pan_center, TParam.pid_max_error);
			oled.println(oled_str);
			sprintf(oled_str, "O:%03d    V:%02u.%02u", TParam.compass_offset, tracker.voltage / 100, tracker.voltage % ((uint16_t)(tracker.voltage / 100) * 100));
			oled.println(oled_str);
			sprintf(oled_str, "P:%04d         ", TParam.pid_p);
			oled.println(oled_str);
			sprintf(oled_str, "I:%04d         ", TParam.pid_i);
			oled.println(oled_str);
			sprintf(oled_str, "D:%04d         ", TParam.pid_d);
			oled.println(oled_str);
		}
		else if (tracker.mode == TRACKER_MODE_AUTO)
		{
			oled.setCursor(0, 0);
			sprintf(oled_str, "H:%03u A:%03u S:%02u", tracker.heading / 10, tracker.course / 10, airplane.sats);
			oled.println(oled_str);
			sprintf(oled_str, "A:%05d  D:%05u", (int)(airplane.alt / 100.0f), airplane.distance);
			oled.println(oled_str);
			oled.print("T LAT:");
			dtostrf(airplane.lat / 10000000.0f, 10, 5, oled_str);
			oled.println(oled_str);
			oled.print("T LON:");
			dtostrf(airplane.lon / 10000000.0f, 10, 5, oled_str);
			oled.println(oled_str);
			//sprintf(oled_str, "V:%02u.%02u  T:%05u", tracker.voltage / 100, tracker.voltage % ((uint16_t)(tracker.voltage / 100) * 100), t);
			sprintf(oled_str, "V:%02u.%02u  H:%05u", tracker.voltage / 100, tracker.voltage % ((uint16_t)(tracker.voltage / 100) * 100), airplane.heading);
			oled.println(oled_str);
			sprintf(oled_str, "A:%05u  S:%05u", (int)(tracker.alt / 100.0f), airplane.speed);
			oled.println(oled_str);
			oled.print("L LAT:");
			dtostrf(tracker.lat / 10000000.0f, 10, 5, oled_str);
			oled.println(oled_str);
			oled.print("L LON:");
			dtostrf(tracker.lon / 10000000.0f, 10, 5, oled_str);
			oled.println(oled_str);
		}

		oled_time = millis() + 500;
	}
}

/*
	将数据发送到蓝牙
*/
boolean sendToBluetooth()
{
	boolean byte_dropped = false;
	boolean packet_dropped = false;

	if (send_index == send_len) frame_timer = millis();

	// for (i = 0; i < send_len; i++)
	// {
	// Serial.print(p_send_data[send_len - send_index], HEX);
	//Serial.print(" ");
	if (digitalSerial.write(p_send_data[send_len - send_index]) == 0)
	{
		//buffer is full, flush & retry.
		digitalSerial.flush();
		byte_dropped = true;
		if (millis() - frame_timer >= 100)
		{
			// drop the whole frame, it's too old. Will resend a fresh one.
			packet_dropped = true;
			// break;
		}
	}
	else
	{
		send_index -= 1;
	}

	if (byte_dropped)
	{
		// i--; //resend dropped byte
		byte_dropped = false;
	}
	if (packet_dropped)
	{
		BT_DATA_SENDING = false;
		send_index = 0;
		// break;
	}

	// int32_t currentmicros = micros();
	// while ((micros() - currentmicros) < softserial_delay );// wait at least 1 byte is sent
	// }

	//Serial.println("");
	return !packet_dropped;
}

//////////////// Me he quedado aquí revisando los #ifdef
//Tilt angle alpha = atan(alt/dist)
void calcTilt()
{
	uint16_t alpha = 0;

	//this will fix an error where the tracker randomly points up when plane is lower than tracker
	if (airplane.alt < tracker.alt)
	{
		airplane.alt = tracker.alt;
	}

	//prevent division by 0
	if (airplane.distance == 0)
	{
		// in larger altitude 1m distance shouldnt mess up the calculation.
		//e.g. in 100m height and dist 1 the angle is 89.4° which is actually accurate enough
		airplane.distance = 1;
	}

	alpha = toDeg(atan((airplane.alt / 100.00f - tracker.alt / 100.00f) / airplane.distance));

	//just for current tests, later we will have negative tilt as well
	if (alpha < 0)
		alpha = 0;
	else if (alpha > 90)
		alpha = 90;

	_servo_tilt_must_move = alpha;
	SERVO_TILT_HAS_ARRIVED = false;
	//moveServoTilt(alpha);
	// SET_TILT_SERVO_SPEED(map(alpha, 0, 90, TParam.titl_0, TParam.titl_90));
}

void getError(void)
{
	int16_t delta = tracker.course - tracker.heading;

	// Serial.println("-----------Error[11]------------");
	// Serial.print("[0]");Serial.print(delta);
	// shift error values
	for (byte i = 0; i < 10; i++)
	{
		Error[i + 1] = Error[i];
		// Serial.print(" [");Serial.print(i + 1);Serial.print("]");Serial.print(Error[i + 1]);
	}
	// Serial.println("");
	// Serial.println("------------------");

	if (delta > 1800)
	{
		delta -= 3600;
	}
	else if (delta < -1800)
	{
		delta += 3600;
	}
	// load new error into top array spot
	Error[0] = delta;
}

void calculatePID(void)
{
	float index;

	//变积分过程
	if (abs(Error[0]) > 300)
	{ 
		index = 0.0;
	}
	else if (abs(Error[0]) < 50)
	{
		index = 1.0;
		Accumulator += Error[0];
	}
	else
	{
		index = (300 - abs(Error[0])) / 300;
		Accumulator += Error[0];
	}

	PID = TParam.pid_p * Error[0] + index * TParam.pid_i * Accumulator + TParam.pid_d * (Error[0] - Error[10]);
	if (TParam.pid_divider > 0)
		PID = PID >> TParam.pid_divider;

	if (PID >= 500)
		PID = 500;
	if (PID <= -500)
		PID = -500;

	if (Error[0] > TParam.pid_max_error)
	{
		tracker.outpwm = TParam.pan_center + PID + TParam.pan_min_speed;
	}
	else if (Error[0] < -1 * TParam.pid_max_error)
	{
		tracker.outpwm = TParam.pan_center + PID - TParam.pan_min_speed;
	}
	else
	{
		tracker.outpwm = TParam.pan_center;
	}
}
