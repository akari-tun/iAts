/*
iAts_TX is a AAT transmit progam, its from GhettoProxy on https://github.com/KipK/Ghettostation/tree/master/GhettoProxy
I rewrote the code that parses the GPS data and sends the LTM protocol.

作者：TanJun
用途：AAT transmit for iAts

*/

#include <FastSerial.h>
#include <avr/pgmspace.h>
#include <Arduino.h>
#include <AP_Math.h>
#include <Metro.h>
#include <AltSoftSerial.h>

#include "Config.h"
#include "Defines.h"
#include "iAts_TX.h"
#include "LightTelemetry.cpp"
#ifdef DEBUG
#include <MemoryFree.h>
#endif
/*
 * BOF preprocessor bug prevent
 */
#define nop() __asm volatile("nop")
#if 1
nop();
#endif
/*
 * EOF preprocessor bug prevent
*/

#ifdef PROTOCOL_MAVLINK
#include <AP_Common.h>
#include <GCS_MAVLink.h>
#include "Mavlink.cpp"
#define MAVLINK_CRC_EXTRA 0
#endif
#ifdef PROTOCOL_NMEA
#include "GPS_NMEA.cpp"
#include "GPS.cpp"
#endif

//################################### SETTING OBJECTS ###############################################

//##### LOOP RATES
Metro loop10hz = Metro(100); //10hz loop
Metro loop50hz = Metro(20); //10hz loop
#ifdef DEBUG
Metro loopDebug = Metro(500);
#endif
//#################################### SETUP LOOP ####################################################

bool gps_ready = false;
bool isSending = false;
bool led = false;

void setup()
{
  //start serial com
  init_serial();
#ifdef PROTOCOL_GPS
  GPS.Init();
#endif
#ifdef PROTOCOL_MAVLINK
  mavlink_comm_0_port = &Serial;
#endif

  pinMode(LED_GPS_PIN, OUTPUT);
  pinMode(LED_OUT_PIN, OUTPUT);
  //LED off
  digitalWrite(LED_GPS_PIN, LOW);
  digitalWrite(LED_OUT_PIN, LOW);

  gps_ready = false;
}

//######################################## MAIN LOOP #####################################################################
void loop()
{
  if (get_telemetry() && !gps_ready) gps_ready = true;

  //LED Controller
  if (loop50hz.check()) {
    if (led && (gps_ready || isSending)) digitalWrite(LED_GPS_PIN, HIGH);
    else digitalWrite(LED_GPS_PIN, LOW);

    led = !led;
  }

#ifdef DEBUG
  if (loopDebug.check())
  {
    debug_proxy();
  }
#endif

#ifdef ONE_BYTE_PARSE_AND_SEND
  if (send()) {
    isSending = false;
    if (loop10hz.check() && gps_ready) {
      LTM();
      gps_ready = false;
      isSending = true;
      frame_timer = millis();
    }
  }
#else
  if (loop10hz.check() && gps_ready) {
    LTM();
    gps_ready = false;
    frame_timer = millis();
    while (!send()) { }
  }
#endif
}

//######################################## TELEMETRY FUNCTIONS #############################################
void init_serial()
{
  Serial.begin(INPUT_BAUD);
  SerialPort2.begin(OUTPUT_BAUD);
}

//Preparing adding other protocol
bool get_telemetry()
{

#if defined(PROTOCOL_MAVLINK) // Ardupilot / PixHawk / Taulabs ( mavlink output ) / Other
  if (enable_frame_request == 1)
  { //Request rate control
    enable_frame_request = 0;
    if (!PASSIVEMODE)
    {
      request_mavlink_rates();
    }
  }

  read_mavlink();

  if (uav_fix_type >= 0x02) gps_ready = true;
#endif

#if defined(PROTOCOL_NMEA) || defined(PROTOCOL_UBLOX)
  return gps_read();
#endif
}

#ifdef DEBUG

void debug_proxy()
{
  //SerialPort1.print("timer ");
  //int currenttime = millis();
  //SerialPort1.println(currenttime);
  Serial.print("mem ");
  int freememory = freeMem();
  Serial.println(freememory);
  Serial.print("uav_alt = ");
  Serial.println(uav_alt);
  Serial.print("uav_pitch = ");
  Serial.println(uav_pitch);
  Serial.print("uav_roll = ");
  Serial.println(uav_roll);
  Serial.print("uav_heading = ");
  Serial.println(uav_heading);
  Serial.print("uav_lat = ");
  Serial.println(uav_lat);
  Serial.print("uav_lon = ");
  Serial.println(uav_lon);
  Serial.print("uav_speed = ");
  Serial.println(uav_groundspeed);
  Serial.print("uav_fix_type = ");
  Serial.println(uav_fix_type);
  Serial.print("uav_satellites_visible = ");
  Serial.println(uav_satellites_visible);
  //SerialPort1.print("softserial_delay = ");
  //SerialPort1.println(softserial_delay);
  //SerialPort1.print("packet_drops = ");
  //SerialPort1.println(packet_drops);
  //SerialPort1.print("parse_error = ");
  //SerialPort1.println(parse_error);
}
#endif
