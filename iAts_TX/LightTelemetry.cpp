/* #################################################################################################################
 * LightTelemetry protocol (LTM)
 *
 * Ghettostation one way telemetry protocol for really low bitrates (1200/2400 bauds). 
 *			   
 * Protocol details: 3 different frames, little endian.
 *   G Frame (GPS position) (2hz @ 1200 bauds , 5hz >= 2400 bauds): 18BYTES
 *    0x24 0x54 0x47 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF  0xFF   0xC0   
 *     $     T    G  --------LAT-------- -------LON--------- SPD --------ALT-------- SAT/FIX  CRC
 *   A Frame (Attitude) (5hz @ 1200bauds , 10hz >= 2400bauds): 10BYTES
 *     0x24 0x54 0x41 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xC0   
 *      $     T   A   --PITCH-- --ROLL--- -HEADING-  CRC
 *   S Frame (Sensors) (2hz @ 1200bauds, 5hz >= 2400bauds): 11BYTES
 *     0x24 0x54 0x53 0xFF 0xFF  0xFF 0xFF    0xFF    0xFF      0xFF       0xC0     
 *      $     T   S   VBAT(mv)  Current(ma)   RSSI  AIRSPEED  ARM/FS/FMOD   CRC
 * ################################################################################################################# */

#if defined(PROTOCOL_LIGHTTELEMETRY)
#include "LightTelemetry.h"

uint8_t LTBuff[LTM_BUFFER_SIZE];
uint8_t SendLen = 0;
uint8_t SendIndex = 0;
uint8_t LTCrc = 0;
int32_t lastmicros = 0;

static bool send() 
{
    if (SendIndex >= SendLen) return true;
    if ((micros() - lastmicros) < softserial_delay) return false;
    
    if (SerialPort2.write(LTBuff[SendIndex]) == 0) {
		//buffer is full, flush & retry.
		SerialPort2.flush();
		if (millis() - frame_timer >= 100) {
			// drop the whole frame, it's too old. Will resend a fresh one.
            SendIndex = SendLen;
		}
	}
	else {
		SendIndex += 1;
        lastmicros = micros();
	}

    return SendIndex >= SendLen;
}

// static bool send_LTM_Packet(uint8_t *LTPacket, uint8_t LTPacket_size)
// {
//     //calculate Checksum
//     uint8_t LTCrc = 0x00;
//     int i;
//     for (i = 3; i < LTPacket_size-1; i++) {
//         LTCrc ^= LTPacket[i];
//     }
//     LTPacket[LTPacket_size-1]=LTCrc;
//     boolean byte_dropped = false;
//     boolean packet_dropped = false;
//     uint32_t frame_timer = millis();
//     for (i = 0; i<LTPacket_size; i++) {
//         if(SerialPort2.write(LTPacket[i]) == 0 ) {
//          //buffer is full, flush & retry.
//             SerialPort2.flush(); 
//             byte_dropped = true;
//             if (millis() - frame_timer >= 100) {
//             // drop the whole frame, it's too old. Will resend a fresh one.
//                packet_dropped = true;
               
//                break;
//             }
        
//         }

//         if (byte_dropped) {
//             i--; //resend dropped byte  
//             byte_dropped = false;
//         }

//         if (packet_dropped) {
//             break;
//         }
//         int32_t currentmicros = micros();
//         while ( (micros() - currentmicros) < softserial_delay ){
//             ;// wait at least 1 byte is sent
//         }
//     }
//     if (packet_dropped) {
//         return false;
//     } else {
//         return true;
//     }
// }


// GPS frame
void LTM_Gframe()
{
    SendLen = LTM_GFRAME_SIZE;
    //protocol: START(2 bytes)FRAMEID(1byte)LAT(cm,4 bytes)LON(cm,4bytes)SPEED(m/s,1bytes)ALT(cm,4bytes)SATS(6bits)FIX(2bits)CRC(xor,1byte)
    //START
    LTBuff[0]=0x24; //$
    LTBuff[1]=0x54; //T
    //FRAMEID
    LTBuff[2]=0x47; // G ( gps frame at 5hz )
    //PAYLOAD
    LTBuff[3]=(uav_lat >> 8*0) & 0xFF;
    LTBuff[4]=(uav_lat >> 8*1) & 0xFF;
    LTBuff[5]=(uav_lat >> 8*2) & 0xFF;
    LTBuff[6]=(uav_lat >> 8*3) & 0xFF;
    LTBuff[7]=(uav_lon >> 8*0) & 0xFF;
    LTBuff[8]=(uav_lon >> 8*1) & 0xFF;
    LTBuff[9]=(uav_lon >> 8*2) & 0xFF;
    LTBuff[10]=(uav_lon >> 8*3) & 0xFF;
    LTBuff[11]=uav_groundspeed >> 8*0;
    LTBuff[12]=(uav_alt >> 8*0) & 0xFF;
    LTBuff[13]=(uav_alt >> 8*1) & 0xFF;
    LTBuff[14]=(uav_alt >> 8*2) & 0xFF;
    LTBuff[15]=(uav_alt >> 8*3) & 0xFF;
    LTBuff[16]= ((uav_satellites_visible << 2 )& 0xFF) | (uav_fix_type & 0b00000011) ; // last 6 bits: sats number, first 2:fix type (0,1,2,3)
    // LTBuff[11]=((uint16_t)uav_groundspeed >> 8*0) & 0xFF;
    // LTBuff[12]=((uint16_t)uav_groundspeed >> 8*1) & 0xFF;
    // LTBuff[13]=(uav_alt >> 8*0) & 0xFF;
    // LTBuff[14]=(uav_alt >> 8*1) & 0xFF;
    // LTBuff[15]=(uav_alt >> 8*2) & 0xFF;
    // LTBuff[16]=(uav_alt >> 8*3) & 0xFF;
    // LTBuff[17]= ((uav_satellites_visible << 2 )& 0xFF) | (uav_fix_type & 0b00000011) ; // last 6 bits: sats number, first 2:fix type (0,1,2,3)
}

//Sensors frame
static void LTM_Sframe() 
{
    SendLen = LTM_SFRAME_SIZE;
    //START
    LTBuff[0]=0x24; //$
    LTBuff[1]=0x54; //T
    //FRAMEID
    LTBuff[2]=0x53; //S 
    //PAYLOAD
    LTBuff[3]=(uav_bat >> 8*0) & 0xFF;                                                    //vbat converted in mv
    LTBuff[4]=(uav_bat >> 8*1) & 0xFF;
    LTBuff[5]=(uav_amp >> 8*0) & 0xFF;                                                    //consumed current in ma.
    LTBuff[6]=(uav_amp >> 8*1) & 0xFF;
    LTBuff[7]=(uav_rssi >> 8*0) & 0xFF;                                                   
    LTBuff[8]=(uav_airspeed >> 8*0) & 0xFF;                                               // no airspeed in multiwii/baseflight
    LTBuff[9]= ((uav_flightmode << 2)& 0xFF ) | ((uav_failsafe << 1)& 0b00000010 ) | (uav_arm & 0b00000001) ; // last 6 bits: flight mode, 2nd bit: failsafe, 1st bit: Arm status.
    // Flight mode(0-19): 0: Manual, 1: Rate, 2: Attitude/Angle, 3: Horizon, 4: Acro, 5: Stabilized1, 6: Stabilized2, 7: Stabilized3,
    // 8: Altitude Hold, 9: Loiter/GPS Hold, 10: Auto/Waypoints, 11: Heading Hold / headFree, 
    // 12: Circle, 13: RTH, 14: FollowMe, 15: LAND, 16:FlybyWireA, 17: FlybywireB, 18: Cruise, 19: Unknown
}

// Attitude frame
static void LTM_Aframe() 
{
    SendLen = LTM_AFRAME_SIZE;
    //A Frame: $T(2 bytes)A(1byte)PITCH(2 bytes)ROLL(2bytes)HEADING(2bytes)CRC(xor,1byte)
    //START
    LTBuff[0]=0x24; //$
    LTBuff[1]=0x54; //T
    //FRAMEID
    LTBuff[2]=0x41; //A 
    //PAYLOAD
    LTBuff[3]=(uav_pitch >> 8*0) & 0xFF;
    LTBuff[4]=(uav_pitch >> 8*1) & 0xFF;
    LTBuff[5]=(uav_roll >> 8*0) & 0xFF;
    LTBuff[6]=(uav_roll >> 8*1) & 0xFF;
    LTBuff[7]=(uav_heading >> 8*0) & 0xFF;
    LTBuff[8]=(uav_heading >> 8*1) & 0xFF;
}

static void LTM() {

    if (ltm_scheduler == 1) {    // is odd
        LTM_Aframe();
    }
    else {                        // is even
        LTM_Gframe();
    }
    
    ltm_scheduler++;

    if (ltm_scheduler > 9) ltm_scheduler = 1;

    LTCrc = 0;
    int i;
    for (i = 3; i < SendLen - 1; i++) {
        LTCrc ^= LTBuff[i];
    }

    LTBuff[SendLen - 1] = LTCrc;
    SendIndex = 0;
}

#endif