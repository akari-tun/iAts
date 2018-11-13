/*########################################### BOARD PINOUTS #########################################################
 Pins for Arduino Mini
 *SERIAL TELEMETRY INPUT PINOUT:
 
     Use TX/RX pads
     
 *SERIAL LTM OUTPUT PINOUT ( AltSoftSerial):
 
     TX: D9 on arduino mini/nano 

*/
/*############################################## CONFIGURATION ####################################################
 # Comment/uncomment/edit according to your needs.
 ##################################################################################################################*/

//########## OPTIONS ###############################################################################################
#define PROTOCOL_LIGHTTELEMETRY

//INPUT PROTOCOL
// Choose only one.

//#define PROTOCOL_UAVTALK                      // OpenPilot / Taulabs protocol
//#define PROTOCOL_MSP                          // MSP from Multiwii / Baseflight
//#define PROTOCOL_MAVLINK                      // Mavlink for Ardupilot / Autoquad / PixHawk / Taulabs (UAVOmavlinkBridge)
#define PROTOCOL_NMEA                           // GPS NMEA ASCII protocol
//#define PROTOCOL_UBLOX                        // GPS UBLOX binary protocol

//!uncomment to use altitude from Baro. Use GPS alt if commented.
//#define BARO_ALT

//INPUT BAUDRATE
#define INPUT_BAUD 38400  //input bps

// GhettoProxy just listen & convert in passive mode. If disabled, it will initiate queries packet.
#define PASSIVEMODE 1

//OUTOPUT BAUDRATE
#define OUTPUT_BAUD 1200   //output bps

//#define ONE_BYTE_PARSE_AND_SEND
//#define DEBUG