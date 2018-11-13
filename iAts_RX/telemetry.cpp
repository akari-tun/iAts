#include <Arduino.h>
#include "telemetry.h"

// Enum machine states
enum LtmDataState
{
  IDLE,
  STATE_START1,
  STATE_START2,
  STATE_MSGTYPE,
  STATE_DATA
};

//
#define LTM_HEADER_START1 0x24 //$
#define LTM_HEADER_START2 0x54 //T
#define LTM_GFRAME 0x47        //G Frame
#define LTM_GFRAME_LENGTH 18
#define LTM_AFRAME 0x41        //A Frame
#define LTM_AFRAME_LENGTH 10

//
static uint8_t LTM_Buffer[LTM_GFRAME_LENGTH - 4];
static uint8_t LTM_Index;
static uint8_t LTM_cmd;
static uint8_t LTM_chk;
static uint8_t LTM_read_index;
static uint8_t LTM_frame_length;
static uint8_t dataState = IDLE;

//data needed to buffer
static Airplane *PTarget;

void parseLTM_GFRAME();
void parseLTM_AFRAME();
uint8_t ltmread_u8();
uint16_t ltmread_u16();
uint32_t ltmread_u32();

void telemetry(Airplane *target)
{
  PTarget = target;
}

boolean encodeTargetData(uint8_t c)
{
  boolean ret = true;

  if (dataState == IDLE && c == '$')
  {
    dataState = STATE_START1;
    #ifdef TELEMETRY_DEBUG
    Serial.print("STATE_LEAD:");
    Serial.println(c, HEX);
    #endif
  }
  else if (dataState == STATE_START1 && c == 'T')
  {
    dataState = STATE_START2;
    #ifdef TELEMETRY_DEBUG
    Serial.print("STATE_START:");
    Serial.println(c, HEX);
    #endif
  }
  else if (dataState == STATE_START2)
  {
    switch (c)
    {
      case 'G':
        LTM_frame_length = LTM_GFRAME_LENGTH;
        dataState = STATE_MSGTYPE;
        break;
      case 'A':
        LTM_frame_length = LTM_AFRAME_LENGTH;
        dataState = STATE_MSGTYPE;
        break;
      case 'S':
        dataState = IDLE;
        return false;
        break;
      default:
        dataState = IDLE;
        ret = false;
    }
    LTM_cmd = c;
    LTM_Index = 0;
    #ifdef TELEMETRY_DEBUG
    Serial.print("STATE_FRAME:");
    Serial.println(c, HEX);
    #endif
  }
  else if (dataState == STATE_MSGTYPE)
  {
    if (LTM_Index == 0)
    {
      LTM_chk = c;
      #ifdef TELEMETRY_DEBUG
      Serial.print("DATA:");
      #endif
    }
    else
    {
      LTM_chk ^= c;
    }
    if (LTM_Index == LTM_frame_length - 4)
    { // received checksum byte
      #ifdef TELEMETRY_DEBUG
      Serial.println("");
      #endif
      if (LTM_chk == 0)
      {
        #ifdef TELEMETRY_DEBUG
        Serial.println("SUCCESS");
        #endif
        //Telemetry ok
        LTM_read_index = 0;
        switch (LTM_cmd)
        {
          case 'G':
            parseLTM_GFRAME();
            break;
          case 'A':
            parseLTM_AFRAME();
            break;
        }
        
        dataState = IDLE;
      }
      else
      { // wrong checksum, drop packet
        dataState = IDLE;
        ret = false;
        #ifdef TELEMETRY_DEBUG
        Serial.println("FAIL");
        #endif
      }
    }
    else 
    {
      LTM_Buffer[LTM_Index++] = c;
      #ifdef TELEMETRY_DEBUG
      Serial.print(c, HEX);
      Serial.print(" ");
      #endif
    }
  }

  return ret;
}

void parseLTM_GFRAME()
{
  if (LTM_cmd == LTM_GFRAME)
  {
    PTarget->lat = ltmread_u32();
    PTarget->lon = ltmread_u32();
    PTarget->speed = ltmread_u8();
    PTarget->alt = ltmread_u32();
    uint8_t satsfix = ltmread_u8();
    PTarget->sats = (satsfix >> 2) & 0xFF;
    PTarget->fix_type = satsfix & 0b00000011;

    if (PTarget->sats >= 5) HAS_FIX = true;
    if (PTarget->alt > 0) HAS_ALT = true;
  }
}

void parseLTM_AFRAME()
{
  if (LTM_cmd == LTM_AFRAME)
  {
    PTarget->pitch = ltmread_u16();
    PTarget->roll = ltmread_u16();
    PTarget->heading = ltmread_u16();
  }
}

uint8_t ltmread_u8()
{
  return LTM_Buffer[LTM_read_index++];
}

uint16_t ltmread_u16()
{
  uint16_t t = ltmread_u8();
  t |= (uint16_t)ltmread_u8() << 8;
  return t;
}

uint32_t ltmread_u32()
{
  uint32_t t = ltmread_u16();
  t |= (uint32_t)ltmread_u16() << 16;
  return t;
}