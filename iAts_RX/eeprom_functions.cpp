#include <Arduino.h>
#include <EEPROM.h>
#include "eeprom_functions.h"

//typedef union
//{
//  float as_float;
//  uint8_t as_byte[4];
//} data32_t;
//
//
//short LoadByteFromEEPROM(uint8_t position)
//{
//  return EEPROM.read(position);
//}
//
//
//int LoadIntegerFromEEPROM(uint8_t position)
//{
//  return (EEPROM.read(position) + (EEPROM.read(position + 1) << 8));
//}
//
//
//float LoadFloatFromEEPROM(uint8_t position)
//{
//  data32_t data;
//  data.as_byte[0] = EEPROM.read(position);
//  data.as_byte[1] = EEPROM.read(position + 1);
//  data.as_byte[2] = EEPROM.read(position + 2);
//  data.as_byte[3] = EEPROM.read(position + 3);
//
//  return data.as_float;
//}
//
//
//void StoreShortToEEPROM(short value, uint8_t position)
//{
//  EEPROM.write(position, value);
//}
//
//
//void StoreIntegerToEEPROM(int value, uint8_t position)
//{
//  EEPROM.write(position,     (uint8_t)value);
//  EEPROM.write(position + 1, (uint8_t)(value >> 8));
//}
//
//
//void StoreFloatToEEPROM(float value, uint8_t position)
//{
//  // copy to data32_t for easier access
//  data32_t tmp = {value};
//
//  EEPROM.write(position,     tmp.as_byte[0]);
//  EEPROM.write(position + 1, tmp.as_byte[1]);
//  EEPROM.write(position + 2, tmp.as_byte[2]);
//  EEPROM.write(position + 3, tmp.as_byte[3]);
//}

void StoreToEEPROM_u8(uint8_t value, uint8_t position)
{
	EEPROM.write(position, value);
}


void StoreToEEPROM_u16(uint16_t value, uint8_t position)
{
	EEPROM.write(position, (uint8_t)value);
	EEPROM.write(position + 1, (uint8_t)(value >> 8));
}


void StoreToEEPROM_u32(uint32_t value, uint8_t position)
{
	EEPROM.write(position, (uint8_t)value);
	EEPROM.write(position + 1, (uint8_t)(value >> 8));
	EEPROM.write(position + 2, (uint8_t)(value >> 16));
	EEPROM.write(position + 3, (uint8_t)(value >> 24));
}

uint8_t LoadFromEEPROM_u8(uint8_t position)
{
	return EEPROM.read(position);
}


uint16_t LoadFromEEPROM_u16(uint8_t position)
{
	return (EEPROM.read(position) + (EEPROM.read(position + 1) << 8));
}


uint32_t LoadFromEEPROM_u32(uint8_t position)
{
	uint32_t v = 0;
	v = EEPROM.read(position);
	v |= (uint16_t)EEPROM.read(position + 1) << 8;
	v |= (uint32_t)EEPROM.read(position + 1) << 16;
	v |= (uint32_t)EEPROM.read(position + 1) << 24;

	return v;
}
