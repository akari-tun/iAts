#ifndef EEPROM_FUNCTIONS
#define EEPROM_FUNCTIONS

#include <inttypes.h>


//// functions to store a byte/int/float to an eeprom position
//short LoadByteFromEEPROM(uint8_t position);
//int   LoadIntegerFromEEPROM(uint8_t position);
//float LoadFloatFromEEPROM(uint8_t position);
//
//// functions to load a byte/int/float from an eeprom position
//void  StoreShortToEEPROM(short value, uint8_t position);
//void  StoreIntegerToEEPROM(int value, uint8_t position);
//void  StoreFloatToEEPROM(float value, uint8_t position);

void StoreToEEPROM_u8(uint8_t value, uint8_t position);
void StoreToEEPROM_u16(uint16_t value, uint8_t position);
void StoreToEEPROM_u32(uint32_t value, uint8_t position);

uint8_t LoadFromEEPROM_u8(uint8_t position);
uint16_t LoadFromEEPROM_u16(uint8_t position);
uint32_t LoadFromEEPROM_u32(uint8_t position);


#endif



