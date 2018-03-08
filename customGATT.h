#ifndef BLUETOOTH_H
#define BLUETOOTH_H
#include <bluefruit.h>

#define UUID16_SVC_ENVIRONMENTAL_SENSING  0x181A
#define UUID16_CHR_TEMPERATURE            0x2A6E
#define UUID16_CHR_HUMIDITY               0x2A6F
#define UUID16_CHR_PRESSURE               0x2A6D
//#define UUID128_CHR_TVOC                  {0xc2, 0x1a, 0xcb, 0xcd, 0xab, 0x5e, 0x42, 0x38, 0x90, 0x96, 0xff, 0xb2, 0x11, 0xcb, 0xbb, 0xe0}
uint8_t UUID128_CHR_TVOC[] = {0xc2, 0x1a, 0xcb, 0xcd, 0xab, 0x5e, 0x42, 0x38, 0x90, 0x96, 0xff, 0xb2, 0x11, 0xcb, 0xbb, 0xe0};




BLEService        environmental_sensing_service = BLEService(UUID16_SVC_ENVIRONMENTAL_SENSING);
BLECharacteristic temperature_characteristic    = BLECharacteristic(UUID16_CHR_TEMPERATURE);
BLECharacteristic humidity_characteristic       = BLECharacteristic(UUID16_CHR_HUMIDITY);
BLECharacteristic pressure_characteristic       = BLECharacteristic(UUID16_CHR_PRESSURE);
BLECharacteristic tvoc_characteristic			= BLECharacteristic(UUID128_CHR_TVOC);

#endif