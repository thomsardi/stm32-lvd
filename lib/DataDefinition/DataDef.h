#ifndef DATADEF_H
#define DATADEF_H

#include <Arduino.h>

struct BatteryData {
    
};

struct AdditionalCANData {
    union {
        struct {
            uint8_t BIT_0: 1;           /*This is the raw bit for line0 status.*/
            uint8_t BIT_1: 1;           /*This is the raw bit for line1 status.*/
            uint8_t BIT_2: 1;           /*This is the raw bit for line2 status.*/
            uint8_t reserved: 5;
        };
        int8_t val;
    } relayState; 
    float current[3];
};

enum DataType {
    PACK_DATA = 0x764C860,
    MOSF_TEMP = 0x763C860,
    VCELL_1_4 = 0x762C860,
    VCELL_5_8 = 0x761C860,
    VCELL_9_12 = 0x760C860,
    VCELL_13_15 = 0x75FC860,
    MAX_MIN_PARAM = 0x75EC860
};

#endif