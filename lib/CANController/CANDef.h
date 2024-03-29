#ifndef CAN_DEF_H
#define CAN_DEF_H

#include <Arduino.h>

#define STM32_CAN_TIR_TXRQ (1U << 0U) // Bit 0: Transmit Mailbox Request
#define STM32_CAN_RIR_RTR (1U << 1U)  // Bit 1: Remote Transmission Request
#define STM32_CAN_RIR_IDE (1U << 2U)  // Bit 2: Identifier Extension
#define STM32_CAN_TIR_RTR (1U << 1U)  // Bit 1: Remote Transmission Request
#define STM32_CAN_TIR_IDE (1U << 2U)  // Bit 2: Identifier Extension

#define CAN_EXT_ID_MASK 0x1FFFFFFFU
#define CAN_STD_ID_MASK 0x000007FFU

typedef enum
{
    CAN_50KBPS,
    CAN_100KBPS,
    CAN_125KBPS,
    CAN_250KBPS,
    CAN_500KBPS,
    CAN_1000KBPS
} BITRATE;

/* Symbolic names for formats of CAN message                                 */
typedef enum
{
    STANDARD_FORMAT = 0,
    EXTENDED_FORMAT
} CAN_FORMAT;

/* Symbolic names for type of CAN message                                    */
typedef enum
{
    DATA_FRAME = 0,
    REMOTE_FRAME
} CAN_FRAME;

typedef struct
{
    uint32_t id;     /* 29 bit identifier                               */
    uint8_t data[8]; /* Data field                                      */
    uint8_t len;     /* Length of data field in bytes                   */
    uint8_t ch;      /* Object channel(Not use)                         */
    uint8_t format;  /* 0 - STANDARD, 1- EXTENDED IDENTIFIER            */
    uint8_t type;    /* 0 - DATA FRAME, 1 - REMOTE FRAME                */
} CAN_msg_t;

typedef const struct
{
    uint8_t TS2;
    uint8_t TS1;
    uint8_t BRP;
} CAN_bit_timing_config_t;

struct FilterConfig 
{
    enum IdeCheck {
        IDE_UNCHECKED = 0,
        IDE_CHECKED = 1
    };

    enum RtrCheck {
        RTR_UNCHECKED = 0,
        RTR_CHECKED = 1
    };

    enum IdeMode {
        STANDARD_ONLY = 0,
        EXTENDED_ONLY = 1
    };

    enum RtrMode {
        DATA_ONLY = 0,
        REMOTE_ONLY = 1
    };

    enum CanFormatAcceptanceMode {
        ANY_FORMAT = 0,
        STANDARD_FORMAT = 1,
        EXTENDED_FORMAT = 2,
    };

    enum CanFrameAcceptanceMode {
        ANY_FRAME = 0,
        DATA_FRAME = 1,
        REMOTE_FRAME = 2
    };

    struct IdConfig 
    {
        uint32_t id;
        CanFormatAcceptanceMode ideMode;
        CanFrameAcceptanceMode rtrMode;
    };
    IdConfig idConfig;

    struct MaskConfig
    {
        uint32_t mask;
        IdeCheck ideCheck = IDE_UNCHECKED;
        RtrCheck rtrCheck = RTR_UNCHECKED;
        IdeMode ideMode;
        RtrMode rtrMode;
    };
    MaskConfig maskConfig;
};

CAN_bit_timing_config_t can_configs[6] = {{2, 13, 45}, {2, 15, 20}, {2, 13, 18}, {2, 13, 9}, {2, 15, 4}, {2, 15, 2}};
#endif