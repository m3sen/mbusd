/*
 * OpenMODBUS/TCP to RS-232/485 MODBUS RTU gateway
 *
 * modbus.h - MODBUS protocol related procedures
 *
 * Copyright (c) 2002-2003, 2013, Victor Antonovich (v.antonovich@gmail.com)
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * - Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * - Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: modbus.h,v 1.3 2015/02/25 10:33:57 kapyar Exp $
 */

#ifndef _MODBUS_H
#define _MODBUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "globals.h"
#include "crc16.h"

/*! \brief Maximum number of Modbus functions codes the protocol stack
 *    should support.
 *
 * The maximum number of supported Modbus functions must be greater than
 * the sum of all enabled functions in this file and custom function
 * handlers. If set to small adding more functions will fail.
 */
#define MB_FUNC_HANDLERS_MAX (16)

#define MB_FUNC_OTHER_REP_SLAVEID_BUF (32)

/*! \brief If the <em>Read Holding Registers</em> function should be enabled. */
#define MB_FUNC_READ_HOLDING_ENABLED (1)

/*! \brief If the <em>Write Single Register</em> function should be enabled. */
#define MB_FUNC_WRITE_HOLDING_ENABLED (1)

/*! \brief If the <em>Write Single Mask(AND/OR) Register</em> function should be enabled. */
#define MB_FUNC_MASK_WRITE_HOLDING_ENABLED (1)

/*! \brief If the <em>Write Multiple registers</em> function should be enabled. */
#define MB_FUNC_WRITE_MULTIPLE_HOLDING_ENABLED (1)

/*! \brief If the <em>Read Coils</em> function should be enabled. */
#define MB_FUNC_READ_COILS_ENABLED (1)

/*! \brief If the <em>Write Coils</em> function should be enabled. */
#define MB_FUNC_WRITE_COIL_ENABLED (1)

/*! \brief If the <em>Write Multiple Coils</em> function should be enabled. */
#define MB_FUNC_WRITE_MULTIPLE_COILS_ENABLED (1)

/*! \brief If the <em>Read/Write Multiple Registers</em> function should be enabled. */
#define MB_FUNC_READWRITE_HOLDING_ENABLED (1)

/*!
 * Constants which defines the format of a modbus frame. The example is
 * shown for a Modbus RTU/ASCII frame. Note that the Modbus PDU is not
 * dependent on the underlying transport.
 *
 * <code>
 * <------------------------ MODBUS SERIAL LINE PDU (1) ------------------->
 *              <----------- MODBUS PDU (1') ---------------->
 *  +-----------+---------------+----------------------------+-------------+
 *  | Address   | Function Code | Data                       | CRC/LRC     |
 *  +-----------+---------------+----------------------------+-------------+
 *  |           |               |                                   |
 * (2)        (3/2')           (3')                                (4)
 *
 * (1)  ... MB_SER_PDU_SIZE_MAX = 256
 * (2)  ... MB_SER_PDU_ADDR_OFF = 0
 * (3)  ... MB_SER_PDU_PDU_OFF  = 1
 * (4)  ... MB_SER_PDU_SIZE_CRC = 2
 *
 * (1') ... MB_PDU_SIZE_MAX     = 253
 * (2') ... MB_PDU_FUNC_OFF     = 0
 * (3') ... MB_PDU_DATA_OFF     = 1
 * </code>
 */

/* ----------------------- Defines ------------------------------------------*/
#define MB_PDU_SIZE_MAX                        253 /*!< Maximum size of a PDU. */
#define MB_PDU_SIZE_MIN                        1   /*!< Function Code */
#define MB_SER_PDU_SIZE_MIN                    4   /*!< Minimum size of a Modbus RTU frame. */
#define MB_SER_PDU_SIZE_MAX                    256 /*!< Maximum size of a Modbus RTU frame. */
#define MB_SER_PDU_SIZE_CRC                    2   /*!< Size of CRC field in PDU. */
#define MB_SER_PDU_ADDR_OFF                    0   /*!< Offset of slave address in Ser-PDU. */
#define MB_SER_PDU_PDU_OFF                     1   /*!< Offset of Modbus-PDU in Ser-PDU. */
#define MB_PDU_FUNC_OFF                        0   /*!< Offset of function code in PDU. */
#define MB_PDU_DATA_OFF                        1   /*!< Offset for response data in PDU. */
#define MB_PDU_FUNC_READ_ADDR_OFF              (MB_PDU_DATA_OFF)
#define MB_PDU_FUNC_WRITE_ADDR_OFF             (MB_PDU_DATA_OFF)
#define MB_PDU_FUNC_WRITE_MUL_ADDR_OFF         (MB_PDU_DATA_OFF)
#define MB_PDU_FUNC_READ_COILCNT_OFF           (MB_PDU_DATA_OFF + 2)
#define MB_PDU_FUNC_READ_SIZE                  (4)
#define MB_PDU_FUNC_READ_COILCNT_MAX           (0x07D0)
#define MB_PDU_FUNC_WRITE_VALUE_OFF            (MB_PDU_DATA_OFF + 2)
#define MB_PDU_FUNC_WRITE_SIZE                 (4)
#define MB_PDU_FUNC_WRITE_MUL_COILCNT_OFF      (MB_PDU_DATA_OFF + 2)
#define MB_PDU_FUNC_WRITE_MUL_BYTECNT_OFF      (MB_PDU_DATA_OFF + 4)
#define MB_PDU_FUNC_WRITE_MUL_VALUES_OFF       (MB_PDU_DATA_OFF + 5)
#define MB_PDU_FUNC_WRITE_MUL_SIZE_MIN         (5)
#define MB_PDU_FUNC_WRITE_MUL_COILCNT_MAX      (0x07B0)
#define MB_PDU_FUNC_READ_REGCNT_OFF            (MB_PDU_DATA_OFF + 2)
#define MB_PDU_FUNC_READ_SIZE                  (4)
#define MB_PDU_FUNC_READ_REGCNT_MAX            (0x007D)
#define MB_PDU_FUNC_WRITE_VALUE_OFF            (MB_PDU_DATA_OFF + 2)
#define MB_PDU_FUNC_WRITE_SIZE                 (4)
#define MB_PDU_FUNC_MASK_WRITE_SIZE            (6)
#define MB_PDU_FUNC_WRITE_MUL_REGCNT_OFF       (MB_PDU_DATA_OFF + 2)
#define MB_PDU_FUNC_WRITE_MUL_BYTECNT_OFF      (MB_PDU_DATA_OFF + 4)
#define MB_PDU_FUNC_WRITE_MUL_VALUES_OFF       (MB_PDU_DATA_OFF + 5)
#define MB_PDU_FUNC_WRITE_MUL_SIZE_MIN         (5)
#define MB_PDU_FUNC_WRITE_MUL_REGCNT_MAX       (0x0078)
#define MB_PDU_FUNC_READWRITE_READ_ADDR_OFF    (MB_PDU_DATA_OFF + 0)
#define MB_PDU_FUNC_READWRITE_READ_REGCNT_OFF  (MB_PDU_DATA_OFF + 2)
#define MB_PDU_FUNC_READWRITE_WRITE_ADDR_OFF   (MB_PDU_DATA_OFF + 4)
#define MB_PDU_FUNC_READWRITE_WRITE_REGCNT_OFF (MB_PDU_DATA_OFF + 6)
#define MB_PDU_FUNC_READWRITE_BYTECNT_OFF      (MB_PDU_DATA_OFF + 8)
#define MB_PDU_FUNC_READWRITE_WRITE_VALUES_OFF (MB_PDU_DATA_OFF + 9)
#define MB_PDU_FUNC_READWRITE_SIZE_MIN         (9)
#define MB_FUNC_NONE                           (0)
#define MB_FUNC_READ_COILS                     (1)
#define MB_FUNC_READ_DISCRETE_INPUTS           (2)
#define MB_FUNC_WRITE_SINGLE_COIL              (5)
#define MB_FUNC_WRITE_MULTIPLE_COILS           (15)
#define MB_FUNC_READ_HOLDING_REGISTER          (3)
#define MB_FUNC_READ_INPUT_REGISTER            (4)
#define MB_FUNC_WRITE_REGISTER                 (6)
#define MB_FUNC_MASK_WRITE_REGISTER            (22)
#define MB_FUNC_WRITE_MULTIPLE_REGISTERS       (16)
#define MB_FUNC_READWRITE_MULTIPLE_REGISTERS   (23)
#define MB_FUNC_DIAG_READ_EXCEPTION            (7)
#define MB_FUNC_DIAG_DIAGNOSTIC                (8)
#define MB_FUNC_DIAG_GET_COM_EVENT_CNT         (11)
#define MB_FUNC_DIAG_GET_COM_EVENT_LOG         (12)
#define MB_FUNC_OTHER_REPORT_SLAVEID           (17)
#define MB_FUNC_ERROR                          (128)
#define REG_COIL_START                         (0x0000)
#define REG_COIL_NREGS                         (0x0040) // Assign 64 bit for RPI Mimic modbus
#define REG_COIL_BYTES                         (REG_COIL_NREGS + 4) / 8
#define REG_COIL_READONLY_END_ADDRESS          10 // 0 ~ 9 Read only
#define REG_HOLDING_START                      (0x0000)
#define REG_HOLDING_NREGS                      (0x20) //32
#define REG_R_0x0000_RTU_TYPE                  0x0000
#define REG_R_0x0001_SW_VERSION                0x0001
#define REG_R_0x0002_MODBUS_ID                 0x0002
#define REG_R_0x0003_BAUDRATE                  0x0003
#define REG_R_0x0004_TEMP                      0x0004
#define REG_R_0x0005_HUM                       0x0005
#define REG_R_0x0006_RS485_VO1                 0x0006
#define REG_R_0x0007_LAMP_VO2                  0x0007
#define REG_RW_0x0008_RES                      0x0008
#define REG_RW_0x0009_MOTOR_PWM                0x0009
#define REG_RW_0x000A_MOTOR_INTERVAL           0x000A
#define REG_RW_0x000B_MOTOR_ON_TIME            0x000B
#define REG_RW_0x000C_R_BLINK_INTERVAL         0x000C
#define REG_RW_0x000D_Y_BLINK_INTERVAL         0x000D
#define REG_RW_0x000E_G_BLINK_INTERVAL         0x000E
#define REG_RW_0x000F_iLL_DIM                  0x000F
#define REG_RW_0x0010_LED_DIM                  0x0010
#define REG_R_0x0011_RS485_VO1_OFFSET          0x0011
#define REG_R_0x0012_LAMP_VO2_OFFSET           0x0012
#define REG_RW_0x0013_TONE_TYPE                0x0013
#define REG_RW_0x0014_RES                      0x0014
#define REG_RW_0x0015_RES                      0x0015
#define REG_RW_0x0016_RES                      0x0016
#define REG_RW_0x0017_RES                      0x0017
#define REG_RW_0x0018_RES                      0x0018
#define REG_RW_0x0019_RES                      0x0019
#define REG_RW_0x001A_RES                      0x001A
#define REG_RW_0x001B_RES                      0x001B
#define REG_RW_0x001C_RES                      0x001C
#define REG_RW_0x001D_RES                      0x001D
#define REG_RW_0x001E_RES                      0x001E
#define REG_RW_0x001F_RES                      0x001F
#define MB_ADDRESS_BROADCAST                   (0) /*! Modbus broadcast address. */
#define RPI_MODBUS_ID                          (0x01)
#define MB_ADDRESS_MIN                         (1)   /*! Smallest possible slave address. */
#define MB_ADDRESS_MAX                         (247) /*! Biggest possible slave address. */

/* ----------------------- Type definitions ---------------------------------*/
typedef enum {
    MB_REG_READ,      /*!< Read register values and pass to protocol stack. */
    MB_REG_WRITE,     /*!< Update register values. */
    MB_REG_MASK_WRITE /*!< Update masking register values. */
} eMBRegisterMode;

typedef enum {
    MB_ENOERR,    /*!< no error. */
    MB_ENOREG,    /*!< illegal register address. */
    MB_EINVAL,    /*!< illegal argument. */
    MB_EPORTERR,  /*!< porting layer error. */
    MB_ENORES,    /*!< insufficient resources. */
    MB_EIO,       /*!< I/O error. */
    MB_EILLSTATE, /*!< protocol stack in illegal state. */
    MB_ETIMEDOUT  /*!< timeout error occurred. */
} eMBErrorCode;

typedef enum {
    MB_EX_NONE                 = 0x00,
    MB_EX_ILLEGAL_FUNCTION     = 0x01,
    MB_EX_ILLEGAL_DATA_ADDRESS = 0x02,
    MB_EX_ILLEGAL_DATA_VALUE   = 0x03,
    MB_EX_SLAVE_DEVICE_FAILURE = 0x04,
    MB_EX_ACKNOWLEDGE          = 0x05,
    MB_EX_SLAVE_BUSY           = 0x06,
    MB_EX_MEMORY_PARITY_ERROR  = 0x08,
    MB_EX_GATEWAY_PATH_FAILED  = 0x0A,
    MB_EX_GATEWAY_TGT_FAILED   = 0x0B
} eMBException;

typedef eMBException (*pxMBFunctionHandler)(uint8_t *pucFrame, uint16_t *pusLength);

typedef struct
{
    uint8_t             ui8FC;
    pxMBFunctionHandler pxHandler;
} xMBFunctionHandler;

typedef union structRTUCoils {
    struct {
        // 0 ~ 15	Control and Status
        // Bit Index	Name	Descryption
        uint8_t b0_READY : 1;              /*!< Ready, default:0 */
        uint8_t b1_BTN1 : 1;               /*!< btn1, default:1 */
        uint8_t b2_BTN2 : 1;               /*!< btn2, default:1 */
        uint8_t b3_BTN3 : 1;               /*!< btn5, default:1 */
        uint8_t b4_BTN4 : 1;               /*!< btn6, default:1 */
        uint8_t b5_BTN5 : 1;               /*!< btn4, default:1 */
        uint8_t b6_BTN6 : 1;               /*!< btn3, default:1 */
        uint8_t b7_BTN7 : 1;               /*!< btn7, default:1 */
        uint8_t b8_BTN8 : 1;               /*!< btn8, default:1 */
        uint8_t b9_SHT3x : 1;              /*!< TBD */
        uint8_t b10_RES : 1;               /*!< TBD */
        uint8_t b11_RES : 1;               /*!< TBD */
        uint8_t b12_RES : 1;               /*!< TBD */
        uint8_t b13_RES : 1;               /*!< TBD */
        uint8_t b14_RES : 1;               /*!< TBD */
        uint8_t b15_RES : 1;               /*!< TBD */
        uint8_t b16_R_ON : 1;              /*!< RED LED ON/OFF*/
        uint8_t b17_Y_ON : 1;              /*!< YELLOW LED ON/OFF */
        uint8_t b18_G_ON : 1;              /*!< GREEN LED ON/OFF */
        uint8_t b19_R_BLINKING : 1;        /*!< RED LED BLINKING */
        uint8_t b20_Y_BLINKING : 1;        /*!< YELLOW LED BLINKING */
        uint8_t b21_G_BLINKING : 1;        /*!< GREEN LED BLINKING */
        uint8_t b22_BZ_ON : 1;             /*!< TBD */
        uint8_t b23_iLL_ON : 1;            /*!< Illuminator ON/OFF */
        uint8_t b24_RES : 1;               /*!< Illuminator DIMMING */
        uint8_t b25_MOTOR : 1;             /*!< TBD */
        uint8_t b26_RES : 1;               /*!< TBD */
        uint8_t b27_RES : 1;               /*!< TBD */
        uint8_t b28_RES : 1;               /*!< TBD */
        uint8_t b29_RES : 1;               /*!< TBD */
        uint8_t b30_SIB_ON : 1;            /*!< TBD */
        uint8_t b31_ILL_OVERCURRENT : 1;   /*!< TBD */
        uint8_t b32_GlobalMotorResync : 1; /*!< TBD */
        uint8_t b33_GlobalMotorStop : 1;   /*!< TBD */
        uint8_t b34_RESET_ALARM : 1;       /*!< TBD */
        uint8_t b35_POWER_OFF : 1;         /*!< TBD */
        uint8_t b36_PBL5 : 1;              /*!< TBD */
        uint8_t b37_PBL6 : 1;              /*!< TBD */
        uint8_t b38_PBL7 : 1;              /*!< TBD */
        uint8_t b39_JBB_RESET : 1;         /*!< Special Function to IAP */
        uint8_t b40_RES : 1;               /*!< RPI MIMIC modbus only */
        uint8_t b41_RES : 1;               /*!< RPI MIMIC modbus only */
        uint8_t b42_RES : 1;               /*!< RPI MIMIC modbus only */
        uint8_t b43_RES : 1;               /*!< RPI MIMIC modbus only */
        uint8_t b44_RES : 1;               /*!< RPI MIMIC modbus only */
        uint8_t b45_RES : 1;               /*!< RPI MIMIC modbus only */
        uint8_t b46_RES : 1;               /*!< RPI MIMIC modbus only */
        uint8_t b47_RES : 1;               /*!< RPI MIMIC modbus only */
        uint8_t b48_RES : 1;               /*!< RPI MIMIC modbus only */
        uint8_t b49_RES : 1;               /*!< RPI MIMIC modbus only */
        uint8_t b50_JBB1_ON : 1;           /*!< SIB Group 1 Power on/off RPI MIMIC modbus only */
        uint8_t b51_JBB2_ON : 1;           /*!< SIB Group 2 Power on/off RPI MIMIC modbus only */
        uint8_t b52_JBB3_ON : 1;           /*!< SIB Group 3 Power on/off RPI MIMIC modbus only */
        uint8_t b53_JBB4_ON : 1;           /*!< SIB Group 4 Power on/off RPI MIMIC modbus only */
        uint8_t b54_JBB5_ON : 1;           /*!< SIB Group 5 Power on/off RPI MIMIC modbus only */
        uint8_t b55_JBB6_ON : 1;           /*!< SIB Group 6 Power on/off RPI MIMIC modbus only */
        uint8_t b56_JBB7_ON : 1;           /*!< SIB Group 7 Power on/off RPI MIMIC modbus only */
        uint8_t b57_JBB8_ON : 1;           /*!< SIB Group 8 Power on/off RPI MIMIC modbus only */
        uint8_t b58_JBB9_ON : 1;           /*!< SIB Group 9 Power on/off RPI MIMIC modbus only */
        uint8_t b59_JBB10_ON : 1;          /*!< SIB Group 10 Power on/off RPI MIMIC modbus only */
        uint8_t b60_RES : 1;               /*!< RPI MIMIC modbus only */
        uint8_t b61_RES : 1;               /*!< RPI MIMIC modbus only */
        uint8_t b62_RES : 1;               /*!< RPI MIMIC modbus only */
        uint8_t b63_RES : 1;               /*!< RPI MIMIC modbus only */
    } bits;
    uint8_t ui8Byte[8];
} structRTUCoils_t;

extern uint16_t         gui16HoldingReg[REG_HOLDING_NREGS];
extern structRTUCoils_t gstCoils;
extern structRTUCoils_t gstPrevCoils;
extern uint8_t *        gpui8CoilBuf;
extern uint8_t          gui8Response[MB_SER_PDU_SIZE_MAX];

uint32_t     MimicModBus(uint8_t *buf, uint32_t ui32Length);
eMBErrorCode HoldingCallBack(uint8_t *pucRegBuffer, uint16_t usAddress, uint16_t usNRegs, eMBRegisterMode eMode);
eMBErrorCode CoilCallBack(uint8_t *pucRegBuffer, uint16_t usAddress, uint16_t usNCoils, eMBRegisterMode eMode);
void TimerHandler(int signum);
//////////////////////////////////////////////////////////////////////////////////
// Added by kmshim
//////////////////////////////////////////////////////////////////////////////////

/*
 * Macro for accessing data in MODBUS frame
 */
#define MB_FRAME(p, d) (*(p + d))

/*
 * MODBUS frame lengths
 */
#define MB_EX_LEN  3
#define MB_MIN_LEN 4
#define MB_ERR_LEN 5
#define MB_MAX_LEN 256

/*
 * Macros for word operations
 */
#define HIGH(w) ((unsigned char)(((w) >> 8) & 0xff))
#define LOW(w)  ((unsigned char)((w)&0xff))
#define WORD_WR_BE(p, w)                               \
    do {                                               \
        *(p)     = (unsigned char)(((w) >> 8) & 0xff); \
        *(p + 1) = (unsigned char)((w)&0xff);          \
    } while (0);
#define WORD_RD_BE(p) ((((unsigned short)*(p) << 8) & 0xff00) + \
                       (*(p + 1) & 0xff))
#define WORD_WR_LE(p, w)                               \
    do {                                               \
        *(p)     = (unsigned char)((w)&0xff);          \
        *(p + 1) = (unsigned char)(((w) >> 8) & 0xff); \
    } while (0);
#define WORD_RD_LE(p) ((((unsigned short)*(p + 1) << 8) & 0xff00) + \
                       (*(p)&0xff))

/*
 * MODBUS packet structure
 */
#define MB_TRANS_ID_H  0       /* transaction ID high byte */
#define MB_TRANS_ID_L  1       /* transaction ID low byte */
#define MB_PROTO_ID_H  2       /* protocol ID high byte */
#define MB_PROTO_ID_L  3       /* protocol ID low byte */
#define MB_LENGTH_H    4       /* length field high byte */
#define MB_LENGTH_L    5       /* length field low byte */
#define MB_UNIT_ID     6       /* unit identifier */
#define MB_FCODE       7       /* function code */
#define MB_DATA        8       /* MODBUS data */
#define MB_DATA_ADDR_H MB_DATA /* MODBUS data: address high byte */
#define MB_DATA_ADDR_L 9       /* MODBUS data: address low byte */
#define MB_DATA_NVAL_H 10      /* MODBUS data: number of values high byte */
#define MB_DATA_NVAL_L 11      /* MODBUS data: number of values low byte */
#define MB_DATA_NBYTES 12      /* MODBUS data: number of bytes to follow (fc 15,16) */

#define TTY_FCODE_IDX 1    /* The index of the function code in a RTU packet */
#define TTY_ERR_MASK  0x80 /* The MSB of the response function code indicates an error response */

/*
 * Exception codes
 */
#define MB_EX_CRC     4
#define MB_EX_TIMEOUT 0x0B

/* Prototypes */
int  modbus_crc_correct(unsigned char *frame, unsigned int len);
void modbus_crc_write(unsigned char *frame, unsigned int len);
void modbus_ex_write(unsigned char *packet, unsigned char code);
int  modbus_check_header(unsigned char *packet);

#ifdef __cplusplus
}
#endif

#endif /* _MODBUS_H */
