/*
 * OpenMODBUS/TCP to RS-232/485 MODBUS RTU gateway
 *
 * modbus.c - MODBUS protocol related procedures
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
 * $Id: modbus.c,v 1.3 2015/02/25 10:33:58 kapyar Exp $
 */

#include "modbus.h"
#include "conn.h"
#include "m3sen_bcm2838_gpio.h"

/*
 * Check CRC of MODBUS frame
 * Parameters: FRAME - address of the frame,
 *             LEN   - frame length;
 * Return: 0 if CRC failed,
 *         non-zero otherwise
 */
int modbus_crc_correct(unsigned char *frame, unsigned int len) {
    return (!crc16(frame, len));
}

/*
 * Write CRC to MODBUS frame
 * Parameters: FRAME - address of the frame,
 *             LEN   - frame length;
 * Return: none
 */
void modbus_crc_write(unsigned char *frame, unsigned int len) {
    WORD_WR_LE(frame + len, crc16(frame, len));
}

/*
 * Write exception response to OpenMODBUS request
 * Parameters: PACKET - address of the request packet,
 *             CODE - exception code;
 * Return: none
 */
void modbus_ex_write(unsigned char *packet, unsigned char code) {
    MB_FRAME(packet, MB_FCODE) |= 0x80;
    MB_FRAME(packet, MB_DATA) = code;
    WORD_WR_BE(packet + MB_LENGTH_H, MB_EX_LEN);
}

/*
 * Check MODBUS packet header consistency
 * Parameters: PACKET - address of the request packet,
 * Return: RC_OK if (mostly) all is right, RC_ERR otherwise
 */
int modbus_check_header(unsigned char *packet) {
    return (MB_FRAME(packet, MB_PROTO_ID_H) == 0 &&
            MB_FRAME(packet, MB_PROTO_ID_L) == 0 &&
            MB_FRAME(packet, MB_LENGTH_H) == 0 &&
            MB_FRAME(packet, MB_LENGTH_L) > 0 &&
            MB_FRAME(packet, MB_LENGTH_L) < BUFSIZE - CRCSIZE)
               ? RC_OK
               : RC_ERR;
}

//////////////////////////////////////////////////////////////////////////////
// Add by kmshim, 2021/06/03
//////////////////////////////////////////////////////////////////////////////
#define FIRMWARE_VERSION 0x0002 // REG_R_0x0001_SW_VERSION 0.1
#define ILL_DEFAULT_PWM  30     //65 @ 100KHz, 55@10Khz(440mA @ 2.9V)
#define BITS_UCHAR       8U

structRTUCoils_t gstCoils = {
    {
        .b0_READY              = 0, /*!< Ready to scan/move/clean default:0 */
        .b1_BTN1               = 1, /*!< btn1, default:1 */
        .b2_BTN2               = 0, /*!< btn2, default:1 */
        .b3_BTN3               = 0, /*!< btn5, default:0 */
        .b4_BTN4               = 0, /*!< btn6, default:0 */
        .b5_BTN5               = 0, /*!< btn4, default:0 */
        .b6_BTN6               = 0, /*!< btn3, default:0 */
        .b7_BTN7               = 0, /*!< btn7, default:0 */
        .b8_BTN8               = 0, /*!< btn8, default:0 */
        .b9_SHT3x              = 0, /*!< TBD */
        .b10_RES               = 0, /*!< TBD */
        .b11_RES               = 0, /*!< TBD */
        .b12_RES               = 0, /*!< TBD */
        .b13_RES               = 0, /*!< TBD */
        .b14_RES               = 0, /*!< TBD */
        .b15_RES               = 0, /*!< TBD */
        .b16_R_ON              = 0, /*!< RED LED ON/OFF*/
        .b17_Y_ON              = 0, /*!< YELLOW LED ON/OFF */
        .b18_G_ON              = 0, /*!< GREEN LED ON/OFF */
        .b19_R_BLINKING        = 0, /*!< RED LED BLINKING */
        .b20_Y_BLINKING        = 0, /*!< YELLOW LED BLINKING */
        .b21_G_BLINKING        = 0, /*!< GREEN LED BLINKING */
        .b22_BZ_ON             = 0, /*!< Buzzer on /off */
        .b23_iLL_ON            = 1, /*!< Illuminator ON/OFF */
        .b24_RES               = 1, /*!< Illuminator DIMMING */
        .b25_MOTOR             = 1, /*!< TBD */
        .b26_RES               = 0, /*!< TBD */
        .b27_RES               = 0, /*!< TBD */
        .b28_RES               = 0, /*!< TBD */
        .b29_RES               = 0, /*!< TBD */
        .b30_SIB_ON            = 1, /*!< TBD */
        .b31_ILL_OVERCURRENT   = 0, /*!< TBD */
        .b32_GlobalMotorResync = 0, /*!< TBD */
        .b33_GlobalMotorStop   = 0, /*!< TBD */
        .b34_RESET_ALARM       = 0, /*!< TBD */
        .b35_POWER_OFF         = 0, /*!< TBD */
        .b36_PBL5              = 0, /*!< TBD */
        .b37_PBL6              = 0, /*!< TBD */
        .b38_PBL7              = 0, /*!< TBD */
        .b39_JBB_RESET         = 0, /*!< Special Function to IAP */
        .b40_RES               = 0, /*!< RPI MIMIC modbus only */
        .b41_RES               = 0, /*!< RPI MIMIC modbus only */
        .b42_RES               = 0, /*!< RPI MIMIC modbus only */
        .b43_RES               = 0, /*!< RPI MIMIC modbus only */
        .b44_RES               = 0, /*!< RPI MIMIC modbus only */
        .b45_RES               = 0, /*!< RPI MIMIC modbus only */
        .b46_RES               = 0, /*!< RPI MIMIC modbus only */
        .b47_RES               = 0, /*!< RPI MIMIC modbus only */
        .b48_RES               = 0, /*!< RPI MIMIC modbus only */
        .b49_RES               = 0, /*!< RPI MIMIC modbus only */
        .b50_JBB1_ON           = 0, /*!< SIB Group 1 Power on/off RPI MIMIC modbus only */
        .b51_JBB2_ON           = 0, /*!< SIB Group 2 Power on/off RPI MIMIC modbus only */
        .b52_JBB3_ON           = 0, /*!< SIB Group 3 Power on/off RPI MIMIC modbus only */
        .b53_JBB4_ON           = 0, /*!< SIB Group 4 Power on/off RPI MIMIC modbus only */
        .b54_JBB5_ON           = 0, /*!< SIB Group 5 Power on/off RPI MIMIC modbus only */
        .b55_JBB6_ON           = 0, /*!< SIB Group 6 Power on/off RPI MIMIC modbus only */
        .b56_JBB7_ON           = 0, /*!< SIB Group 7 Power on/off RPI MIMIC modbus only */
        .b57_JBB8_ON           = 0, /*!< SIB Group 8 Power on/off RPI MIMIC modbus only */
        .b58_JBB9_ON           = 0, /*!< SIB Group 9 Power on/off RPI MIMIC modbus only */
        .b59_JBB10_ON          = 0, /*!< SIB Group 10 Power on/off RPI MIMIC modbus only */
        .b60_RES               = 0, /*!< RPI MIMIC modbus only */
        .b61_RES               = 0, /*!< RPI MIMIC modbus only */
        .b62_RES               = 0, /*!< RPI MIMIC modbus only */
        .b63_RES               = 0, /*!< RPI MIMIC modbus only */
    }};

structRTUCoils_t gstPrevCoils = {
    {
        .b0_READY              = 0, /*!< Ready to scan/move/clean default:0 */
        .b1_BTN1               = 0, /*!< btn1, default:1 */
        .b2_BTN2               = 0, /*!< btn2, default:1 */
        .b3_BTN3               = 0, /*!< btn5, default:0 */
        .b4_BTN4               = 0, /*!< btn6, default:0 */
        .b5_BTN5               = 0, /*!< btn4, default:0 */
        .b6_BTN6               = 0, /*!< btn3, default:0 */
        .b7_BTN7               = 0, /*!< btn7, default:0 */
        .b8_BTN8               = 0, /*!< btn8, default:0 */
        .b9_SHT3x              = 0, /*!< TBD */
        .b10_RES               = 0, /*!< TBD */
        .b11_RES               = 0, /*!< TBD */
        .b12_RES               = 0, /*!< TBD */
        .b13_RES               = 0, /*!< TBD */
        .b14_RES               = 0, /*!< TBD */
        .b15_RES               = 0, /*!< TBD */
        .b16_R_ON              = 0, /*!< RED LED ON/OFF*/
        .b17_Y_ON              = 0, /*!< YELLOW LED ON/OFF */
        .b18_G_ON              = 0, /*!< GREEN LED ON/OFF */
        .b19_R_BLINKING        = 0, /*!< RED LED BLINKING */
        .b20_Y_BLINKING        = 0, /*!< YELLOW LED BLINKING */
        .b21_G_BLINKING        = 0, /*!< GREEN LED BLINKING */
        .b22_BZ_ON             = 0, /*!< TBD */
        .b23_iLL_ON            = 0, /*!< Illuminator ON/OFF */
        .b24_RES               = 0, /*!< Illuminator DIMMING */
        .b25_MOTOR             = 0, /*!< TBD */
        .b26_RES               = 0, /*!< TBD */
        .b27_RES               = 0, /*!< TBD */
        .b28_RES               = 0, /*!< TBD */
        .b29_RES               = 0, /*!< TBD */
        .b30_SIB_ON            = 0, /*!< TBD */
        .b31_ILL_OVERCURRENT   = 0, /*!< TBD */
        .b32_GlobalMotorResync = 0, /*!< TBD */
        .b33_GlobalMotorStop   = 0, /*!< TBD */
        .b34_RESET_ALARM       = 0, /*!< TBD */
        .b35_POWER_OFF         = 0, /*!< TBD */
        .b36_PBL5              = 0, /*!< TBD */
        .b37_PBL6              = 0, /*!< TBD */
        .b38_PBL7              = 0, /*!< TBD */
        .b39_JBB_RESET         = 0, /*!< Special Function to IAP */
        .b40_RES               = 0, /*!< RPI MIMIC modbus only */
        .b41_RES               = 0, /*!< RPI MIMIC modbus only */
        .b42_RES               = 0, /*!< RPI MIMIC modbus only */
        .b43_RES               = 0, /*!< RPI MIMIC modbus only */
        .b44_RES               = 0, /*!< RPI MIMIC modbus only */
        .b45_RES               = 0, /*!< RPI MIMIC modbus only */
        .b46_RES               = 0, /*!< RPI MIMIC modbus only */
        .b47_RES               = 0, /*!< RPI MIMIC modbus only */
        .b48_RES               = 0, /*!< RPI MIMIC modbus only */
        .b49_RES               = 0, /*!< RPI MIMIC modbus only */
        .b50_JBB1_ON           = 0, /*!< SIB Group 1 Power on/off RPI MIMIC modbus only */
        .b51_JBB2_ON           = 0, /*!< SIB Group 2 Power on/off RPI MIMIC modbus only */
        .b52_JBB3_ON           = 0, /*!< SIB Group 3 Power on/off RPI MIMIC modbus only */
        .b53_JBB4_ON           = 0, /*!< SIB Group 4 Power on/off RPI MIMIC modbus only */
        .b54_JBB5_ON           = 0, /*!< SIB Group 5 Power on/off RPI MIMIC modbus only */
        .b55_JBB6_ON           = 0, /*!< SIB Group 6 Power on/off RPI MIMIC modbus only */
        .b56_JBB7_ON           = 0, /*!< SIB Group 7 Power on/off RPI MIMIC modbus only */
        .b57_JBB8_ON           = 0, /*!< SIB Group 8 Power on/off RPI MIMIC modbus only */
        .b58_JBB9_ON           = 0, /*!< SIB Group 9 Power on/off RPI MIMIC modbus only */
        .b59_JBB10_ON          = 0, /*!< SIB Group 10 Power on/off RPI MIMIC modbus only */
        .b60_RES               = 0, /*!< RPI MIMIC modbus only */
        .b61_RES               = 0, /*!< RPI MIMIC modbus only */
        .b62_RES               = 0, /*!< RPI MIMIC modbus only */
        .b63_RES               = 0, /*!< RPI MIMIC modbus only */
    }};

uint16_t gui16HoldingReg[REG_HOLDING_NREGS] =
    {
        'P' << 8 | 'I',   // REG_R_0x0000_RTU_TYPE
        FIRMWARE_VERSION, // REG_R_0x0001_SW_VERSION
        0x0001,           // REG_R_0x0002_MODBUS_ID
        0x0004,           // REG_R_0x0003_BAUDRATE
        0x0000,           // REG_R_0x0004_TEMP
        0x0000,           // REG_R_0x0005_HUM
        0,                // REG_R_0x0006_RS485_VO1
        0,                // REG_R_0x0007_LAMP_VO2
        0,                // REG_RW_0x0008_RES
        0,                // REG_RW_0x0009_MOTOR_PWM, range: 0 ~ 10, unit: x10 [%]
        0,                // REG_RW_0x000A_MOTOR_INTERVAL, unit: sec
        0,                // REG_RW_0x000B_MOTOR_ON_TIME
        0,                // REG_RW_0x000C_R_BLINK_INTERVAL
        0,                // REG_RW_0x000D_Y_BLINK_INTERVAL
        0,                // REG_RW_0x000E_G_BLINK_INTERVAL
        0,                // REG_RW_0x000F_iLL_DIM, ragne: 0 ~ 100, unit: %
        0,                // REG_RW_0x0010_LED_DIM, range: 0 ~ 100, unit: %
        0,                // REG_R_0x0011_RS485_VO1_OFFSET
        0,                // REG_R_0x0012_LAMP_VO2_OFFSET
        0,                // REG_RW_0x0013_TONE_TYPE
        0,                // REG_RW_0x0014_RES
        0,                // REG_RW_0x0015_RES
        0,                // REG_RW_0x0016_RES
        0,                // REG_RW_0x0017_RES
        0,                // REG_RW_0x0018_RES
        0,                // REG_RW_0x0019_RES
        0,                // REG_RW_0x001A_RES
        0,                // REG_RW_0x001B_RES
        0,                // REG_RW_0x001C_RES
        0,                // REG_RW_0x001D_RES
        0,                // REG_RW_0x001E_RES
        0,                // REG_RW_0x001F_RES
};

/* ----------------------- Static variables ---------------------------------*/
uint8_t *                 gpui8CoilBuf = (uint8_t *)&gstCoils.bits;
eMBException              prveMBError2Exception(eMBErrorCode eErrorCode);
static xMBFunctionHandler xFuncHandlers[MB_FUNC_HANDLERS_MAX];

/* ----------------------- Start implementation -----------------------------*/
void xMBUtilSetBits(uint8_t *ucByteBuf, uint16_t usBitOffset, uint8_t ucNBits, uint8_t ucValue) {
    uint16_t usWordBuf;
    uint16_t usMask;
    uint16_t usByteOffset;
    uint16_t usNPreBits;
    uint16_t usValue = ucValue;

    usByteOffset = (uint16_t)((usBitOffset) / BITS_UCHAR);
    usNPreBits   = (uint16_t)(usBitOffset - usByteOffset * BITS_UCHAR);
    usValue <<= usNPreBits;
    usMask = (uint16_t)((1 << (uint16_t)ucNBits) - 1);
    usMask <<= usBitOffset - usByteOffset * BITS_UCHAR;
    usWordBuf = ucByteBuf[usByteOffset];
    usWordBuf |= ucByteBuf[usByteOffset + 1] << BITS_UCHAR;
    usWordBuf                   = (uint16_t)((usWordBuf & (~usMask)) | usValue);
    ucByteBuf[usByteOffset]     = (uint8_t)(usWordBuf & 0xFF);
    ucByteBuf[usByteOffset + 1] = (uint8_t)(usWordBuf >> BITS_UCHAR);
}

uint8_t xMBUtilGetBits(uint8_t *ucByteBuf, uint16_t usBitOffset, uint8_t ucNBits) {
    uint16_t usWordBuf;
    uint16_t usMask;
    uint16_t usByteOffset;
    uint16_t usNPreBits;

    usByteOffset = (uint16_t)((usBitOffset) / BITS_UCHAR);
    usNPreBits   = (uint16_t)(usBitOffset - usByteOffset * BITS_UCHAR);
    usMask       = (uint16_t)((1 << (uint16_t)ucNBits) - 1);
    usWordBuf    = ucByteBuf[usByteOffset];
    usWordBuf |= ucByteBuf[usByteOffset + 1] << BITS_UCHAR;
    usWordBuf >>= usNPreBits;
    usWordBuf &= usMask;

    return (uint8_t)usWordBuf;
}

eMBException prveMBError2Exception(eMBErrorCode eErrorCode) {
    eMBException eStatus;

    switch (eErrorCode) {
    case MB_ENOERR:
        eStatus = MB_EX_NONE;
        break;

    case MB_ENOREG:
        eStatus = MB_EX_ILLEGAL_DATA_ADDRESS;
        break;

    case MB_ETIMEDOUT:
        eStatus = MB_EX_SLAVE_BUSY;
        break;

    default:
        eStatus = MB_EX_SLAVE_DEVICE_FAILURE;
        break;
    }

    return eStatus;
}

eMBErrorCode HoldingCallBack(uint8_t *pucRegBuffer, uint16_t usAddress, uint16_t usNRegs, eMBRegisterMode eMode) {
    eMBErrorCode eStatus = MB_ENOERR;
    int          iRegIndex;
    if ((usAddress + usNRegs) <= (REG_HOLDING_START + REG_HOLDING_NREGS)) {
        iRegIndex = (int)(usAddress - REG_HOLDING_START);
        switch (eMode) {
        case MB_REG_READ:
            while (usNRegs > 0) {
                *pucRegBuffer++ = (uint8_t)(gui16HoldingReg[iRegIndex] >> 8);
                *pucRegBuffer++ = (uint8_t)(gui16HoldingReg[iRegIndex] & 0xFF);
                iRegIndex++;
                usNRegs--;
            }
            break;
        case MB_REG_WRITE:
            while (usNRegs > 0) {
                gui16HoldingReg[iRegIndex] = *pucRegBuffer++ << 8;
                gui16HoldingReg[iRegIndex] |= *pucRegBuffer++;
                switch (iRegIndex) {
                case REG_R_0x0002_MODBUS_ID:
                    //SaveRTUID((uint8_t)gui16HoldingReg[REG_R_0x0002_MODBUS_ID]);
                    break;
                case REG_RW_0x0010_LED_DIM:
                    gui16HoldingReg[REG_RW_0x0010_LED_DIM] = gui16HoldingReg[REG_RW_0x0010_LED_DIM] > 100 ? 100 : gui16HoldingReg[REG_RW_0x0010_LED_DIM];
                    break;
                case REG_RW_0x0009_MOTOR_PWM:
                    gui16HoldingReg[REG_RW_0x0009_MOTOR_PWM] = gui16HoldingReg[REG_RW_0x0009_MOTOR_PWM] > 20 ? 20 : gui16HoldingReg[REG_RW_0x0009_MOTOR_PWM];
                    break;
                case REG_RW_0x000F_iLL_DIM:
                    gui16HoldingReg[REG_RW_0x000F_iLL_DIM] = gui16HoldingReg[REG_RW_0x000F_iLL_DIM] > 100 ? 100 : gui16HoldingReg[REG_RW_0x000F_iLL_DIM];
                    break;
                default:
                    break;
                }
                iRegIndex++;
                usNRegs--;
            }

            break;
        case MB_REG_MASK_WRITE: {
            uint16_t ui16ANDMask = pucRegBuffer[0] << 8 | pucRegBuffer[1];
            uint16_t ui16ORMask  = pucRegBuffer[2] << 8 | pucRegBuffer[3];

            ui16ORMask &= ~ui16ANDMask;
            gui16HoldingReg[iRegIndex] &= ui16ANDMask;
            gui16HoldingReg[iRegIndex] |= ui16ORMask;
        } break;
        }
    } else {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}

eMBErrorCode CoilCallBack(uint8_t *pucRegBuffer, uint16_t usAddress, uint16_t usNCoils, eMBRegisterMode eMode) {
    eMBErrorCode eStatus = MB_ENOERR;
    int          iRegIndex;

    if ((usAddress + usNCoils) <= (REG_COIL_START + REG_COIL_NREGS)) {
        iRegIndex = (int)(usAddress - REG_COIL_START);
        switch (eMode) {
        case MB_REG_READ:
            while (usNCoils > 0) {
                uint8_t ucResult = xMBUtilGetBits(gpui8CoilBuf, iRegIndex, 1);
                xMBUtilSetBits(pucRegBuffer, iRegIndex - (usAddress - REG_COIL_START), 1, ucResult);
                iRegIndex++;
                usNCoils--;
            }
            break;
        case MB_REG_WRITE:
            while (usNCoils > 0) {
                uint8_t ucResult = xMBUtilGetBits(pucRegBuffer, iRegIndex - (usAddress - REG_COIL_START), 1);
                xMBUtilSetBits(gpui8CoilBuf, iRegIndex, 1, ucResult);
                iRegIndex++;
                usNCoils--;
            }
            break;
        case MB_REG_MASK_WRITE:
            break;
        }
    } else {
        eStatus = MB_ENOREG;
    }

    return eStatus;
}

#if MB_FUNC_READ_COILS_ENABLED > 0

eMBException ReadCoils(uint8_t *pucFrame, uint16_t *usLen) {
    uint16_t     usRegAddress;
    uint16_t     usCoilCount;
    uint8_t      ucNBytes;
    uint8_t *    pucFrameCur;
    eMBException eStatus = MB_EX_NONE;
    eMBErrorCode eRegStatus;

#ifdef DEBUG
    logw(7, "TLM: ReadCoils");
#endif

    if (*usLen == (MB_PDU_FUNC_READ_SIZE + MB_PDU_SIZE_MIN)) {
        usRegAddress = (uint16_t)(pucFrame[MB_PDU_FUNC_READ_ADDR_OFF] << 8);
        usRegAddress |= (uint16_t)(pucFrame[MB_PDU_FUNC_READ_ADDR_OFF + 1]);
        usCoilCount = (uint16_t)(pucFrame[MB_PDU_FUNC_READ_COILCNT_OFF] << 8);
        usCoilCount |= (uint16_t)(pucFrame[MB_PDU_FUNC_READ_COILCNT_OFF + 1]);

        if ((usCoilCount >= 1) && (usCoilCount < MB_PDU_FUNC_READ_COILCNT_MAX)) {
            pucFrameCur = &pucFrame[MB_PDU_FUNC_OFF];
            *usLen      = MB_PDU_FUNC_OFF;

            *pucFrameCur++ = MB_FUNC_READ_COILS;
            *usLen += 1;

            if ((usCoilCount & 0x0007) != 0) {
                ucNBytes = (uint8_t)(usCoilCount / 8 + 1);
            } else {
                ucNBytes = (uint8_t)(usCoilCount / 8);
            }
            *pucFrameCur++ = ucNBytes;
            *usLen += 1;

            eRegStatus = CoilCallBack(pucFrameCur, usRegAddress, usCoilCount, MB_REG_READ);

            if (eRegStatus != MB_ENOERR) {
                eStatus = prveMBError2Exception(eRegStatus);
            } else {
                *usLen += ucNBytes;
            }
        } else {
            eStatus = MB_EX_ILLEGAL_DATA_VALUE;
        }
    } else {
        eStatus = MB_EX_ILLEGAL_DATA_VALUE;
    }
    return eStatus;
}

#if MB_FUNC_WRITE_COIL_ENABLED > 0
eMBException WriteCoil(uint8_t *pucFrame, uint16_t *usLen) {
    uint16_t     usRegAddress;
    uint8_t      ucBuf[2];
    eMBException eStatus = MB_EX_NONE;
    eMBErrorCode eRegStatus;
#ifdef DEBUG
    logw(7, "TLM: WriteCoil");
#endif

    if (*usLen == (MB_PDU_FUNC_WRITE_SIZE + MB_PDU_SIZE_MIN)) {
        usRegAddress = (uint16_t)(pucFrame[MB_PDU_FUNC_WRITE_ADDR_OFF] << 8);
        usRegAddress |= (uint16_t)(pucFrame[MB_PDU_FUNC_WRITE_ADDR_OFF + 1]);

        if (usRegAddress < REG_COIL_READONLY_END_ADDRESS) {
            eStatus = MB_EX_ILLEGAL_DATA_ADDRESS;
        } else {
            if ((pucFrame[MB_PDU_FUNC_WRITE_VALUE_OFF + 1] == 0x00) && ((pucFrame[MB_PDU_FUNC_WRITE_VALUE_OFF] == 0xFF) || (pucFrame[MB_PDU_FUNC_WRITE_VALUE_OFF] == 0x00))) {
                ucBuf[1]   = 0;
                ucBuf[0]   = (pucFrame[MB_PDU_FUNC_WRITE_VALUE_OFF] == 0xFF) ? 1 : 0;
                eRegStatus = CoilCallBack(&ucBuf[0], usRegAddress, 1, MB_REG_WRITE);
                if (eRegStatus != MB_ENOERR) {
                    eStatus = prveMBError2Exception(eRegStatus);
                }
            } else {
                eStatus = MB_EX_ILLEGAL_DATA_VALUE;
            }
        }
    } else {
        eStatus = MB_EX_ILLEGAL_DATA_VALUE;
    }
    return eStatus;
}

#endif

#if MB_FUNC_WRITE_MULTIPLE_COILS_ENABLED > 0

eMBException WriteMultipleCoils(uint8_t *pucFrame, uint16_t *usLen) {
    uint16_t     usRegAddress;
    uint16_t     usCoilCnt;
    uint8_t      ucByteCount;
    uint8_t      ucByteCountVerify;
    eMBException eStatus = MB_EX_NONE;
    eMBErrorCode eRegStatus;

#ifdef DEBUG
    logw(7, "TLM: WriteMultipleCoils");
#endif

    if (*usLen > (MB_PDU_FUNC_WRITE_SIZE + MB_PDU_SIZE_MIN)) {
        usRegAddress = (uint16_t)(pucFrame[MB_PDU_FUNC_WRITE_MUL_ADDR_OFF] << 8);
        usRegAddress |= (uint16_t)(pucFrame[MB_PDU_FUNC_WRITE_MUL_ADDR_OFF + 1]);

        usCoilCnt = (uint16_t)(pucFrame[MB_PDU_FUNC_WRITE_MUL_COILCNT_OFF] << 8);
        usCoilCnt |= (uint16_t)(pucFrame[MB_PDU_FUNC_WRITE_MUL_COILCNT_OFF + 1]);

        ucByteCount = pucFrame[MB_PDU_FUNC_WRITE_MUL_BYTECNT_OFF];
        if (usRegAddress < REG_COIL_READONLY_END_ADDRESS) {
            eStatus = MB_EX_ILLEGAL_DATA_ADDRESS;
        } else {
            if ((usCoilCnt & 0x0007) != 0) {
                ucByteCountVerify = (uint8_t)(usCoilCnt / 8 + 1);
            } else {
                ucByteCountVerify = (uint8_t)(usCoilCnt / 8);
            }
            if ((usCoilCnt >= 1) && (usCoilCnt <= MB_PDU_FUNC_WRITE_MUL_COILCNT_MAX) && (ucByteCountVerify == ucByteCount)) {
                eRegStatus = CoilCallBack(&pucFrame[MB_PDU_FUNC_WRITE_MUL_VALUES_OFF], usRegAddress, usCoilCnt, MB_REG_WRITE);
                if (eRegStatus != MB_ENOERR) {
                    eStatus = prveMBError2Exception(eRegStatus);
                } else {
                    *usLen = MB_PDU_FUNC_WRITE_MUL_BYTECNT_OFF;
                }
            } else {
                eStatus = MB_EX_ILLEGAL_DATA_VALUE;
            }
        }
    } else {
        eStatus = MB_EX_ILLEGAL_DATA_VALUE;
    }
    return eStatus;
}

#endif

#endif

#if MB_FUNC_MASK_WRITE_HOLDING_ENABLED > 0

eMBException WriteMaskHoldingRegister(uint8_t *pucFrame, uint16_t *usLen) {
    uint16_t     usRegAddress;
    eMBException eStatus = MB_EX_NONE;
    eMBErrorCode eRegStatus;

#ifdef DEBUG
    logw(7, "TLM: WriteMaskHoldingRegister");
#endif

    if (*usLen == (MB_PDU_FUNC_MASK_WRITE_SIZE + MB_PDU_SIZE_MIN)) {
        usRegAddress = (uint16_t)(pucFrame[MB_PDU_FUNC_WRITE_ADDR_OFF] << 8);
        usRegAddress |= (uint16_t)(pucFrame[MB_PDU_FUNC_WRITE_ADDR_OFF + 1]);
        eRegStatus = HoldingCallBack(&pucFrame[MB_PDU_FUNC_WRITE_VALUE_OFF], usRegAddress, 1, MB_REG_MASK_WRITE);
        if (eRegStatus != MB_ENOERR) {
            eStatus = prveMBError2Exception(eRegStatus);
        }
    } else {
        eStatus = MB_EX_ILLEGAL_DATA_VALUE;
    }
    return eStatus;
}
#endif

#if MB_FUNC_WRITE_HOLDING_ENABLED > 0

eMBException WriteHoldingRegister(uint8_t *pucFrame, uint16_t *usLen) {
    uint16_t     usRegAddress;
    eMBException eStatus = MB_EX_NONE;
    eMBErrorCode eRegStatus;

#ifdef DEBUG
    logw(7, "TLM: WriteHoldingRegister");
#endif

    if (*usLen == (MB_PDU_FUNC_WRITE_SIZE + MB_PDU_SIZE_MIN)) {
        usRegAddress = (uint16_t)(pucFrame[MB_PDU_FUNC_WRITE_ADDR_OFF] << 8);
        usRegAddress |= (uint16_t)(pucFrame[MB_PDU_FUNC_WRITE_ADDR_OFF + 1]);

        switch (usRegAddress) {
        default:
            eRegStatus = HoldingCallBack(&pucFrame[MB_PDU_FUNC_WRITE_VALUE_OFF], usRegAddress, 1, MB_REG_WRITE);
            if (eRegStatus != MB_ENOERR) {
                eStatus = prveMBError2Exception(eRegStatus);
            }
            break;
        case REG_R_0x0000_RTU_TYPE:
        case REG_R_0x0001_SW_VERSION:
        case REG_R_0x0002_MODBUS_ID:
        case REG_R_0x0003_BAUDRATE:
        case REG_R_0x0004_TEMP:
        case REG_R_0x0005_HUM:
            eStatus = MB_EX_ILLEGAL_DATA_ADDRESS;
            break;
        case REG_RW_0x001F_RES:
            break;
        }
    } else {
        eStatus = MB_EX_ILLEGAL_DATA_VALUE;
    }
    return eStatus;
}
#endif

#if MB_FUNC_WRITE_MULTIPLE_HOLDING_ENABLED > 0
eMBException WriteMultipleHoldingRegister(uint8_t *pucFrame, uint16_t *usLen) {
    uint16_t     usRegAddress;
    uint16_t     usRegCount;
    uint8_t      ucRegByteCount;
    eMBException eStatus = MB_EX_NONE;
    eMBErrorCode eRegStatus;

#ifdef DEBUG
    logw(7, "TLM: WriteMultipleHoldingRegister");
#endif

    if (*usLen >= (MB_PDU_FUNC_WRITE_MUL_SIZE_MIN + MB_PDU_SIZE_MIN)) {
        usRegAddress = (uint16_t)(pucFrame[MB_PDU_FUNC_WRITE_MUL_ADDR_OFF] << 8);
        usRegAddress |= (uint16_t)(pucFrame[MB_PDU_FUNC_WRITE_MUL_ADDR_OFF + 1]);

        usRegCount = (uint16_t)(pucFrame[MB_PDU_FUNC_WRITE_MUL_REGCNT_OFF] << 8);
        usRegCount |= (uint16_t)(pucFrame[MB_PDU_FUNC_WRITE_MUL_REGCNT_OFF + 1]);

        ucRegByteCount = pucFrame[MB_PDU_FUNC_WRITE_MUL_BYTECNT_OFF];

        if ((usRegCount >= 1) &&
            (usRegCount <= MB_PDU_FUNC_WRITE_MUL_REGCNT_MAX) &&
            (ucRegByteCount == (uint8_t)(2 * usRegCount))) {
            eRegStatus = HoldingCallBack(&pucFrame[MB_PDU_FUNC_WRITE_MUL_VALUES_OFF], usRegAddress, usRegCount, MB_REG_WRITE);
            if (eRegStatus != MB_ENOERR) {
                eStatus = prveMBError2Exception(eRegStatus);
            } else {
                *usLen = MB_PDU_FUNC_WRITE_MUL_BYTECNT_OFF;
            }
        } else {
            eStatus = MB_EX_ILLEGAL_DATA_VALUE;
        }
    } else {
        eStatus = MB_EX_ILLEGAL_DATA_VALUE;
    }
    return eStatus;
}
#endif

#if MB_FUNC_READ_HOLDING_ENABLED > 0
eMBException ReadHoldingRegister(uint8_t *pucFrame, uint16_t *usLen) {
    uint16_t     usRegAddress;
    uint16_t     usRegCount;
    uint8_t *    pucFrameCur;
    eMBException eStatus = MB_EX_NONE;
    eMBErrorCode eRegStatus;

#ifdef DEBUG
    logw(7, "TLM: ReadHoldingRegister");
#endif

    if (*usLen == (MB_PDU_FUNC_READ_SIZE + MB_PDU_SIZE_MIN)) {
        usRegAddress = (uint16_t)(pucFrame[MB_PDU_FUNC_READ_ADDR_OFF] << 8);
        usRegAddress |= (uint16_t)(pucFrame[MB_PDU_FUNC_READ_ADDR_OFF + 1]);
        usRegCount = (uint16_t)(pucFrame[MB_PDU_FUNC_READ_REGCNT_OFF] << 8);
        usRegCount = (uint16_t)(pucFrame[MB_PDU_FUNC_READ_REGCNT_OFF + 1]);
        if ((usRegCount >= 1) && (usRegCount <= MB_PDU_FUNC_READ_REGCNT_MAX)) {
            pucFrameCur    = &pucFrame[MB_PDU_FUNC_OFF];
            *usLen         = MB_PDU_FUNC_OFF;
            *pucFrameCur++ = MB_FUNC_READ_HOLDING_REGISTER;
            *usLen += 1;
            *pucFrameCur++ = (uint8_t)(usRegCount * 2);
            *usLen += 1;
            eRegStatus = HoldingCallBack(pucFrameCur, usRegAddress, usRegCount, MB_REG_READ);
            if (eRegStatus != MB_ENOERR) {
                eStatus = prveMBError2Exception(eRegStatus);
            } else {
                *usLen += usRegCount * 2;
            }
        } else {
            eStatus = MB_EX_ILLEGAL_DATA_VALUE;
        }
    } else {
        eStatus = MB_EX_ILLEGAL_DATA_VALUE;
    }
    return eStatus;
}
#endif

#if MB_FUNC_READWRITE_HOLDING_ENABLED > 0
eMBException ReadWriteMultipleHoldingRegister(uint8_t *pucFrame, uint16_t *usLen) {
    uint16_t     usRegReadAddress;
    uint16_t     usRegReadCount;
    uint16_t     usRegWriteAddress;
    uint16_t     usRegWriteCount;
    uint8_t      ucRegWriteByteCount;
    uint8_t *    pucFrameCur;
    eMBException eStatus = MB_EX_NONE;
    eMBErrorCode eRegStatus;

#ifdef DEBUG
    logw(7, "TLM: ReadWriteMultipleHoldingRegister");
#endif

    if (*usLen >= (MB_PDU_FUNC_READWRITE_SIZE_MIN + MB_PDU_SIZE_MIN)) {
        usRegReadAddress = (uint16_t)(pucFrame[MB_PDU_FUNC_READWRITE_READ_ADDR_OFF] << 8U);
        usRegReadAddress |= (uint16_t)(pucFrame[MB_PDU_FUNC_READWRITE_READ_ADDR_OFF + 1]);

        usRegReadCount = (uint16_t)(pucFrame[MB_PDU_FUNC_READWRITE_READ_REGCNT_OFF] << 8U);
        usRegReadCount |= (uint16_t)(pucFrame[MB_PDU_FUNC_READWRITE_READ_REGCNT_OFF + 1]);

        usRegWriteAddress = (uint16_t)(pucFrame[MB_PDU_FUNC_READWRITE_WRITE_ADDR_OFF] << 8U);
        usRegWriteAddress |= (uint16_t)(pucFrame[MB_PDU_FUNC_READWRITE_WRITE_ADDR_OFF + 1]);

        usRegWriteCount = (uint16_t)(pucFrame[MB_PDU_FUNC_READWRITE_WRITE_REGCNT_OFF] << 8U);
        usRegWriteCount |= (uint16_t)(pucFrame[MB_PDU_FUNC_READWRITE_WRITE_REGCNT_OFF + 1]);

        ucRegWriteByteCount = pucFrame[MB_PDU_FUNC_READWRITE_BYTECNT_OFF];

        switch (usRegWriteAddress) {
        default:
            if ((usRegReadCount >= 1) &&
                (usRegReadCount <= 0x7D) &&
                (usRegWriteCount >= 1) &&
                (usRegWriteCount <= 0x79) &&
                ((2 * usRegWriteCount) == ucRegWriteByteCount)) {
                /* Make callback to update the register values. */
                eRegStatus = HoldingCallBack(&pucFrame[MB_PDU_FUNC_READWRITE_WRITE_VALUES_OFF], usRegWriteAddress, usRegWriteCount, MB_REG_WRITE);

                if (eRegStatus == MB_ENOERR) {
                    /* Set the current PDU data pointer to the beginning. */
                    pucFrameCur = &pucFrame[MB_PDU_FUNC_OFF];
                    *usLen      = MB_PDU_FUNC_OFF;

                    /* First byte contains the function code. */
                    *pucFrameCur++ = MB_FUNC_READWRITE_MULTIPLE_REGISTERS;
                    *usLen += 1;

                    /* Second byte in the response contain the number of bytes. */
                    *pucFrameCur++ = (uint8_t)(usRegReadCount * 2);
                    *usLen += 1;

                    /* Make the read callback. */
                    eRegStatus =
                        HoldingCallBack(pucFrameCur, usRegReadAddress, usRegReadCount, MB_REG_READ);
                    if (eRegStatus == MB_ENOERR) {
                        *usLen += 2 * usRegReadCount;
                    }
                }
                if (eRegStatus != MB_ENOERR) {
                    eStatus = prveMBError2Exception(eRegStatus);
                }
            } else {
                eStatus = MB_EX_ILLEGAL_DATA_VALUE;
            }
            break;
        case REG_R_0x0000_RTU_TYPE:
        case REG_R_0x0001_SW_VERSION:
        case REG_R_0x0002_MODBUS_ID:
        case REG_R_0x0003_BAUDRATE:
        case REG_R_0x0004_TEMP:
        case REG_R_0x0005_HUM:
            eStatus = MB_EX_ILLEGAL_DATA_ADDRESS;
            break;
        }
    }
    return eStatus;
}
#endif
uint32_t HAL_GetTick(void) {
    uint32_t       tval = 0;
    struct timeval ts, tts, t_out;
    (void)gettimeofday(&tts, NULL);
    tval = 1000ul * (tts.tv_sec - ts.tv_sec) + (tts.tv_usec - ts.tv_usec) / 1000;
}

void GreenHandler(void) {
    static uint32_t ui32PrevTickStart = 0;
    uint32_t        ui32Tickstart     = HAL_GetTick();
#ifdef DEBUG
    //logw(2, "TLM: GetTick %ums -> %ums", ui32PrevTickStart, ui32Tickstart);
#endif

    if (gstCoils.bits.b21_G_BLINKING != gstPrevCoils.bits.b21_G_BLINKING) {
        gstPrevCoils.bits.b21_G_BLINKING = gstCoils.bits.b21_G_BLINKING;
        if (gstCoils.bits.b21_G_BLINKING) {
            Toogle(GREEN);
            ui32PrevTickStart = ui32Tickstart;
        } else {
            WritePin(GREEN, gstCoils.bits.b18_G_ON);
        }
    }
    if (gstCoils.bits.b21_G_BLINKING) {
#ifdef DEBUG
        // logw(2, "TLM: TickDiff %ums", ui32Tickstart - ui32PrevTickStart);
#endif

        if ((ui32Tickstart - ui32PrevTickStart) > gui16HoldingReg[REG_RW_0x000E_G_BLINK_INTERVAL]) {
            Toogle(GREEN);
            ui32PrevTickStart = ui32Tickstart;
        }
    } else {
        //GREEN On/Off
        if (gstCoils.bits.b18_G_ON != gstPrevCoils.bits.b18_G_ON) {
            gstPrevCoils.bits.b18_G_ON = gstCoils.bits.b18_G_ON;
            WritePin(GREEN, gstCoils.bits.b18_G_ON);
        }
    }
}

void YellowHandler(void) {
    static uint32_t ui32PrevTickStart = 0;
    uint32_t        ui32Tickstart     = HAL_GetTick();
    if (gstCoils.bits.b20_Y_BLINKING != gstPrevCoils.bits.b20_Y_BLINKING) {
        gstPrevCoils.bits.b20_Y_BLINKING = gstCoils.bits.b20_Y_BLINKING;
        if (gstCoils.bits.b20_Y_BLINKING) {
            Toogle(YELLOW);
            ui32PrevTickStart = ui32Tickstart;
        } else {
            WritePin(YELLOW, gstCoils.bits.b17_Y_ON);
        }
    }
    if (gstCoils.bits.b20_Y_BLINKING) {
        if ((ui32Tickstart - ui32PrevTickStart) > gui16HoldingReg[REG_RW_0x000D_Y_BLINK_INTERVAL]) {
            Toogle(YELLOW);
            ui32PrevTickStart = ui32Tickstart;
        }
    } else {
        //RED On/Off
        if (gstCoils.bits.b17_Y_ON != gstPrevCoils.bits.b17_Y_ON) {
            gstPrevCoils.bits.b17_Y_ON = gstCoils.bits.b17_Y_ON;
            WritePin(YELLOW, gstCoils.bits.b17_Y_ON);
        }
    }
}

void RedHandler(void) {
    static uint32_t ui32PrevTickStart = 0;
    uint32_t        ui32Tickstart     = HAL_GetTick();
    if (gstCoils.bits.b19_R_BLINKING != gstPrevCoils.bits.b19_R_BLINKING) {
        gstPrevCoils.bits.b19_R_BLINKING = gstCoils.bits.b19_R_BLINKING;
        if (gstCoils.bits.b19_R_BLINKING) {
            Toogle(RED);
            ui32PrevTickStart = ui32Tickstart;
        } else {
            WritePin(RED, gstCoils.bits.b16_R_ON);
        }
    }
    if (gstCoils.bits.b19_R_BLINKING) {
        if ((ui32Tickstart - ui32PrevTickStart) > gui16HoldingReg[REG_RW_0x000C_R_BLINK_INTERVAL]) {
            Toogle(RED);
            ui32PrevTickStart = ui32Tickstart;
        }
    } else {
        //RED On/Off
        if (gstCoils.bits.b16_R_ON != gstPrevCoils.bits.b16_R_ON) {
            gstPrevCoils.bits.b16_R_ON = gstCoils.bits.b16_R_ON;
            WritePin(RED, gstCoils.bits.b16_R_ON);
        }
    }
}

void LEDHandler(void) {
    //determine RED blinking start and end
    gstCoils.bits.b0_READY = false;
    RedHandler();
    YellowHandler();
    GreenHandler();
    gstCoils.bits.b0_READY = true;
}

void MelodyHandler(void) {
    // TBD
}

void JBBPowerHandler(void) {
    if (gstPrevCoils.bits.b50_JBB1_ON != gstCoils.bits.b50_JBB1_ON) {
        WritePin(JBB1, gstCoils.bits.b50_JBB1_ON);
        gstPrevCoils.bits.b50_JBB1_ON = gstCoils.bits.b50_JBB1_ON;
    }
    if (gstPrevCoils.bits.b51_JBB2_ON != gstCoils.bits.b51_JBB2_ON) {
        WritePin(JBB2, gstCoils.bits.b51_JBB2_ON);
        gstPrevCoils.bits.b51_JBB2_ON = gstCoils.bits.b51_JBB2_ON;
    }
    if (gstPrevCoils.bits.b52_JBB3_ON != gstCoils.bits.b52_JBB3_ON) {
        WritePin(JBB3, gstCoils.bits.b52_JBB3_ON);
        gstPrevCoils.bits.b52_JBB3_ON = gstCoils.bits.b52_JBB3_ON;
    }
    if (gstPrevCoils.bits.b53_JBB4_ON != gstCoils.bits.b53_JBB4_ON) {
        WritePin(JBB4, gstCoils.bits.b53_JBB4_ON);
        gstPrevCoils.bits.b53_JBB4_ON = gstCoils.bits.b53_JBB4_ON;
    }
    if (gstPrevCoils.bits.b54_JBB5_ON != gstCoils.bits.b54_JBB5_ON) {
        WritePin(JBB5, gstCoils.bits.b54_JBB5_ON);
        gstPrevCoils.bits.b54_JBB5_ON = gstCoils.bits.b54_JBB5_ON;
    }
    if (gstPrevCoils.bits.b55_JBB6_ON != gstCoils.bits.b55_JBB6_ON) {
        WritePin(JBB6, gstCoils.bits.b55_JBB6_ON);
        gstPrevCoils.bits.b55_JBB6_ON = gstCoils.bits.b55_JBB6_ON;
    }
    if (gstPrevCoils.bits.b56_JBB7_ON != gstCoils.bits.b56_JBB7_ON) {
        WritePin(JBB7, gstCoils.bits.b56_JBB7_ON);
        gstPrevCoils.bits.b56_JBB7_ON = gstCoils.bits.b56_JBB7_ON;
    }
    if (gstPrevCoils.bits.b57_JBB8_ON != gstCoils.bits.b57_JBB8_ON) {
        WritePin(JBB8, gstCoils.bits.b57_JBB8_ON);
        gstPrevCoils.bits.b57_JBB8_ON = gstCoils.bits.b57_JBB8_ON;
    }
    if (gstPrevCoils.bits.b58_JBB9_ON != gstCoils.bits.b58_JBB9_ON) {
        WritePin(JBB9, gstCoils.bits.b58_JBB9_ON);
        gstPrevCoils.bits.b58_JBB9_ON = gstCoils.bits.b58_JBB9_ON;
    }
    if (gstPrevCoils.bits.b59_JBB10_ON != gstCoils.bits.b59_JBB10_ON) {
        WritePin(JBB10, gstCoils.bits.b59_JBB10_ON);
        gstPrevCoils.bits.b59_JBB10_ON = gstCoils.bits.b59_JBB10_ON;
    }
}

void TimerHandler(int signum) {
    static uint8_t ui8cnt = 0;
#ifdef DEBUG
    //logw(2, "TLM: 1sec Timer handler %d", signum);
#endif
    //////////////////////////////////////////////////////////
    // Exectute
    //////////////////////////////////////////////////////////
    JBBPowerHandler();
    LEDHandler();
    MelodyHandler();
    //////////////////////////////////////////////////////////
    // Exectute
    //////////////////////////////////////////////////////////
}

uint8_t gui8Response[MB_SER_PDU_SIZE_MAX];

uint32_t MimicModBus(uint8_t *buf, uint32_t ui32ReceiveTotal) {
    uint8_t      ui8FC;
    uint16_t     ui16Length;
    eMBException eException;
    ui8FC            = buf[MB_SER_PDU_PDU_OFF];
    eException       = MB_EX_ILLEGAL_FUNCTION;
    uint16_t ui16CRC = crc16((uint8_t *)buf, ui32ReceiveTotal);

#ifdef DEBUG
    logw(7, "TLM: MimicModBus");
#endif

    if ((ui32ReceiveTotal >= MB_SER_PDU_SIZE_MIN) && (ui16CRC == 0)) {
        memcpy(gui8Response, buf, ui32ReceiveTotal);
        ui16Length = (uint16_t)(ui32ReceiveTotal - MB_SER_PDU_PDU_OFF - MB_SER_PDU_SIZE_CRC);

        for (uint8_t i = 0; i < MB_FUNC_HANDLERS_MAX; i++) {
            if (xFuncHandlers[i].ui8FC == 0) {
                break;
            } else if (xFuncHandlers[i].ui8FC == ui8FC) {
                eException = xFuncHandlers[i].pxHandler(&gui8Response[1], &ui16Length);
                break;
            }
        }

        if (buf[MB_SER_PDU_ADDR_OFF] != MB_ADDRESS_BROADCAST) {
            if (eException != MB_EX_NONE) {
                ui16Length                 = 0;
                gui8Response[ui16Length++] = (uint8_t)(ui8FC | MB_FUNC_ERROR);
                gui8Response[ui16Length++] = eException;
            } else {
                ui16Length++;
                ui16CRC                    = crc16((uint8_t *)gui8Response, ui16Length);
                gui8Response[ui16Length++] = (uint8_t)(ui16CRC & 0xFF);
                gui8Response[ui16Length++] = (uint8_t)(ui16CRC >> 8);
                ui16Length++;
            }
        }
        return ui32ReceiveTotal;
    }
    return 0;
}

static xMBFunctionHandler xFuncHandlers[MB_FUNC_HANDLERS_MAX] =
    {
#if MB_FUNC_READ_HOLDING_ENABLED > 0
        {MB_FUNC_READ_HOLDING_REGISTER, ReadHoldingRegister},
#endif
#if MB_FUNC_WRITE_MULTIPLE_HOLDING_ENABLED > 0
        {MB_FUNC_WRITE_MULTIPLE_REGISTERS, WriteMultipleHoldingRegister},
#endif
#if MB_FUNC_WRITE_HOLDING_ENABLED > 0
        {MB_FUNC_WRITE_REGISTER, WriteHoldingRegister},
#endif
#if MB_FUNC_MASK_WRITE_HOLDING_ENABLED > 0
        {MB_FUNC_MASK_WRITE_REGISTER, WriteMaskHoldingRegister},
#endif
#if MB_FUNC_READWRITE_HOLDING_ENABLED > 0
        {MB_FUNC_READWRITE_MULTIPLE_REGISTERS, ReadWriteMultipleHoldingRegister},
#endif
#if MB_FUNC_READ_COILS_ENABLED > 0
        {MB_FUNC_READ_COILS, ReadCoils},
#endif
#if MB_FUNC_WRITE_COIL_ENABLED > 0
        {MB_FUNC_WRITE_SINGLE_COIL, WriteCoil},
#endif
#if MB_FUNC_WRITE_MULTIPLE_COILS_ENABLED > 0
        {MB_FUNC_WRITE_MULTIPLE_COILS, WriteMultipleCoils},
#endif
};
