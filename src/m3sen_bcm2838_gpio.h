
#ifndef M3SEN_BCM2838_H
#define M3SEN_BCM2838_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
#define __I volatile /*!< Defines 'read only' permissions                 */
#else
#define __I volatile const /*!< Defines 'read only' permissions                 */
#endif
#define __O  volatile /*!< Defines 'write only' permissions                */
#define __IO volatile /*!< Defines 'read / write' permissions              */

#ifndef __IM /*!< Fallback for older CMSIS versions               */
#define __IM __I
#endif
#ifndef __OM /*!< Fallback for older CMSIS versions               */
#define __OM __O
#endif
#ifndef __IOM /*!< Fallback for older CMSIS versions               */
#define __IOM __IO
#endif

#define INPUT  0x00 /*!< 000 = GPIO Pin 9 is an input                */
#define OUTPUT 0x01 /*!< 001 = GPIO Pin 9 is an output               */
#define ALT0   0x04 /*!< 100 = GPIO Pin 9 takes alternate function 0 */
#define ALT1   0x05 /*!< 101 = GPIO Pin 9 takes alternate function 1 */
#define ALT2   0x06 /*!< 110 = GPIO Pin 9 takes alternate function 2 */
#define ALT3   0x07 /*!< 111 = GPIO Pin 9 takes alternate function 3 */
#define ALT4   0x03 /*!< 011 = GPIO Pin 9 takes alternate function 4 */
#define ALT5   0x02 /*!< 010 = GPIO Pin 9 takes alternate function 5 */

#define GPIO0  0  /*!< FSEL0  */
#define GPIO1  1  /*!< FSEL1  */
#define GPIO2  2  /*!< FSEL2  */
#define GPIO3  3  /*!< FSEL3  */
#define GPIO4  4  /*!< FSEL4  */
#define GPIO5  5  /*!< FSEL5  */
#define GPIO6  6  /*!< FSEL6  */
#define GPIO7  7  /*!< FSEL7  */
#define GPIO8  8  /*!< FSEL8  */
#define GPIO9  9  /*!< FSEL9  */
#define GPIO10 10 /*!< FSEL10 */
#define GPIO11 11 /*!< FSEL11 */
#define GPIO12 12 /*!< FSEL12 */
#define GPIO13 13 /*!< FSEL13 */
#define GPIO14 14 /*!< FSEL14 */
#define GPIO15 15 /*!< FSEL15 */
#define GPIO16 16 /*!< FSEL16 */
#define GPIO17 17 /*!< FSEL17 */
#define GPIO18 18 /*!< FSEL18 */
#define GPIO19 19 /*!< FSEL19 */
#define GPIO20 20 /*!< FSEL20 */
#define GPIO21 21 /*!< FSEL21 */
#define GPIO22 22 /*!< FSEL22 */
#define GPIO23 23 /*!< FSEL23 */
#define GPIO24 24 /*!< FSEL24 */
#define GPIO25 25 /*!< FSEL25 */
#define GPIO26 26 /*!< FSEL26 */
#define GPIO27 27 /*!< FSEL27 */
#define GPIO28 28 /*!< FSEL28 */
#define GPIO29 29 /*!< FSEL29 */
#define GPIO30 30 /*!< FSEL30 */
#define GPIO31 31 /*!< FSEL31 */
#define GPIO32 32 /*!< FSEL32 */
#define GPIO33 33 /*!< FSEL33 */
#define GPIO34 34 /*!< FSEL34 */
#define GPIO35 35 /*!< FSEL35 */
#define GPIO36 36 /*!< FSEL36 */
#define GPIO37 37 /*!< FSEL37 */
#define GPIO38 38 /*!< FSEL38 */
#define GPIO39 39 /*!< FSEL39 */
#define GPIO40 40 /*!< FSEL40 */
#define GPIO41 41 /*!< FSEL41 */
#define GPIO42 42 /*!< FSEL42 */
#define GPIO43 43 /*!< FSEL43 */
#define GPIO44 44 /*!< FSEL44 */
#define GPIO45 45 /*!< FSEL45 */
#define GPIO46 46 /*!< FSEL46 */
#define GPIO47 47 /*!< FSEL47 */
#define GPIO48 48 /*!< FSEL48 */
#define GPIO49 49 /*!< FSEL49 */
#define GPIO50 50 /*!< FSEL50 */
#define GPIO51 51 /*!< FSEL51 */
#define GPIO52 52 /*!< FSEL52 */
#define GPIO53 53 /*!< FSEL53 */
#define GPIO54 54 /*!< FSEL54 */
#define GPIO55 55 /*!< FSEL55 */
#define GPIO56 56 /*!< FSEL56 */
#define GPIO57 57 /*!< FSEL57 */

#define JBB1      19 //: GPIO19
#define JBB2      26 //: GPIO26
#define JBB3      20 //: GPIO20
#define JBB4      16 //: GPIO16
#define JBB5      12 //: GPIO12
#define JBB6      7  //: GPIO7
#define JBB7      8  //: GPIO8
#define JBB8      25 //: GPIO25
#define JBB9      24 //: GPIO24
#define JBB10     23 //: GPIO23
#define RED       4  //: GPIO4
#define YELLOW    17 //: GPIO17
#define GREEN     27 //: GPIO27
#define BTN1      13 //: GPIO13
#define BTN2      6  //: GPIO6
#define BTN3      5  //: GPIO5
#define BTN4      0  //: GPIO0
#define BTN_LAMP1 13 //: GPIO11
#define BTN_LAMP2 6  //: GPIO9
#define BTN_LAMP3 5  //: GPIO10
#define BTN_LAMP4 0  //: GPIO22

#define GPIO_BASE 0xFE200000 // 물리적인 주소

typedef struct
{
    __IO uint32_t GPFSEL[6];                 /*!< 0x00 ~ 0x14 GPFSEL0,5 GPIO Function Select 0                                      */
    __IO uint32_t RESERVED1;                 /*!< 0x18                                                                              */
    __IO uint32_t GPSET[2];                  /*!< 0x1c ~ 0x20 GPSET0,1 GPIO Pin Output Set0                                         */
    __IO uint32_t RESERVED2;                 /*!< 0x24                                                                              */
    __IO uint32_t GPCLR[2];                  /*!< 0x28 ~ 0x2C GPCLR0,1 GPIO Pin Output Clear 0                                      */
    __IO uint32_t RESERVED3;                 /*!< 0x30                                                                              */
    __IO uint32_t GPLEV[2];                  /*!< 0x34 ~ 0x38 GPLEV0,1 GPIO Pin Level 0                                             */
    __IO uint32_t RESERVED4;                 /*!< 0x3C                                                                              */
    __IO uint32_t GPEDS[2];                  /*!< 0x40 ~ 0x44 GPEDS0,1 GPIO Pin Event Detect Status 0                               */
    __IO uint32_t RESERVED5;                 /*!< 0x48                                                                              */
    __IO uint32_t GPREN[2];                  /*!< 0x4c ~ 0x50 GPREN0,1 GPIO Pin Rising Edge Detect Enable 0                         */
    __IO uint32_t RESERVED6;                 /*!< 0x54                                                                              */
    __IO uint32_t GPFEN[2];                  /*!< 0x58 ~ 0x5C GPFEN0,1 GPIO Pin Falling Edge Detect Enable 0                        */
    __IO uint32_t RESERVED7;                 /*!< 0x60                                                                              */
    __IO uint32_t GPHEN[2];                  /*!< 0x64 ~ 0x68 GPHEN0,1 GPIO Pin High Detect Enable 0                                */
    __IO uint32_t RESERVED8;                 /*!< 0x6C                                                                              */
    __IO uint32_t GPLEN[2];                  /*!< 0x70 ~ 0x74 GPLEN0,1 GPIO Pin Low Detect Enable 0                                 */
    __IO uint32_t RESERVED9;                 /*!< 0x78                                                                              */
    __IO uint32_t GPAREN[2];                 /*!< 0x7c ~ 0x80 GPAREN0,1 GPIO Pin Async. Rising Edge Detect 0                        */
    __IO uint32_t RESERVED10;                /*!< 0x84                                                                              */
    __IO uint32_t GPAFEN[2];                 /*!< 0x88 ~ 0x8C GPAFEN0,1 GPIO Pin Async. Falling Edge Detect 0                       */
    __IO uint32_t RESERVED11[21];            /*!< 0x90 ~ 0xE0                                                                       */
    __IO uint32_t GPIO_PUP_PDN_CNTRL_REG[4]; /*!< 0xe4 ~ 0xf0 GPIO_PUP_PDN_CNTRL_REG0,1,2,3 GPIO Pull-up / Pull-down Register 0     */
} GPIO_Type;


extern uint8_t sibs[10];
extern uint8_t leds[3];

void InitGPIO(void);
void SetBase(uint32_t *ui32BaseAddr);
bool ConfigPin(uint8_t ui8Pin, uint8_t ui8Type);
bool WritePin(uint8_t ui8Pin, bool bValue);
bool ReadPin(uint8_t ui8Pin);
bool Toogle(uint8_t ui8Pin);

#ifdef __cplusplus
}
#endif

#endif
