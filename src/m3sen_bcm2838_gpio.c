#include <stdio.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <bcm_host.h>
#include "m3sen_bcm2838_gpio.h"
#ifdef LOG
#include "log.h"
#endif

GPIO_Type *gpio_base;

uint8_t sibs[10] = {JBB1, JBB2, JBB3, JBB4, JBB5, JBB6, JBB7, JBB8, JBB9, JBB10};
uint8_t leds[3]  = {RED, YELLOW, GREEN};

void InitGPIO(void) {
    uint32_t *gpio_memory_map;

    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) {
#ifdef LOG
        logw(0, "TLM: can't open /dev/mem \n");
#endif
        exit(-1);
    }
    int model = bcm_host_get_model_type();
#ifdef LOG
    switch (model) {
    case BCM_HOST_BOARD_TYPE_MODELA:
        logw(7, "Pi A\n");
        break;
    case BCM_HOST_BOARD_TYPE_MODELB:
        logw(7, "Pi B\n");
        break;
    case BCM_HOST_BOARD_TYPE_MODELAPLUS:
        logw(7, "Pi A+\n");
        break;
    case BCM_HOST_BOARD_TYPE_MODELBPLUS:
        logw(7, "Pi B+\n");
        break;
    case BCM_HOST_BOARD_TYPE_PI2MODELB:
        logw(7, "Pi 2B\n");
        break;
    case BCM_HOST_BOARD_TYPE_ALPHA:
        logw(7, "Pi Alpha\n");
        break;
    case BCM_HOST_BOARD_TYPE_CM:
        logw(7, "Pi CM\n");
        break;
    case BCM_HOST_BOARD_TYPE_CM2:
        logw(7, "Pi CM2\n");
        break;
    case BCM_HOST_BOARD_TYPE_PI3MODELB:
        logw(7, "Pi 3B\n");
        break;
    case BCM_HOST_BOARD_TYPE_PI0:
        logw(7, "Pi 0\n");
        break;
    case BCM_HOST_BOARD_TYPE_CM3:
        logw(7, "Pi CM3\n");
        break;
    case BCM_HOST_BOARD_TYPE_CUSTOM:
        logw(7, "Pi Custom\n");
        break;
    case BCM_HOST_BOARD_TYPE_PI0W:
        logw(7, "Pi 0W\n");
        break;
    case BCM_HOST_BOARD_TYPE_PI3MODELBPLUS:
        logw(7, "Pi 3B+\n");
        break;
    case BCM_HOST_BOARD_TYPE_PI3MODELAPLUS:
        logw(7, "Pi 3A+\n");
        break;
    case BCM_HOST_BOARD_TYPE_FPGA:
        logw(7, "Pi FPGA\n");
        break;
    case BCM_HOST_BOARD_TYPE_CM3PLUS:
        logw(7, "Pi CM3+\n");
        break;
    case BCM_HOST_BOARD_TYPE_PI4MODELB:
        logw(7, "Pi 4B\n");
        break;
    default:
        logw(7, "Unknown\n");
    }

    int      pid           = bcm_host_get_processor_id();
    uint32_t ui32PheriBase = bcm_host_get_peripheral_address();

    switch (pid) {
    case BCM_HOST_PROCESSOR_BCM2835:
        logw(7, "Processor: BCM2835\n");
        break;
    case BCM_HOST_PROCESSOR_BCM2836:
        logw(7, "Processor: BCM2836\n");
        break;
    case BCM_HOST_PROCESSOR_BCM2837:
        logw(7, "Processor: BCM2837\n");
        break;
    case BCM_HOST_PROCESSOR_BCM2838:
        logw(7, "Processor: BCM2838\n");
        break;
    default:
        logw(7, "Processor: unknown\n");
        break;
    }
    logw(7, "Pheri BASE 0x%08x\n", ui32PheriBase);
#endif

    gpio_memory_map = (uint32_t *)mmap(0, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, fd, GPIO_BASE);

    if (gpio_memory_map == MAP_FAILED) {
#ifdef LOG
        logw(0, " Error : mmap \n");
#endif
        exit(-1);
    }

    SetBase(gpio_memory_map);
    ConfigPin(JBB1, OUTPUT);
    WritePin(JBB1, 1);
    ConfigPin(JBB2, OUTPUT);
    WritePin(JBB2, 1);
    ConfigPin(JBB3, OUTPUT);
    WritePin(JBB3, 1);
    ConfigPin(JBB4, OUTPUT);
    WritePin(JBB4, 1);
    ConfigPin(JBB5, OUTPUT);
    WritePin(JBB5, 1);
    ConfigPin(JBB6, OUTPUT);
    WritePin(JBB6, 1);
    ConfigPin(JBB7, OUTPUT);
    WritePin(JBB7, 1);
    ConfigPin(JBB8, OUTPUT);
    WritePin(JBB8, 1);
    ConfigPin(JBB9, OUTPUT);
    WritePin(JBB9, 1);
    ConfigPin(JBB10, OUTPUT);
    WritePin(JBB10, 1);

    ConfigPin(RED, OUTPUT);
    WritePin(RED, 0);
    ConfigPin(YELLOW, OUTPUT);
    WritePin(YELLOW, 0);
    ConfigPin(GREEN, OUTPUT);
    WritePin(GREEN, 1);
}
void SetBase(uint32_t *ui32BaseAddr) {
    gpio_base = (GPIO_Type *)ui32BaseAddr;
}

bool ConfigPin(uint8_t ui8Pin, uint8_t ui8Type) {
    uint8_t ui8Index  = ui8Pin / 10;
    uint8_t ui8PinPos = ui8Pin % 10;

    if (ui8Pin > GPIO57)
        return false;

    gpio_base->GPFSEL[ui8Index] &= ~(ui8Type << (ui8PinPos * 3));
    gpio_base->GPFSEL[ui8Index] |= ui8Type << (ui8PinPos * 3);
    return true;
}

bool WritePin(uint8_t ui8Pin, bool bValue) {
    uint8_t ui8Index  = ui8Pin / 32;
    uint8_t ui8PinPos = ui8Pin % 32;

    if (ui8Pin > GPIO57) return false;
#ifdef DEBUG
    logw(7, "Index: 0x%08x\n", gpio_base->GPLEV[ui8Index]);
    logw(7, "[%d] Index: %d, pos: %d, val:%d\n", ui8Pin, ui8Index, ui8PinPos, bValue);
#endif
    if (bValue)
        gpio_base->GPSET[ui8Index] = (0x01 << ui8PinPos);
    else
        gpio_base->GPCLR[ui8Index] = (0x01 << ui8PinPos);
#ifdef DEBUG
    logw(7, "Index: 0x%08x\n", gpio_base->GPLEV[ui8Index]);
#endif
    return true;
}
bool ReadPin(uint8_t ui8Pin) {
    uint8_t ui8Index  = ui8Pin / 32;
    uint8_t ui8PinPos = ui8Pin % 32;

    if (ui8Pin > GPIO57)
        return false;

    return gpio_base->GPLEV[ui8Index] &= (0x01 << ui8PinPos);
}

bool Toogle(uint8_t ui8Pin) {
    uint8_t ui8Index  = ui8Pin / 32;
    uint8_t ui8PinPos = ui8Pin % 32;
    if (ReadPin(ui8Pin))
        gpio_base->GPCLR[ui8Index] = (0x01 << ui8PinPos);
    else
        gpio_base->GPSET[ui8Index] = (0x01 << ui8PinPos);
    return true;
}