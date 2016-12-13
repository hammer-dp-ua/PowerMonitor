#include <stdint.h>
#include <stm32f0xx_gpio.h>

#define DISPLAY_RS_PIN              GPIO_Pin_6
#define DISPLAY_RS_PORT             GPIOF
#define DISPLAY_RW_PIN              GPIO_Pin_15
#define DISPLAY_RW_PORT             GPIOB
#define DISPLAY_CHIP_ENABLE_PIN     GPIO_Pin_8
#define DISPLAY_CHIP_ENABLE_PORT    GPIOA
#define DISPLAY_DB4_PIN             GPIO_Pin_9
#define DISPLAY_DB4_POPT            GPIOA
#define DISPLAY_DB5_PIN             GPIO_Pin_10
#define DISPLAY_DB5_POPT            GPIOA
#define DISPLAY_DB6_PIN             GPIO_Pin_11
#define DISPLAY_DB6_POPT            GPIOA
#define DISPLAY_DB7_PIN             GPIO_Pin_12
#define DISPLAY_DB7_POPT            GPIOA
#define EIGHT_BITS                  1
#define FOUR_BITS                   0
#define OUTPUT                      1
#define INPUT                       0

/*
 * volatile uint32_t cnt = 1,000,000;
 * while (--cnt > 0);
 * 10,500,000 cycles will be executed
 */
#define HCLK      48000000
#define MS_150    HCLK * 0.15 / 10
#define MS_15     HCLK * 0.015 / 10
#define MS_5      HCLK * 0.005 / 10
#define MS_2      HCLK * 0.002 / 10
#define US_100    HCLK * 0.0001 / 10
#define US_50     HCLK * 0.00005 / 10 // 50us

#define VOLTAGE 1
#define CURRENT 0

void BC1602A_Init();
void sendDataToDisplay(uint8_t data, uint8_t isEightBits, uint8_t sendToDDRAM);
void initDisplayDataPins(uint8_t direction);
void waitWhileLCDIsBusy();
void setDDRAMAddress(uint8_t address);
void setStaticDisplayMatrix();
void writeDigitToDisplay(uint8_t position, uint16_t digit);
void writeVoltageOrCurrentToDisplay(uint8_t position, uint16_t digit, uint8_t);
void writePowerToDisplay(uint8_t position, uint32_t digit);
