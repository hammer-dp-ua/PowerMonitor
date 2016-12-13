#include "BC1602A_display.h"

void BC1602A_Init()
{
   //IWDG_ReloadCounter();

   GPIO_InitTypeDef gpioInitType;
   gpioInitType.GPIO_Pin = DISPLAY_RS_PIN;
   gpioInitType.GPIO_PuPd = GPIO_PuPd_NOPULL;
   gpioInitType.GPIO_Mode = GPIO_Mode_OUT;
   gpioInitType.GPIO_Speed = GPIO_Speed_Level_1; // Low 2 MHz
   gpioInitType.GPIO_OType = GPIO_OType_PP;
   GPIO_Init(DISPLAY_RS_PORT, &gpioInitType);

   gpioInitType.GPIO_Pin = DISPLAY_RW_PIN;
   GPIO_Init(DISPLAY_RW_PORT, &gpioInitType);

   gpioInitType.GPIO_Pin = DISPLAY_CHIP_ENABLE_PIN;
   GPIO_Init(DISPLAY_CHIP_ENABLE_PORT, &gpioInitType);

   initDisplayDataPins(OUTPUT);

   volatile uint32_t counter = MS_15;
   while (--counter > 0);

   sendDataToDisplay(0x3, FOUR_BITS, 0);
   counter = MS_5;
   while (--counter > 0);

   sendDataToDisplay(0x3, FOUR_BITS, 0);
   counter = US_100;
   while (--counter > 0);

   sendDataToDisplay(0x3, FOUR_BITS, 0);
   counter = US_50;
   while (--counter > 0);

   // Function set ( Set interface to be 4 bits long. )
   sendDataToDisplay(0x2, FOUR_BITS, 0);
   counter = US_50;
   while (--counter > 0);

   // Function set (Interface is 4 bits long. Specify the number of display lines and character font)
   sendDataToDisplay(0x28, EIGHT_BITS, 0);
   counter = US_50;
   while (--counter > 0);

   // Display off
   sendDataToDisplay(0x8, EIGHT_BITS, 0);
   counter = US_50;
   while (--counter > 0);

   // Clear display
   sendDataToDisplay(0x1, EIGHT_BITS, 0);
   counter = MS_2;
   while (--counter > 0);

   sendDataToDisplay(0x6, EIGHT_BITS, 0);
   counter = US_50;
   while (--counter > 0);

   //IWDG_ReloadCounter();

   sendDataToDisplay(0xC, EIGHT_BITS, 0); // Turn on display
}

void initDisplayDataPins(uint8_t direction)
{
   GPIO_InitTypeDef gpioInitType;
   gpioInitType.GPIO_Pin = DISPLAY_DB4_PIN | DISPLAY_DB5_PIN | DISPLAY_DB6_PIN | DISPLAY_DB7_PIN;
   gpioInitType.GPIO_PuPd = GPIO_PuPd_NOPULL;
   gpioInitType.GPIO_Speed = GPIO_Speed_Level_1; // Low 2 MHz

   if (direction == OUTPUT)
   {
      gpioInitType.GPIO_Mode = GPIO_Mode_OUT;
      gpioInitType.GPIO_OType = GPIO_OType_PP;
   }
   else
   {
      gpioInitType.GPIO_Mode = GPIO_Mode_IN;
   }
   GPIO_Init(DISPLAY_DB4_POPT, &gpioInitType); // Port for DB4 - DB7 pins is the same
}

// For pins 4-7. Data should be in LSB bits, i.e. xxxx1010
void sendDataToDisplay(uint8_t data, uint8_t isEightBits, uint8_t sendToDDRAM)
{
   volatile uint32_t counter;

   if (isEightBits)
   {
      //GPIO_WriteBit(DISPLAY_RW_PORT, DISPLAY_RW_PIN, DISABLE);
      GPIO_WriteBit(DISPLAY_DB4_POPT, DISPLAY_DB4_PIN, data & 16);
      GPIO_WriteBit(DISPLAY_DB5_POPT, DISPLAY_DB5_PIN, data & 32);
      GPIO_WriteBit(DISPLAY_DB6_POPT, DISPLAY_DB6_PIN, data & 64);
      GPIO_WriteBit(DISPLAY_DB7_POPT, DISPLAY_DB7_PIN, data & 128);
      if (sendToDDRAM)
      {
         GPIO_WriteBit(DISPLAY_RS_PORT, DISPLAY_RS_PIN, ENABLE);
      }
      GPIO_WriteBit(DISPLAY_CHIP_ENABLE_PORT, DISPLAY_CHIP_ENABLE_PIN, ENABLE);
      GPIO_WriteBit(DISPLAY_CHIP_ENABLE_PORT, DISPLAY_CHIP_ENABLE_PIN, DISABLE);
      if (sendToDDRAM)
      {
         GPIO_WriteBit(DISPLAY_RS_PORT, DISPLAY_RS_PIN, DISABLE);
      }
      counter = US_50;
      while (--counter > 0);
   }

   GPIO_WriteBit(DISPLAY_DB4_POPT, DISPLAY_DB4_PIN, data & 1);
   GPIO_WriteBit(DISPLAY_DB5_POPT, DISPLAY_DB5_PIN, data & 2);
   GPIO_WriteBit(DISPLAY_DB6_POPT, DISPLAY_DB6_PIN, data & 4);
   GPIO_WriteBit(DISPLAY_DB7_POPT, DISPLAY_DB7_PIN, data & 8);
   if (sendToDDRAM)
   {
      GPIO_WriteBit(DISPLAY_RS_PORT, DISPLAY_RS_PIN, ENABLE);
   }
   GPIO_WriteBit(DISPLAY_CHIP_ENABLE_PORT, DISPLAY_CHIP_ENABLE_PIN, ENABLE);
   GPIO_WriteBit(DISPLAY_CHIP_ENABLE_PORT, DISPLAY_CHIP_ENABLE_PIN, DISABLE);
   if (sendToDDRAM)
   {
      GPIO_WriteBit(DISPLAY_RS_PORT, DISPLAY_RS_PIN, DISABLE);
   }
   counter = US_50;
   while (--counter > 0);
   //IWDG_ReloadCounter();
}

void setDDRAMAddress(uint8_t address)
{
   address |= 0x80;
   sendDataToDisplay(address, EIGHT_BITS, 0);
   volatile uint32_t counter = US_50;
   while (--counter > 0);
   //IWDG_ReloadCounter();
}

void setStaticDisplayMatrix()
{
   sendDataToDisplay('I', EIGHT_BITS, 1);
   sendDataToDisplay('N', EIGHT_BITS, 1);

   setDDRAMAddress(40);
   sendDataToDisplay('O', EIGHT_BITS, 1);
   sendDataToDisplay('U', EIGHT_BITS, 1);
   sendDataToDisplay('T', EIGHT_BITS, 1);
}

void writeDigitToDisplay(uint8_t position, uint16_t digit)
{
   uint8_t digitsAmount = 5;
   sendDataToDisplay(4, EIGHT_BITS, 0); // Shift to left
   setDDRAMAddress(position + digitsAmount - 1);
   while (digitsAmount--)
   {
      sendDataToDisplay('0' + (uint8_t)(digit%10), EIGHT_BITS, 1);
      digit /= 10;
   }
}

void writeVoltageOrCurrentToDisplay(uint8_t position, uint16_t digit, uint8_t voltageOrCurrent)
{
   uint8_t digitsAmount = 3;
   sendDataToDisplay(4, EIGHT_BITS, 0); // Shift to left
   setDDRAMAddress(position + digitsAmount -1 + 1 + 1);
   if (voltageOrCurrent == VOLTAGE)
   {
      sendDataToDisplay('V', EIGHT_BITS, 1);
   }
   else
   {
      sendDataToDisplay('A', EIGHT_BITS, 1);
   }

   uint8_t digitsWritten = 0;
   while (digitsAmount--)
   {
      sendDataToDisplay('0' + (uint8_t)(digit%10), EIGHT_BITS, 1);
      digitsWritten++;
      digit /= 10;
      if (voltageOrCurrent == VOLTAGE && digitsWritten == 1)
      {
         sendDataToDisplay('.', EIGHT_BITS, 1);
      }
      else if (voltageOrCurrent == CURRENT && digitsWritten == 2)
      {
         sendDataToDisplay('.', EIGHT_BITS, 1);
      }
   }
}

void writePowerToDisplay(uint8_t position, uint32_t digit)
{
   uint8_t digitsAmount = 6;
   uint8_t digitsAmountDownCounter = digitsAmount;
   sendDataToDisplay(4, EIGHT_BITS, 0); // Shift to left
   setDDRAMAddress(position + digitsAmount - 1 + 1 + 1); // + 'W' + '.'
   sendDataToDisplay('W', EIGHT_BITS, 1);

   while (digitsAmountDownCounter--)
   {
      sendDataToDisplay('0' + (uint8_t)(digit%10), EIGHT_BITS, 1);
      digit /= 10;

      if (digitsAmountDownCounter == digitsAmount - 2)
      {
         sendDataToDisplay('.', EIGHT_BITS, 1);
      }
   }
}
