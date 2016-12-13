#include "stm32f0xx.h"
#include "arm_math.h"
#include "BC1602A_display.h"

#define A_SECOND_INTERRUPT_FLAG           0x1
#define VOLTAGE_END_OF_CONVERSION_FLAG    0x2
#define CURRENT_END_OF_CONVERSION_FLAG    0x4
#define DISCHARGING_FLAG                  0x8
#define VOLTAGE_CHANNEL                   ADC_Channel_5

#define ZERO_CURRENT_ADC_BITS 32810 // Value when there is no any load
#define ACS756_mVA            40 //
#define MAX_VOLTAGE           16.5F // Max voltage

#define ADC1_DR_ADDRESS (uint32_t)&(ADC1->DR)

#define POWER_5V_PIN    GPIO_Pin_7
#define POWER_5V_PORT   GPIOF

#define VOLTAGE_ADC_DMA_INDEX 0

volatile char constant[] __attribute__ ((section(".text.const"))) = "123456123456123456123456123456123456123456123456123456123456123456123456123456123456123456123456123456123456123456123456123456123456123456123456123456123456123456123456123456123456123456123456123456123456123456123456123456123456123456123456123456123456123456123456123456123456123456123456123456123456123456123456123456123456123456123456123456123456123456123456123456123456123456123456123456123456123456123456";

volatile uint8_t generalFlags;
volatile uint32_t seconds;

float outPowerAccumulator;
float inPowerAccumulator;
__IO uint16_t adcDmaConvertedDataTable[1];
volatile uint8_t spiInterruptsCounter;
volatile uint16_t spiAdcData;

void IWDG_Config();
void Clock_Config();
void Pins_Config();
void TIMER3_Confing();
void ADC_Config();
void RTC_Config();
uint16_t getADCValue(uint32_t channel);
void setFlag(uint8_t flag);
void resetFlag(uint8_t flag);
void DMA_Config();
void SPI_Config();

void TIM3_IRQHandler()
{
   TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

   if (TIM3->CR1 & TIM_CR1_CEN)
   {
      setFlag(A_SECOND_INTERRUPT_FLAG);
   }
}

void ADC1_COMP_IRQHandler()
{
   ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
}

void RTC_IRQHandler()
{
   RTC_ClearITPendingBit(RTC_IT_ALRA);
   EXTI_ClearITPendingBit(EXTI_Line17);

   seconds++;
   setFlag(A_SECOND_INTERRUPT_FLAG);
}

void DMA1_Channel1_IRQHandler()
{
   if (DMA_GetITStatus(DMA1_IT_TC1))
   {
      DMA_ClearITPendingBit(DMA1_IT_TC1);

      setFlag(VOLTAGE_END_OF_CONVERSION_FLAG);
   }
}

void SPI2_IRQHandler()
{
   spiInterruptsCounter++;

   if (spiInterruptsCounter == 1)
   {
      spiAdcData = ((uint16_t)SPI_ReceiveData8(SPI2)) << 12;
   }
   else if (spiInterruptsCounter == 2)
   {
      spiAdcData |= ((uint16_t)SPI_ReceiveData8(SPI2)) << 4;
   }
   else if (spiInterruptsCounter == 3)
   {
      spiAdcData |= ((uint16_t)SPI_ReceiveData8(SPI2)) >> 4;
   }

   if (spiInterruptsCounter >= 3)
   {
      spiInterruptsCounter = 0;
      SPI_Cmd(SPI2, DISABLE);
      setFlag(CURRENT_END_OF_CONVERSION_FLAG);
   }
}

int main()
{
   //IWDG_Config();
   Clock_Config();
   Pins_Config();
   DMA_Config();
   ADC_Config();
   TIMER3_Confing();
   SPI_Config();
   RTC_Config();
   GPIO_WriteBit(POWER_5V_PORT, POWER_5V_PIN, ENABLE);
   BC1602A_Init();

   setStaticDisplayMatrix();

   while (1)
   {
      if (generalFlags & A_SECOND_INTERRUPT_FLAG)
      {
         resetFlag(A_SECOND_INTERRUPT_FLAG);

         SPI_Cmd(SPI2, ENABLE);
         ADC_StartOfConversion(ADC1);
      }

      if (generalFlags & VOLTAGE_END_OF_CONVERSION_FLAG && generalFlags & CURRENT_END_OF_CONVERSION_FLAG)
      {
         resetFlag(VOLTAGE_END_OF_CONVERSION_FLAG | CURRENT_END_OF_CONVERSION_FLAG);

         float voltage = ((uint32_t) adcDmaConvertedDataTable[VOLTAGE_ADC_DMA_INDEX]) * MAX_VOLTAGE / 4096;
         float current = 0.0;

         if (spiAdcData > ZERO_CURRENT_ADC_BITS && spiAdcData - ZERO_CURRENT_ADC_BITS > 25) // 25 bits are 50mA
         {
            // I = bits * 5 / (65535 * 0.04). If V=3.25V: 0.026. 3.25 / 5 = 0.65. 40mV/A * 0.65 = 26mV/A
            current = (spiAdcData - ZERO_CURRENT_ADC_BITS) / 524.28; // bits / 524.28

            outPowerAccumulator += voltage * current;
         }
         else if (spiAdcData < ZERO_CURRENT_ADC_BITS && ZERO_CURRENT_ADC_BITS - spiAdcData > 25)
         {
            current = (ZERO_CURRENT_ADC_BITS - spiAdcData) / 524.28;
            inPowerAccumulator += voltage * current;
         }

         if (!(generalFlags & DISCHARGING_FLAG) && (outPowerAccumulator / 3600) > 100)
         {
            setFlag(DISCHARGING_FLAG);
            inPowerAccumulator = 0.0;
         }

         if ((generalFlags & DISCHARGING_FLAG) && (inPowerAccumulator > outPowerAccumulator))
         {
            resetFlag(DISCHARGING_FLAG);
            outPowerAccumulator = 0.0;
         }

         if (!(generalFlags & DISCHARGING_FLAG) && (outPowerAccumulator < 1) && seconds > 4 * 3600) // 4 days
         {
            seconds = 0;
            inPowerAccumulator = 0.0;
            // Send data to server
         }

         writePowerToDisplay(43, (uint32_t) (outPowerAccumulator / 36)); // Power * 100
         writeVoltageOrCurrentToDisplay(0xB, (uint16_t) (voltage * 10), VOLTAGE);
         writeVoltageOrCurrentToDisplay(0x4B, (uint16_t) (current * 100), CURRENT);

         seconds = constant[0];
      }
   }
}

void IWDG_Config()
{
   IWDG_Enable();
   IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
   IWDG_SetPrescaler(IWDG_Prescaler_256);
   IWDG_SetReload(156); // 1 second
   while (IWDG_GetFlagStatus(IWDG_FLAG_PVU) == SET);
   while (IWDG_GetFlagStatus(IWDG_FLAG_RVU) == SET);
}

void Clock_Config()
{
   RCC_PCLKConfig(RCC_HCLK_Div16); // Timers' clock will be PCLK x2
}

void Pins_Config()
{
   RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOF, ENABLE);

   GPIO_InitTypeDef gpioInitType;
   gpioInitType.GPIO_Pin = 0x9FFF; // Pins 0 - 12, 15
   gpioInitType.GPIO_Mode = GPIO_Mode_IN;
   gpioInitType.GPIO_Speed = GPIO_Speed_Level_1; // Low 2 MHz
   gpioInitType.GPIO_PuPd = GPIO_PuPd_UP;
   GPIO_Init(GPIOA, &gpioInitType);

   gpioInitType.GPIO_Pin = GPIO_Pin_All;
   GPIO_Init(GPIOB, &gpioInitType);

   gpioInitType.GPIO_PuPd = GPIO_PuPd_NOPULL;
   gpioInitType.GPIO_Mode = GPIO_Mode_OUT;
   gpioInitType.GPIO_Pin = POWER_5V_PIN;
   GPIO_Init(POWER_5V_PORT, &gpioInitType);
}

void TIMER3_Confing()
{
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

   TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
   TIM_TimeBaseStructure.TIM_Period = 0xFFFF; // 100
   TIM_TimeBaseStructure.TIM_Prescaler = 0; // 60000. 48MHz / 16 / 60000 * 2. The counter clock frequency CK_CNT is equal to fCK_PSC/ (PSC[15:0] + 1).
   TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
   TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
   TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

   /*NVIC_EnableIRQ(TIM3_IRQn);
   NVIC_SetPriority(TIM3_IRQn, 0);
   TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

   TIM_Cmd(TIM3, ENABLE);*/
}

void ADC_Config()
{
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

   GPIO_InitTypeDef gpioInitType;
   gpioInitType.GPIO_Pin = GPIO_Pin_5;
   gpioInitType.GPIO_PuPd = GPIO_PuPd_NOPULL;
   gpioInitType.GPIO_Mode = GPIO_Mode_AN;
   gpioInitType.GPIO_Speed = GPIO_Speed_Level_1; // Low 2 MHz
   GPIO_Init(GPIOA, &gpioInitType);

   ADC_InitTypeDef adcInitType;
   adcInitType.ADC_ContinuousConvMode = ENABLE;
   adcInitType.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
   ADC_StructInit(&adcInitType);
   ADC_Init(ADC1, &adcInitType);

   ADC_ClockModeConfig(ADC1, ADC_ClockMode_SynClkDiv4);

   ADC_DMARequestModeConfig(ADC1, ADC_DMAMode_Circular);
   ADC_DMACmd(ADC1, ENABLE);

   ADC_ChannelConfig(ADC1, VOLTAGE_CHANNEL, ADC_SampleTime_41_5Cycles);

   ADC_AutoPowerOffCmd(ADC1, ENABLE);

   ADC_GetCalibrationFactor(ADC1);

   ADC_Cmd(ADC1, ENABLE);
   while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADEN));
}

void DMA_Config()
{
   RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 , ENABLE);

   DMA_InitTypeDef dmaInitType;
   dmaInitType.DMA_PeripheralBaseAddr = ADC1_DR_ADDRESS;
   dmaInitType.DMA_MemoryBaseAddr = (uint32_t)&adcDmaConvertedDataTable[0];
   dmaInitType.DMA_DIR = DMA_DIR_PeripheralSRC;
   dmaInitType.DMA_BufferSize = 1;
   dmaInitType.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
   dmaInitType.DMA_MemoryInc = DMA_MemoryInc_Disable; // DMA_MemoryInc_Enable if DMA_InitTypeDef.DMA_BufferSize > 1
   dmaInitType.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
   dmaInitType.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
   dmaInitType.DMA_Mode = DMA_Mode_Circular;
   dmaInitType.DMA_Priority = DMA_Priority_High;
   dmaInitType.DMA_M2M = DMA_M2M_Disable;
   DMA_Init(DMA1_Channel1, &dmaInitType);

   DMA_Cmd(DMA1_Channel1, ENABLE);

   DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);
   NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

void RTC_Config()
{
   RCC_BackupResetCmd(ENABLE);
   RCC_BackupResetCmd(DISABLE);

   RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
   PWR_BackupAccessCmd(ENABLE);

   RCC_LSEDriveConfig(RCC_LSEDrive_MediumHigh); // RCC_LSEDrive_High, RCC_LSEDrive_MediumHigh, RCC_LSEDrive_MediumLow, RCC_LSEDrive_Low
   RCC_LSEConfig(RCC_LSE_ON);
   while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET);

   RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
   RTC_InitTypeDef rtcInitType;
   RTC_StructInit(&rtcInitType);
   ErrorStatus errorStatus = RTC_Init(&rtcInitType);
   errorStatus |= RTC_SmoothCalibConfig(RTC_SmoothCalibPeriod_32sec, 0, 470); // 470 masked pulses per 32 s
   RCC_RTCCLKCmd(ENABLE);
   RTC_WaitForSynchro(); // Won't start without this. Software reset will be required

   EXTI_InitTypeDef extiTypeDef;
   EXTI_ClearITPendingBit(EXTI_Line17);
   extiTypeDef.EXTI_Line = EXTI_Line17;
   extiTypeDef.EXTI_Mode = EXTI_Mode_Interrupt;
   extiTypeDef.EXTI_Trigger = EXTI_Trigger_Rising;
   extiTypeDef.EXTI_LineCmd = ENABLE;
   EXTI_Init(&extiTypeDef);
   NVIC_EnableIRQ(RTC_IRQn);
   RTC_ITConfig(RTC_IT_ALRA, ENABLE);
   RTC_ClearITPendingBit(RTC_IT_ALRA);

   RTC_WriteProtectionCmd(DISABLE);
   RTC->ALRMAR = RTC_AlarmMask_All;
   RTC_WriteProtectionCmd(ENABLE);

   errorStatus |= RTC_AlarmCmd(RTC_Alarm_A, ENABLE);
}

void SPI_Config()
{
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

   GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_0);
   GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_0);
   GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_0);

   GPIO_InitTypeDef gpioInitType;
   gpioInitType.GPIO_Pin = GPIO_Pin_12;
   gpioInitType.GPIO_PuPd = GPIO_PuPd_UP;
   gpioInitType.GPIO_Mode = GPIO_Mode_AF;
   gpioInitType.GPIO_OType = GPIO_OType_PP;
   gpioInitType.GPIO_Speed = GPIO_Speed_Level_3; // 50 MHz
   GPIO_Init(GPIOB, &gpioInitType);

   gpioInitType.GPIO_Pin = GPIO_Pin_13;
   gpioInitType.GPIO_PuPd = GPIO_PuPd_DOWN;
   GPIO_Init(GPIOB, &gpioInitType);

   gpioInitType.GPIO_Pin = GPIO_Pin_14;
   GPIO_Init(GPIOB, &gpioInitType);

   SPI_InitTypeDef spiInitType;
   spiInitType.SPI_Direction = SPI_Direction_2Lines_RxOnly;
   spiInitType.SPI_Mode = SPI_Mode_Master;
   spiInitType.SPI_DataSize = SPI_DataSize_8b;
   spiInitType.SPI_CPOL = SPI_CPOL_High;
   spiInitType.SPI_CPHA = SPI_CPHA_1Edge;
   spiInitType.SPI_NSS = SPI_NSS_Hard;
   spiInitType.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
   spiInitType.SPI_FirstBit = SPI_FirstBit_MSB;
   spiInitType.SPI_CRCPolynomial = 7;
   SPI_Init(SPI2, &spiInitType);
   SPI2->CR2 |= 0x4;

   SPI_RxFIFOThresholdConfig(SPI2, SPI_RxFIFOThreshold_QF);

   NVIC_EnableIRQ(SPI2_IRQn);
   SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, ENABLE);
}

/*void writeToBuffer()
{
   if (pinsAndTimerIndex < 70)
   {
      timerBuffer[pinsAndTimerIndex] = TIM_GetCounter(TIM3);
      volatile uint8_t pins = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4);
      pins <<= 1;
      pins |= GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_3);
      pins <<= 1;
      pins |= GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_2);
      pinsBuffer[pinsAndTimerIndex] = pins;
   }
   pinsAndTimerIndex++;
}*/

void setFlag(uint8_t flag)
{
   generalFlags |= flag;
}

void resetFlag(uint8_t flag)
{
   generalFlags &= ~(generalFlags & flag);
}
