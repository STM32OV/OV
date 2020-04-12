/**
  ******************************************************************************
  * @file    UART/UART_TwoBoards_ComPolling/Src/main.c 
  * @author  MCD Application Team
  * @brief   This sample code shows how to use STM32F4xx UART HAL API to transmit 
  *          and receive a data buffer with a communication process based on
  *          polling transfer. 
  *          The communication is done using 2 Boards.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */

/** @addtogroup UART_TwoBoards_ComPolling
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define TRANSMITTER_BOARD

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* UART handler declaration */
UART_HandleTypeDef UartHandle;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);
static uint16_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);

/* Private functions ---------------------------------------------------------*/
#define I2C_ADDRESS        0x6C
/* Buffer used for reception */


/* Private variables ---------------------------------------------------------*/
/* I2C handler declaration */
I2C_HandleTypeDef I2cHandle;


void InitI2C()
{
	/*##-1- Configure the I2C peripheral ######################################*/
	I2cHandle.Instance             = I2Cx;

	I2cHandle.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
	I2cHandle.Init.ClockSpeed      = 100000;
	I2cHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	I2cHandle.Init.DutyCycle       = I2C_DUTYCYCLE_16_9;
	I2cHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	I2cHandle.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
	I2cHandle.Init.OwnAddress1     = I2C_ADDRESS;
	I2cHandle.Init.OwnAddress2     = 0xFE;

	if(HAL_I2C_Init(&I2cHandle) != HAL_OK)
	{
	  /* Initialization Error */
	  Error_Handler();
	}

}

uint32_t uwPeriod = 0;
/* Pulses value */
uint32_t uwPulse1, uwPulse2, uwPulse3, uwPulse4 = 0;

/* Timer handler declaration */
TIM_HandleTypeDef    TimHandlePWM;

/* Timer Output Compare Configuration Structure declaration */
TIM_OC_InitTypeDef sConfigPWM;

void InitTimerPWM()
{
	  TimHandlePWM.Instance = TIM8;

	  TimHandlePWM.Init.Period = 4999;
	  TimHandlePWM.Init.Prescaler = 83;
	  TimHandlePWM.Init.ClockDivision     = 0;
	  TimHandlePWM.Init.CounterMode       = TIM_COUNTERMODE_UP ;
	  TimHandlePWM.Init.RepetitionCounter = 1;

	  if(HAL_TIM_PWM_Init(&TimHandlePWM) != HAL_OK)
	  {
	    /* Initialization Error */
	    Error_Handler();
	  }

	  /*##-2- Configure the PWM channels #########################################*/
	  /* Common configuration for all channels */
	  sConfigPWM.OCMode      = TIM_OCMODE_PWM2;
	  sConfigPWM.OCFastMode  = TIM_OCFAST_DISABLE;
	  sConfigPWM.OCPolarity  = TIM_OCPOLARITY_LOW;
	  sConfigPWM.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	  sConfigPWM.OCIdleState = TIM_OCIDLESTATE_RESET;
	  sConfigPWM.OCNIdleState= TIM_OCNIDLESTATE_RESET;

	  /* Set the pulse value for channel 1 */
	  sConfigPWM.Pulse = uwPulse1;
	  if(HAL_TIM_PWM_ConfigChannel(&TimHandlePWM, &sConfigPWM, TIM_CHANNEL_1) != HAL_OK)
	  {
	    /* Configuration Error */
	    Error_Handler();
	  }

	  if(HAL_TIM_PWM_ConfigChannel(&TimHandlePWM, &sConfigPWM, TIM_CHANNEL_2) != HAL_OK)
	  {
	    /* Configuration Error */
	    Error_Handler();
	  }

	  if(HAL_TIM_PWM_ConfigChannel(&TimHandlePWM, &sConfigPWM, TIM_CHANNEL_3) != HAL_OK)
	  {
	    /* Configuration Error */
	    Error_Handler();
	  }


	  if(HAL_TIM_PWM_Start(&TimHandlePWM, TIM_CHANNEL_1) != HAL_OK)
	  {
		Error_Handler();
	  }

	  if(HAL_TIM_PWM_Start(&TimHandlePWM, TIM_CHANNEL_2) != HAL_OK)
	  {
		Error_Handler();
	  }

	  if(HAL_TIM_PWM_Start(&TimHandlePWM, TIM_CHANNEL_3) != HAL_OK)
	  {
		Error_Handler();
	  }

	  {
		GPIO_InitTypeDef   GPIO_InitStruct;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
		GPIO_InitStruct.Pin = PWM_PIN0;
		HAL_GPIO_Init(PWM_GPIO_PORT, &GPIO_InitStruct);
	  }

	  {
		GPIO_InitTypeDef   GPIO_InitStruct;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
		GPIO_InitStruct.Pin = PWM_PIN1;
		HAL_GPIO_Init(PWM_GPIO_PORT, &GPIO_InitStruct);
	  }

	  {
		GPIO_InitTypeDef   GPIO_InitStruct;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
		GPIO_InitStruct.Pin = PWM_PIN2;
		HAL_GPIO_Init(PWM_GPIO_PORT, &GPIO_InitStruct);
	  }
}

/* Timer handler declaration */
TIM_HandleTypeDef    TimHandlePWMPiezo;

/* Timer Output Compare Configuration Structure declaration */
TIM_OC_InitTypeDef sConfigPWMPiezo;

void InitTimerPWMPiezo()
{
	TimHandlePWMPiezo.Instance = TIM5;

	TimHandlePWMPiezo.Init.Period = 416;//2.4kHz
	TimHandlePWMPiezo.Init.Prescaler = 83;
	TimHandlePWMPiezo.Init.ClockDivision     = 0;
	TimHandlePWMPiezo.Init.CounterMode       = TIM_COUNTERMODE_UP ;
	TimHandlePWMPiezo.Init.RepetitionCounter = 1;

	  if(HAL_TIM_PWM_Init(&TimHandlePWMPiezo) != HAL_OK)
	  {
	    /* Initialization Error */
	    Error_Handler();
	  }

	  /*##-2- Configure the PWM channels #########################################*/
	  /* Common configuration for all channels */
	  sConfigPWMPiezo.OCMode      = TIM_OCMODE_PWM2;
	  sConfigPWMPiezo.OCFastMode  = TIM_OCFAST_DISABLE;
	  sConfigPWMPiezo.OCPolarity  = TIM_OCPOLARITY_LOW;
	  sConfigPWMPiezo.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	  sConfigPWMPiezo.OCIdleState = TIM_OCIDLESTATE_RESET;
	  sConfigPWMPiezo.OCNIdleState= TIM_OCNIDLESTATE_RESET;

	  /* Set the pulse value for channel 1 */
	  sConfigPWMPiezo.Pulse = 208;
	  if(HAL_TIM_PWM_ConfigChannel(&TimHandlePWMPiezo, &sConfigPWMPiezo, TIM_CHANNEL_1) != HAL_OK)
	  {
	    /* Configuration Error */
	    Error_Handler();
	  }

	  sConfigPWMPiezo.OCPolarity  = TIM_OCPOLARITY_HIGH;
	  if(HAL_TIM_PWM_ConfigChannel(&TimHandlePWMPiezo, &sConfigPWMPiezo, TIM_CHANNEL_2) != HAL_OK)
	  {
	    /* Configuration Error */
	    Error_Handler();
	  }



	  if(HAL_TIM_PWM_Start(&TimHandlePWMPiezo, TIM_CHANNEL_1) != HAL_OK)
	  {
		Error_Handler();
	  }

	  if(HAL_TIM_PWM_Start(&TimHandlePWMPiezo, TIM_CHANNEL_2) != HAL_OK)
	  {
		Error_Handler();
	  }
}


uint8_t piezo_alarm=0;

void PiezoOn()
{
	  {
		GPIO_InitTypeDef   GPIO_InitStruct;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
		GPIO_InitStruct.Pin = PIEZO_PIN_A;
		HAL_GPIO_Init(PIEZO_PORT, &GPIO_InitStruct);
	  }

	  {
		GPIO_InitTypeDef   GPIO_InitStruct;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
		GPIO_InitStruct.Pin = PIEZO_PIN_B;
		HAL_GPIO_Init(PIEZO_PORT, &GPIO_InitStruct);
	  }
}

void PiezoOff()
{
	  {
		GPIO_InitTypeDef   GPIO_InitStruct;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
		GPIO_InitStruct.Pin = PIEZO_PIN_A;
		HAL_GPIO_Init(PIEZO_PORT, &GPIO_InitStruct);
	  }

	  {
		GPIO_InitTypeDef   GPIO_InitStruct;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
		GPIO_InitStruct.Pin = PIEZO_PIN_B;
		HAL_GPIO_Init(PIEZO_PORT, &GPIO_InitStruct);
	  }
}


ADC_HandleTypeDef    AdcHandle;
ADC_ChannelConfTypeDef sConfig;
ADC_ChannelConfTypeDef sConfigy;
ADC_ChannelConfTypeDef sConfigz;
DMA_HandleTypeDef  hdma_adc;


/* Variable used to get converted value */
__IO uint16_t uhADCxConvertedValue[ADCSAMPLECOUNT*4] = {0};

void InitADC()
{
	/*##-1- Configure the ADC peripheral #######################################*/
	AdcHandle.Instance = ADCx;

	AdcHandle.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV8;
	AdcHandle.Init.Resolution = ADC_RESOLUTION_12B;
	AdcHandle.Init.ScanConvMode = ENABLE;
	AdcHandle.Init.ContinuousConvMode = ENABLE;
	AdcHandle.Init.DiscontinuousConvMode = DISABLE;
	AdcHandle.Init.NbrOfDiscConversion = 0;
	AdcHandle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	AdcHandle.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
	AdcHandle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	AdcHandle.Init.NbrOfConversion = 4;
	AdcHandle.Init.DMAContinuousRequests = ENABLE;
	AdcHandle.Init.EOCSelection = DISABLE;

	if(HAL_ADC_Init(&AdcHandle) != HAL_OK)
	{
	  /* Initialization Error */
	  Error_Handler();
	}

	/*##-2- Configure ADC regular channel ######################################*/
	/* Note: Considering IT occurring after each number of size of              */
	/*       "uhADCxConvertedValue"  ADC conversions (IT by DMA end             */
	/*       of transfer), select sampling time and ADC clock with sufficient   */
	/*       duration to not create an overhead situation in IRQHandler.        */
	sConfig.Channel = ADCx_CHANNEL;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES   ;
	sConfig.Offset = 0;

	if(HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
	{
	  /* Channel Configuration Error */
	  Error_Handler();
	}

	sConfigy.Channel = ADCy_CHANNEL;
	sConfigy.Rank = 2;
	sConfigy.SamplingTime = ADC_SAMPLETIME_480CYCLES   ;
	sConfigy.Offset = 0;

	if(HAL_ADC_ConfigChannel(&AdcHandle, &sConfigy) != HAL_OK)
	{
	  /* Channel Configuration Error */
	  Error_Handler();
	}

	sConfigz.Channel = ADCz_CHANNEL;
	sConfigz.Rank = 3;
	sConfigz.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	sConfigz.Offset = 0;

	if(HAL_ADC_ConfigChannel(&AdcHandle, &sConfigz) != HAL_OK)
	{
	  /* Channel Configuration Error */
	  Error_Handler();
	}

	sConfigz.Channel = ADCw_CHANNEL;
	sConfigz.Rank = 4;
	sConfigz.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	sConfigz.Offset = 0;

	if(HAL_ADC_ConfigChannel(&AdcHandle, &sConfigz) != HAL_OK)
	{
	  /* Channel Configuration Error */
	  Error_Handler();
	}

	/*##-3- Start the conversion process and enable interrupt ##################*/
	/* Note: Considering IT occurring after each number of ADC conversions      */
	/*       (IT by DMA end of transfer), select sampling time and ADC clock    */
	/*       with sufficient duration to not create an overhead situation in    */
	/*        IRQHandler. */
	if(HAL_ADC_Start_DMA(&AdcHandle, (uint16_t*)&uhADCxConvertedValue[0], 1024*8*4) != HAL_OK)
	{
	  Error_Handler();
	}
}




//TODO: Make this better.
int Circular(int value, int bufsize)
{
	while(value<0)value+=bufsize;
	return value;
}


uint32_t ComputeAverage(int pos_in_buf)
{
	uint32_t r=0;

	for(int i=0;i<64;i++)
	{
		r+= (uint32_t)uhADCxConvertedValue[Circular(pos_in_buf-4-4*i, ADCSAMPLECOUNT*4)];
	}
	return r/64;
}



SPI_HandleTypeDef SpiHandle;

void InitSPI()
{
	SpiHandle.Instance               = SPIx;

	SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	SpiHandle.Init.Direction         = SPI_DIRECTION_2LINES;
	SpiHandle.Init.CLKPhase          = SPI_PHASE_1EDGE;
	SpiHandle.Init.CLKPolarity       = SPI_POLARITY_LOW;
	SpiHandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
	SpiHandle.Init.CRCPolynomial     = 7;
	SpiHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
	SpiHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
	SpiHandle.Init.NSS               = SPI_NSS_SOFT;
	SpiHandle.Init.TIMode            = SPI_TIMODE_DISABLE;

	SpiHandle.Init.Mode = SPI_MODE_MASTER;

	if(HAL_SPI_Init(&SpiHandle) != HAL_OK)
	{
		Error_Handler();
	}
}


void SetPinOutput(GPIO_TypeDef* GPIOx, uint16_t pin)
{
	  GPIO_InitTypeDef  GPIO_InitStruct;

	  /* Enable the GPIO_LED Clock */
//	  LEDx_GPIO_CLK_ENABLE(Led);
	  __HAL_RCC_GPIOA_CLK_ENABLE();
	  __HAL_RCC_GPIOB_CLK_ENABLE();
	  __HAL_RCC_GPIOC_CLK_ENABLE();

	  /* Configure the GPIO_LED pin */
	  GPIO_InitStruct.Pin = pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_PULLUP;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;

	  HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void SetPinInputPullup(GPIO_TypeDef* GPIOx, uint16_t pin)
{

	  GPIO_InitTypeDef  GPIO_InitStruct;

	  /* Configure the GPIO_LED pin */
	  GPIO_InitStruct.Pin = pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_PULLUP;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;

	  HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}


TIM_HandleTypeDef TimHandle = {0};
uint32_t uwPrescalerValue = 65535;
uint32_t timperiod = 10000; //1 tick is 0.01us

void InitTimer()
{
	uwPrescalerValue = 0;//(uint32_t) ((SystemCoreClock) / 50000000) - 1;
	//SystemCoreClock ->1mHz

  /* Set TIMx instance */
  TimHandle.Instance = MTIMx; //Tim
  TimHandle.Init.Period = 4999;
  TimHandle.Init.Prescaler = 83;
  TimHandle.Init.ClockDivision = 0;
  TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
  if(HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
  {
	/* Initialization Error */
	Error_Handler();
  }

  /*##-2- Start the TIM Base generation in interrupt mode ####################*/
  /* Start Channel1 */
  if(HAL_TIM_Base_Start_IT(&TimHandle) != HAL_OK)
  {
	/* Starting Error */
	Error_Handler();
  }
}


void DrawGraphPrec(int8_t* pvalues, int16_t offset)
{
	st7565_drawlinemulti(0,63-offset,128,63-offset,1);

	for(uint8_t i=0;i<128;i+=10)
	{
		st7565_setpixelmulti(i, 63-offset, 2);
	}


	for(uint8_t i=1;i<128;i++)
	{
		if(pvalues[i]!=127 && pvalues[i-1]!=127)
		{
			st7565_drawlinemulti(i, 63-offset- (int16_t)pvalues[i],i-1, 63-offset-(int16_t)pvalues[i-1], 2);
//			st7565_setpixelmulti(i, 63-offset-pvalues[i], 2);
		}
	}
}

int8_t current[128]={0};
int8_t volume[128]= {0};
int8_t pressure[128] = {0};
int8_t flow[128]={0};

uint16_t currid;
int countertim=0;

uint16_t timer_frame=0;

uint8_t IsDown(uint16_t btn)
{
	return (uint8_t)HAL_GPIO_ReadPin(BTN_PORT,  btn);
}

uint16_t btnpinmap[5]={BTN_PIN_ENTER, BTN_PIN_RIGHT, BTN_PIN_LEFT, BTN_PIN_UP, BTN_PIN_DOWN};
uint8_t keystate[5]={0,0,0,0,0};
uint8_t keypressed[5]={0,0,0,0,0};

void ManageKeyboard()
{
	uint8_t newkeystate[5];
	for(int i=0;i<5;i++)
	{
		newkeystate[i]=IsDown(btnpinmap[i]);
	}

	for(int i=0;i<5;i++)
	{
		if(newkeystate[i]&& keystate[i]==0)
		{
			keypressed[i]=1;
			PiezoOn();
		}
		else keypressed[i]=0;
	}

	for(int i=0;i<5;i++)
	{
		keystate[i]=newkeystate[i];
	}
}

void DrawMenu();
void UpdateServoPWM();

void Render()
{

	if(piezo_alarm==0)PiezoOff();
	ManageKeyboard();

	if(countertim<400)
	{
		st7565_drawstringmulti(0,0,"emergency-ventilator",2);
		st7565_drawstringmulti(0,1,"open source MIT license",2);
		st7565_drawstringmulti(0,3,"youtube.com/alextherelentlessengineer",2);
		st7565_drawstringmulti(0,7,"v1.0",2);

		return;
	}

	st7565_drawlinemulti(currid,0,currid, 64,2);
	DrawGraphPrec(volume,8);
	DrawGraphPrec(pressure,28);
	DrawGraphPrec(flow,40);
	DrawGraphPrec(current,52);

	DrawMenu();
}




//This timer is running at ?? HZ
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  timer_frame++;

//  HAL_GPIO_TogglePin(PIEZO_PORT, PIEZO_PIN_A);
//  HAL_GPIO_TogglePin(PIEZO_PORT, PIEZO_PIN_B);

  UpdateServoPWM();

  if(piezo_alarm)PiezoOn();

  if(countertim%2==0)
  {
	  Render();
  }


  st7565_display_multi(countertim%2);

  if(countertim%2==1)st7565_clear_multi();

 // HAL_GPIO_WritePin(PIEZO_PORT, PIEZO_PIN_B, 1);

  countertim++;
}


/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{

  /* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, instruction and Data caches
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Global MSP (MCU Support Package) initialization
     */
  HAL_Init();
  
  /* Configure LED3, LED4, LED5 and LED6 */
//  BSP_LED_Init(LED3);
//  BSP_LED_Init(LED4);
// BSP_LED_Init(LED5);
//  BSP_LED_Init(LED6);

  /* Configure the system clock to 168 MHz */
  SystemClock_Config();

  InitParams();

  InitTimerPWM();
  InitADC();

  /*##-1- Configure the UART peripheral ######################################*/
  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
  /* UART1 configured as follow:
      - Word Length = 8 Bits
      - Stop Bit = One Stop bit
      - Parity = None
      - BaudRate = 9600 baud
      - Hardware flow control disabled (RTS and CTS signals) */
  UartHandle.Instance          = USARTx;
  
  UartHandle.Init.BaudRate     = 250000;
  UartHandle.Init.WordLength   = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits     = UART_STOPBITS_1;
  UartHandle.Init.Parity       = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode         = UART_MODE_TX_RX;
  UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
    
  if(HAL_UART_Init(&UartHandle) != HAL_OK)
  {
    Error_Handler();
  }
  
  /* Configure KEY Button */
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_GPIO);
	

  /* The board sends the message and expects to receive it back */
  
//  __HAL_RCC_GPIOB_CLK_ENABLE();
//  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

  SetPinOutput(LCD_PORT, LCD_PIN_CS);
  SetPinOutput(LCD_PORT, LCD_PIN_A0);

	HAL_GPIO_WritePin(LCD_PORT, LCD_PIN_A0, 1);
	HAL_GPIO_WritePin(LCD_PORT, LCD_PIN_CS, 1);

	InitSPI();

//	InitI2C();

	st7565_init();
	st7565_clear_display();

	SetPinInputPullup(BTN_PORT, BTN_PIN_ENTER);
	SetPinInputPullup(BTN_PORT, BTN_PIN_UP);
	SetPinInputPullup(BTN_PORT, BTN_PIN_DOWN);
	SetPinInputPullup(BTN_PORT, BTN_PIN_LEFT);
	SetPinInputPullup(BTN_PORT, BTN_PIN_RIGHT);

	InitTimerPWMPiezo();
	InitTimer();


	//	HAL_GPIO_WritePin(LCD_PORT, LCD_PIN_CS, 1);

//	while(1)
//	{
/*		HAL_GPIO_WritePin(LCD_PORT, LCD_PIN_CS, GPIO_PIN_RESET);	//Select the chip
		HAL_GPIO_WritePin(LCD_PORT, LCD_PIN_A0, GPIO_PIN_SET);		//Set it in data mode

		uint8_t cmdz[10] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
		HAL_SPI_Transmit(&SpiHandle, (uint8_t*)cmdz, 10, 1000);

		HAL_GPIO_WritePin(LCD_PORT, LCD_PIN_CS, GPIO_PIN_SET);	//Select the chip

		HAL_Delay(10);*/
//	}

/*	{
	  uint8_t vreg[2] = {0x0b,0x00};
	  HAL_I2C_Master_Transmit(&I2cHandle, 0x6C, vreg, 2, 50);

	  uint8_t msg[5] = {0x00, 0xD0, 0x40, 0x18, 0x06};
	  HAL_I2C_Master_Transmit(&I2cHandle, 0x6C, msg, 5, 50);

	  uint8_t set2mode[4] = {0x00, 0xD0, 0x51, 0x2C};
	  HAL_I2C_Master_Transmit(&I2cHandle, 0x6C, set2mode, 4, 50);
//	uint8_t commands[4] = {0x00, 0x00, 0x00, 0x00};
//	  HAL_I2C_Master_Transmit(&I2cHandle, 0x6C, commands, 4, 50);
	}*/


  while(1)
  {

//	  uint16_t timerperiod=65536-counter*5;
//	  if(counter%5==0)

/*	  if(counter%16==0)
	  {

		  st7565_clear();
	  }

	  st7565_drawline(32+counter%16,0,128+32+counter%16,64,1);

	  st7565_drawline( (counter%16)*2,0, (counter%16)*2,64,1);
	  st7565_drawline( (counter%16)*2+1,0, (counter%16)*2+1,64,1);

	  st7565_display();*/

	//  st7565_setpixel(4,0,1);
//	  HAL_Delay(1);

/*	  uint8_t reg = 0x07;
	  HAL_I2C_Master_Transmit(&I2cHandle, 0x6C, &reg, 1, 50);
	  if(HAL_I2C_Master_Receive(&I2cHandle, 0x6C, (uint8_t *)aRxBuffer, 2, 50) != HAL_OK)
	  {
	    if (HAL_I2C_GetError(&I2cHandle) != HAL_I2C_ERROR_AF)
	    {
	      Error_Handler();
	    }
	  }*/

/*

		counter++;

		pos_target = pos_min;
		if(trigger>0)
		{
			pos_target=pos_max;
			timeout=0;
		}
		trigger--;

		timeout++;
		if(timeout>((3000*9)/(int16_t)params[1])) //->4000 is 9 sec.
		{
			trigger=1000;
		}

		if(pos_curr<pos_target)pos_curr+=1;
		if(pos_curr>pos_target)pos_curr-=1;

		TimHandlePWM.Instance->CCR1 = 2800-pos_curr*((int16_t)params[2])/40-50;
		TimHandlePWM.Instance->CCR2 = 2800-pos_curr*((int16_t)params[2])/40;
		TimHandlePWM.Instance->CCR3 = 2800-pos_curr*((int16_t)params[2])/40-200;

		uint32_t pos_in_buf = ADCSAMPLECOUNT*4 - (uint32_t)__HAL_DMA_GET_COUNTER(AdcHandle.DMA_Handle);
		pos_in_buf/=4;	//there are 3 values interleaved. Just read the first one.
		pos_in_buf*=4;	//Round up to the 3rd closest.

		uint32_t ya = ComputeAverage(pos_in_buf-4);
		uint32_t yb = ComputeAverage(pos_in_buf-3);
		uint32_t yc = ComputeAverage(pos_in_buf-2);
		uint32_t yw = ComputeAverage(pos_in_buf-1);

		if(yw<170 && trigger<-1500)trigger=1000;

		if(counter%32==0)
		{
			char buffer[50];
			int n=sprintf (buffer, "%i, %i, %i, %i, %i, %i, %i\n", pos_curr, ya, yb, yc, yw, (uint16_t)aRxBuffer[0], (uint16_t)aRxBuffer[1]);

			int vvolprec = pos_curr/50;
			if(vvolprec>126)vvolprec=126;
			if(vvolprec<-126)vvolprec=-126;
			currid=(counter/64)%128;
			volume[currid]=vvolprec;
			for(int i=1;i<10;i++)volume[(currid+i)%128]=127; //Delete points in advance

			int vcurrprec = (ya+yb+yc)/50;
			if(vcurrprec>126)vcurrprec=126;
			if(vcurrprec<-126)vcurrprec=-126;
			current[currid]=vcurrprec;
			for(int i=1;i<10;i++)current[(currid+i)%128]=127; //Delete points in advance

			int vpressprec = (yw)/50;
			if(vpressprec>126)vpressprec=126;
			if(vpressprec<-126)vpressprec=-126;
			pressure[currid]=vpressprec;
			for(int i=1;i<10;i++)pressure[(currid+i)%128]=127; //Delete points in advance


			if(HAL_UART_Transmit(&UartHandle, (uint8_t*)buffer, n , 5000)!= HAL_OK)
			{
				Error_Handler();
			}
		}
		HAL_Delay(1);*/

  }

  /* Infinite loop */
  while (1)
  {    
  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 168000000
  *            HCLK(Hz)                       = 168000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 336
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
//  RCC_OscInitStruct.PLL.PLLR = 2;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  /* STM32F405x/407x/415x/417x Revision Z devices: prefetch is supported  */
  if (HAL_GetREVID() == 0x1001)
  {
    /* Enable the Flash prefetch */
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
  }
}

/**
  * @brief  UART error callbacks
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
  /* Turn LED3 on: Transfer error in reception/transmission process */
  BSP_LED_On(LED3); 
}

/**
  * @brief  Compares two buffers.
  * @param  pBuffer1, pBuffer2: buffers to be compared.
  * @param  BufferLength: buffer's length
  * @retval 0  : pBuffer1 identical to pBuffer2
  *         >0 : pBuffer1 differs from pBuffer2
  *//*
static uint16_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength)
{
  while (BufferLength--)
  {
    if ((*pBuffer1) != *pBuffer2)
    {
      return BufferLength;
    }
    pBuffer1++;
    pBuffer2++;
  }

  return 0;
}*/

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* Turn LED5 on */
  BSP_LED_On(LED5);
  while(1)
  {
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
