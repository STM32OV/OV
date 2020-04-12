/**
  ******************************************************************************
  * @file    UART/UART_TwoBoards_ComPolling/Inc/main.h 
  * @author  MCD Application Team
  * @brief   Header for main.c module
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4_discovery.h"

#include "ST7565.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* User can use this section to tailor USARTx/UARTx instance used and associated 
   resources */
/* Definition for USARTx clock resources */
#define USARTx                           USART2
#define USARTx_CLK_ENABLE()              __HAL_RCC_USART2_CLK_ENABLE();
#define USARTx_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE() 

#define USARTx_FORCE_RESET()             __HAL_RCC_USART2_FORCE_RESET()
#define USARTx_RELEASE_RESET()           __HAL_RCC_USART2_RELEASE_RESET()

/* Definition for USARTx Pins */
#define USARTx_TX_PIN                    GPIO_PIN_2
#define USARTx_TX_GPIO_PORT              GPIOA  
#define USARTx_TX_AF                     GPIO_AF7_USART2
#define USARTx_RX_PIN                    GPIO_PIN_3
#define USARTx_RX_GPIO_PORT              GPIOA 
#define USARTx_RX_AF                     GPIO_AF7_USART2

#define PWM_GPIO_PORT 					 GPIOC
#define PWM_PIN0                         GPIO_PIN_6
#define PWM_PIN0_POS                     6
#define PWM_PIN1                         GPIO_PIN_7
#define PWM_PIN1_POS                     7
#define PWM_PIN2                         GPIO_PIN_8
#define PWM_PIN2_POS                     8

#define TIM_PWM_GPIO_CLK_ENABLE() __HAL_RCC_GPIOC_CLK_ENABLE()



#define ADCSAMPLECOUNT (1024*8)

#define ADCx                            ADC1
#define ADCx_CLK_ENABLE()               __HAL_RCC_ADC1_CLK_ENABLE()
#define DMAx_CLK_ENABLE()               __HAL_RCC_DMA2_CLK_ENABLE()
#define ADCx_CHANNEL_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOA_CLK_ENABLE()

#define ADCx_FORCE_RESET()              __HAL_RCC_ADC_FORCE_RESET()
#define ADCx_RELEASE_RESET()            __HAL_RCC_ADC_RELEASE_RESET()

/* Definition for ADCx Channel Pin */
#define ADCx_CHANNEL_PIN                GPIO_PIN_4
#define ADCy_CHANNEL_PIN                GPIO_PIN_5
#define ADCz_CHANNEL_PIN                GPIO_PIN_6
#define ADCw_CHANNEL_PIN                GPIO_PIN_7
#define ADCu_CHANNEL_PIN				GPIO_PIN_0
#define ADCv_CHANNEL_PIN				GPIO_PIN_1
#define ADCx_CHANNEL_GPIO_PORT          GPIOA

/* Definition for ADCx's Channel */
#define ADCx_CHANNEL                    ADC_CHANNEL_4
#define ADCy_CHANNEL					ADC_CHANNEL_5
#define ADCz_CHANNEL					ADC_CHANNEL_6
#define ADCw_CHANNEL					ADC_CHANNEL_7

/* Definition for ADCx's DMA */
#define ADCx_DMA_CHANNEL                DMA_CHANNEL_0
#define ADCx_DMA_STREAM                 DMA2_Stream4

/* Definition for ADCx's NVIC */
#define ADCx_DMA_IRQn                   DMA2_Stream4_IRQn
#define ADCx_DMA_IRQHandler             DMA2_Stream4_IRQHandler


#define SPIx                             SPI1
#define SPIx_CLK_ENABLE()                __HAL_RCC_SPI1_CLK_ENABLE()
#define SPIx_SCK_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOB_CLK_ENABLE()
#define SPIx_MISO_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()
#define SPIx_MOSI_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()

#define SPIx_FORCE_RESET()               __HAL_RCC_SPI2_FORCE_RESET()
#define SPIx_RELEASE_RESET()             __HAL_RCC_SPI2_RELEASE_RESET()

/* Definition for SPIx Pins */
#define SPIx_SCK_PIN                     GPIO_PIN_3
#define SPIx_SCK_GPIO_PORT               GPIOB
#define SPIx_SCK_AF                      GPIO_AF5_SPI1
#define SPIx_MISO_PIN                    GPIO_PIN_4
#define SPIx_MISO_GPIO_PORT              GPIOB
#define SPIx_MISO_AF                     GPIO_AF5_SPI1
#define SPIx_MOSI_PIN                    GPIO_PIN_5
#define SPIx_MOSI_GPIO_PORT              GPIOB
#define SPIx_MOSI_AF                     GPIO_AF5_SPI1

#define LCD_PORT						 GPIOB
#define LCD_PIN_CS 						 GPIO_PIN_7
#define LCD_PIN_A0  					 GPIO_PIN_6

#define BTN_PORT 						GPIOC
#define BTN_PIN_ENTER					GPIO_PIN_15
#define BTN_PIN_RIGHT					GPIO_PIN_2
#define BTN_PIN_LEFT 					GPIO_PIN_3
#define BTN_PIN_UP						GPIO_PIN_1
#define BTN_PIN_DOWN					GPIO_PIN_0
#define BTN_ENTER						0
#define BTN_RIGHT						1
#define BTN_LEFT						2
#define BTN_UP							3
#define BTN_DOWN						4



#define PIEZO_PORT						GPIOA
#define PIEZO_GPIO_CLK_ENABLE()    	    __HAL_RCC_GPIOA_CLK_ENABLE()
#define PIEZO_PIN_A						GPIO_PIN_0
#define PIEZO_PIN_B						GPIO_PIN_1


#define MTIMx                           TIM3
#define MTIMx_CLK_ENABLE()              __HAL_RCC_TIM3_CLK_ENABLE()
#define MTIMx_IRQn                      TIM3_IRQn
#define TIMx_IRQHandler                 TIM3_IRQHandler


#define I2Cx                             I2C2
#define I2Cx_CLK_ENABLE()                __HAL_RCC_I2C2_CLK_ENABLE()
#define I2Cx_SDA_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOB_CLK_ENABLE()
#define I2Cx_SCL_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOB_CLK_ENABLE()

#define I2Cx_FORCE_RESET()               __HAL_RCC_I2C2_FORCE_RESET()
#define I2Cx_RELEASE_RESET()             __HAL_RCC_I2C2_RELEASE_RESET()

/* Definition for I2Cx Pins */
#define I2Cx_SCL_PIN                    GPIO_PIN_10
#define I2Cx_SCL_GPIO_PORT              GPIOB
#define I2Cx_SCL_AF                     GPIO_AF4_I2C2
#define I2Cx_SDA_PIN                    GPIO_PIN_11
#define I2Cx_SDA_GPIO_PORT              GPIOB
#define I2Cx_SDA_AF                     GPIO_AF4_I2C2


#define MODEAC_TIME_INSPIRE 					1515	//Time it takes to deliver the full volume to the patient.
#define MODEAC_TIMEOUT_TRIGGER					500 	//How much time should we wait before we deliver another breath.
#define MODEAC_SERVO_K							3		//Servo K is X*0.1mL per microsecond


void DrawMenu();
void InitParams();


/* Size of Transmission buffer */
//#define TXBUFFERSIZE                      (COUNTOF(aTxBuffer) - 1)
/* Size of Reception buffer */
//#define RXBUFFERSIZE                      TXBUFFERSIZE
  
/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
/* Exported functions ------------------------------------------------------- */

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
