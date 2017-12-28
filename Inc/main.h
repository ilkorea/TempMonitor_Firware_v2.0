/**
  ******************************************************************************
  * File Name          : main.hpp
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define nLED_EN2_Pin GPIO_PIN_2
#define nLED_EN2_GPIO_Port GPIOE
#define nLED_EN3_Pin GPIO_PIN_3
#define nLED_EN3_GPIO_Port GPIOE
#define nLED_EN4_Pin GPIO_PIN_4
#define nLED_EN4_GPIO_Port GPIOE
#define nLED_EN5_Pin GPIO_PIN_5
#define nLED_EN5_GPIO_Port GPIOE
#define nLED_EN6_Pin GPIO_PIN_6
#define nLED_EN6_GPIO_Port GPIOE
#define ADC_RTD_Pin GPIO_PIN_0
#define ADC_RTD_GPIO_Port GPIOC
#define NTC_ADC0_Pin GPIO_PIN_0
#define NTC_ADC0_GPIO_Port GPIOA
#define NTC_ADC1_Pin GPIO_PIN_1
#define NTC_ADC1_GPIO_Port GPIOA
#define NTC_ADC2_Pin GPIO_PIN_2
#define NTC_ADC2_GPIO_Port GPIOA
#define NTC_ADC3_Pin GPIO_PIN_3
#define NTC_ADC3_GPIO_Port GPIOA
#define nCS_ADD0_Pin GPIO_PIN_4
#define nCS_ADD0_GPIO_Port GPIOA
#define nCS_ADD1_Pin GPIO_PIN_5
#define nCS_ADD1_GPIO_Port GPIOA
#define nCS_ADD2_Pin GPIO_PIN_6
#define nCS_ADD2_GPIO_Port GPIOA
#define nCS_ADD3_Pin GPIO_PIN_7
#define nCS_ADD3_GPIO_Port GPIOA
#define POWER_STATE_Pin GPIO_PIN_4
#define POWER_STATE_GPIO_Port GPIOC
#define SD_STATE_Pin GPIO_PIN_5
#define SD_STATE_GPIO_Port GPIOC
#define BD_HUM_Pin GPIO_PIN_0
#define BD_HUM_GPIO_Port GPIOB
#define BD_TEMP_Pin GPIO_PIN_1
#define BD_TEMP_GPIO_Port GPIOB
#define nLED_EN7_Pin GPIO_PIN_7
#define nLED_EN7_GPIO_Port GPIOE
#define nLATCH_DATA_Pin GPIO_PIN_8
#define nLATCH_DATA_GPIO_Port GPIOE
#define nRELAY_EN0_Pin GPIO_PIN_9
#define nRELAY_EN0_GPIO_Port GPIOE
#define nRELAY_EN1_Pin GPIO_PIN_10
#define nRELAY_EN1_GPIO_Port GPIOE
#define nRELAY_EN2_Pin GPIO_PIN_11
#define nRELAY_EN2_GPIO_Port GPIOE
#define nRELAY_EN3_Pin GPIO_PIN_12
#define nRELAY_EN3_GPIO_Port GPIOE
#define ETH_RESET_Pin GPIO_PIN_13
#define ETH_RESET_GPIO_Port GPIOE
#define ETH_INTn_Pin GPIO_PIN_15
#define ETH_INTn_GPIO_Port GPIOE
#define in_LE1_Pin GPIO_PIN_10
#define in_LE1_GPIO_Port GPIOB
#define ETH_SCSn_Pin GPIO_PIN_12
#define ETH_SCSn_GPIO_Port GPIOB
#define ETH_SCLK_Pin GPIO_PIN_13
#define ETH_SCLK_GPIO_Port GPIOB
#define ETH_MISO_Pin GPIO_PIN_14
#define ETH_MISO_GPIO_Port GPIOB
#define ETH_MOSI_Pin GPIO_PIN_15
#define ETH_MOSI_GPIO_Port GPIOB
#define RS485_TX_Pin GPIO_PIN_8
#define RS485_TX_GPIO_Port GPIOD
#define RS485_RX_Pin GPIO_PIN_9
#define RS485_RX_GPIO_Port GPIOD
#define RS485_EN_Pin GPIO_PIN_10
#define RS485_EN_GPIO_Port GPIOD
#define nNTC_MUXEN0_Pin GPIO_PIN_12
#define nNTC_MUXEN0_GPIO_Port GPIOD
#define nNTC_MUXEN1_Pin GPIO_PIN_13
#define nNTC_MUXEN1_GPIO_Port GPIOD
#define nNTC_MUXEN2_Pin GPIO_PIN_14
#define nNTC_MUXEN2_GPIO_Port GPIOD
#define nNTC_MUXEN3_Pin GPIO_PIN_15
#define nNTC_MUXEN3_GPIO_Port GPIOD
#define SDIO_POWER_Pin GPIO_PIN_6
#define SDIO_POWER_GPIO_Port GPIOC
#define EXTI7_SDIO_DETECT_Pin GPIO_PIN_7
#define EXTI7_SDIO_DETECT_GPIO_Port GPIOC
#define SDIO_D0_Pin GPIO_PIN_8
#define SDIO_D0_GPIO_Port GPIOC
#define SDIO_D1_Pin GPIO_PIN_9
#define SDIO_D1_GPIO_Port GPIOC
#define USB_TX_Pin GPIO_PIN_9
#define USB_TX_GPIO_Port GPIOA
#define USB_RX_Pin GPIO_PIN_10
#define USB_RX_GPIO_Port GPIOA
#define SDIO_D2_Pin GPIO_PIN_10
#define SDIO_D2_GPIO_Port GPIOC
#define SDIO_D3_Pin GPIO_PIN_11
#define SDIO_D3_GPIO_Port GPIOC
#define SDIO_CK_Pin GPIO_PIN_12
#define SDIO_CK_GPIO_Port GPIOC
#define FACTORY_RESET_Pin GPIO_PIN_0
#define FACTORY_RESET_GPIO_Port GPIOD
#define BT_MODE_Pin GPIO_PIN_1
#define BT_MODE_GPIO_Port GPIOD
#define SDIO_CMD_Pin GPIO_PIN_2
#define SDIO_CMD_GPIO_Port GPIOD
#define BT_TX_Pin GPIO_PIN_5
#define BT_TX_GPIO_Port GPIOD
#define BT_RX_Pin GPIO_PIN_6
#define BT_RX_GPIO_Port GPIOD
#define FND0_Pin GPIO_PIN_5
#define FND0_GPIO_Port GPIOB
#define FND1_Pin GPIO_PIN_6
#define FND1_GPIO_Port GPIOB
#define FND2_Pin GPIO_PIN_7
#define FND2_GPIO_Port GPIOB
#define FND3_Pin GPIO_PIN_8
#define FND3_GPIO_Port GPIOB
#define in_LE0_Pin GPIO_PIN_9
#define in_LE0_GPIO_Port GPIOB
#define nLED_EN0_Pin GPIO_PIN_0
#define nLED_EN0_GPIO_Port GPIOE
#define nLED_EN1_Pin GPIO_PIN_1
#define nLED_EN1_GPIO_Port GPIOE

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
