/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#include "string.h"
#include "stdio.h"
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart2;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim4;
/* USER CODE BEGIN EV */
extern volatile enum stato_corrente;
extern unsigned soglia_500ms;
extern unsigned soglia_200ms;
extern unsigned soglia_350ms;
float valore_sensore_T; // valore sensore di temperatura
float temperatura;
int prima_scansione = 0; // flagdi prima scansione
float V25 = 0.76; // valore della tensione dell adc1 a 25 °C
float avg_slope = 2.5; // dato da datasheet
char buffer_temperatura[3]; // buffer in cui inserire la stringa relativa al valore di temperatura
float valore_sensore_V; // valore di tensione
float tensione;
float Vdd = 3.6; // tensione di alimentazione adc da datasheet
int flag_tensione = 0;
int flag_temperatura = 0;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles ADC1 global interrupt.
  */
void ADC_IRQHandler(void)
{
  /* USER CODE BEGIN ADC_IRQn 0 */

	if (flag_temperatura == 0 || flag_tensione == 0){
	if (flag_temperatura == 1){ // se ha finito la misura di temperatura
		valore_sensore_T = LL_ADC_ReadReg(ADC1, DR); // assegno ad una varibile il valore della conversione
		temperatura = ((valore_sensore_T - V25)/avg_slope) + 25 ;// calcolo la temperatura attraverso l'equazione data nel datasheet
		temperatura -= 26; // correzione dell offset del sensore di temperatura
		LL_ADC_WriteReg(ADC1,SR, LL_ADC_ReadReg(ADC1,SR) & ~0b10); // tolgo flag conversione
		flag_temperatura = 0; // segnalo che la scansione di temperatura è avvenuta


		if (temperatura > -40 && temperatura < 125){  // se il valore di temperatura entra nel range dato dal datasheet
			HAL_UART_Transmit_IT(&huart2,"temperatura = ", sizeof(("temperatura = " )));
			//sprintf(buffer_temperatura, "%1.f", temperatura); // salvo in buffer il valore della temperatura in char
			//HAL_UART_Transmit_IT(&huart2, buffer_temperatura, sizeof(buffer_temperatura), 1);
			HAL_UART_Transmit_IT(&huart2,"\n", sizeof(("\n" )));
		}

		else {
			HAL_UART_Transmit_IT(&huart2,"errore : temperatura out of range", sizeof(("errore : temperatura out of range")));
		}
	}




	 if (flag_tensione == 1){  // se ha finito la misura di tensione

		LL_ADC_WriteReg(ADC1,SR, LL_ADC_ReadReg(ADC1,SR) & ~0b10); // tolgo flag conversione
		flag_tensione = 0;                           // azzero flag prima scansione
		valore_sensore_V =  LL_ADC_ReadReg(ADC1, DR); // assegno ad una varibile il valore della conversione
		tensione = (Vdd/256) * valore_sensore_V; // calcolo la tensione

		if (tensione < 1.8){
			stato_corrente = danger_state;
			LL_GPIO_WriteReg(GPIOA, ODR, LL_GPIO_ReadReg(GPIOA, ODR) | 0b100000); // accendo GPIO PA5 collegato al led verde sulla board

		}

		else if (tensione > 2.7){
			stato_corrente = danger_state;
			LL_GPIO_WriteReg(GPIOC, ODR, LL_GPIO_ReadReg(GPIOA, ODR) | 0b100000); // accendo GPIO PC5 collegato al led rosso sulla breadboard


		}

		else {
			stato_corrente = running_state;
			LL_GPIO_WriteReg(GPIOC, ODR, LL_GPIO_ReadReg(GPIOA, ODR) & ~0b100000); // spengo GPIO PC5 collegato al led rosso sulla breadboard
			LL_GPIO_WriteReg(GPIOA, ODR, LL_GPIO_ReadReg(GPIOA, ODR) & ~0b100000); // spengo GPIO PA5 collegato al led verde sulla board

		}




	}
	}

	else {
		flag_temperatura = 0;
		flag_tensione = 0;
	}
  /* USER CODE END ADC_IRQn 0 */

  /* USER CODE BEGIN ADC_IRQn 1 */

  /* USER CODE END ADC_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

	if (stato_corrente == running_state){ // se sono in stato run

		if ((LL_TIM_ReadReg(TIM3,SR) & 0b1000) == 0b1000) {  // se flag ch3 200ms è alzata (temperatura)

			LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_18); // scelgo canale 18 (temperatura)
			LL_ADC_WriteReg(ADC1, CR2,LL_ADC_ReadReg(ADC1, CR2) | 0x40000000); // start adc (bit 30)
			LL_TIM_WriteReg(TIM3, SR, (LL_TIM_ReadReg(TIM3,SR) & ~0b1000)); // resetto flag
			flag_temperatura = 1;
			LL_TIM_WriteReg(TIM3, CCR3, (LL_TIM_ReadReg(TIM3,CCR3) + soglia_200ms)); // aggiorno soglia di ch3 200ms

		}
	}



 	 if (stato_corrente == running_state || stato_corrente == danger_state){
		if ((LL_TIM_ReadReg(TIM3,SR) & 0b100) == 0b100){ // se flag ch2 350ms è alzata (tensione)

			LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_0); // scelgo canale 0 (tensione)
			flag_tensione = 1;
			LL_ADC_WriteReg(ADC1, CR2,LL_ADC_ReadReg(ADC1, CR2) | 0x40000000); // start adc (bit 30)
			LL_TIM_WriteReg(TIM3, SR, (LL_TIM_ReadReg(TIM3,SR) & ~0b100)); // resetto flag
			LL_TIM_WriteReg(TIM3, CCR2, (LL_TIM_ReadReg(TIM3,CCR2) + soglia_350ms)); // aggiorno soglia di ch2 350ms


		}
	}



	else if (((LL_TIM_ReadReg(TIM3,SR) & 0b10) == 0b10) && stato_corrente == waiting_state){   //se flag ch1 500ms è alzata e sono in stato wait

		  LL_TIM_WriteReg(TIM3, SR, (LL_TIM_ReadReg(TIM3,SR) & ~0b10)); // resetto flag
		  HAL_UART_Transmit_IT(&huart2,"Board in waiting state - please press the emergency button", sizeof("Board in waiting state - please press the emergency button"));
		  LL_TIM_WriteReg(TIM3, CCR1, (LL_TIM_ReadReg(TIM3,CCR1) + soglia_500ms)); // aggiorno soglia di ch1 500ms
			  		  }
  /* USER CODE END TIM3_IRQn 0 */
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
