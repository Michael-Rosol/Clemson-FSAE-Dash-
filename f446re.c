/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN PV */
CAN_HandleTypeDef hcan1;
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint32_t TxMailbox1[4];
uint32_t TxMailbox;
uint8_t RxData[8];
uint8_t TxData[8];

/*
 * Okay, I did not realize that both of the boards did
 * not have external crystals. So the best course of action would be to
 * configure the clock of the F446RE as HSI.
 * Then for the H723 I should configure the ST-Link V3 just like your guide
 * and then use Bypass clock source.
 * Configure the GPIO pins if you want 7-segment!
 */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


uint8_t temp1, temp2, temp3, temp4;

#define D1_HIGH() HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, GPIO_PIN_SET)
#define D1_LOW() HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, GPIO_PIN_RESET)
#define D2_HIGH() HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, GPIO_PIN_SET)
#define D2_LOW() HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, GPIO_PIN_RESET)
#define D3_HIGH() HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, GPIO_PIN_SET)
#define D3_LOW() HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, GPIO_PIN_RESET)
#define D4_HIGH() HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, GPIO_PIN_SET)
#define D4_LOW() HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, GPIO_PIN_RESET)
#define DP_ON() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET)  // Turn on decimal point
#define DP_OFF() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET)  // Turn off decimal point

uint32_t counter = 10; // Start at 10.5



uint8_t segmentNumber[10] = {
        0x3f,  // 0
        0x06,  // 1
        0x5b,  // 2
        0x4f,  // 3
        0x66,  // 4
        0x6d,  // 5
        0x7d,  // 6
        0x07,  // 7
        0x7f,  // 8
        0x67   // 9
};
/*
 * Configure the Correct Digits For the 7-Segment
 *
 *  */
void SevenSegment_Update(uint8_t number){
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, ((number>>0)&0x01)); // a
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, ((number>>1)&0x01)); // b
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, ((number>>2)&0x01)); // c
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, ((number>>3)&0x01)); // d
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, ((number>>4)&0x01)); // e
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, ((number>>5)&0x01)); // f
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, ((number>>6)&0x01)); // g

}

//void DisplayTxData(uint32_t value) {
//    // Break the value into individual digits
//    uint8_t temp1 = (value / 1000) % 10; // Thousands place
//    uint8_t temp2 = (value / 100) % 10;  // Hundreds place
//    uint8_t temp3 = (value / 10) % 10;   // Tens place
//    uint8_t temp4 = value % 10;          // Units place
//
//
//    if (value < 10){
//        D4_LOW();  // Activate D1
//        D3_HIGH();
//        D2_HIGH();
//        D1_HIGH();
//    } else if (value < 100){
//        D4_LOW();
//        D3_LOW();  // Activate D2
//        D2_HIGH();
//        D1_HIGH();
//    }
//    else if (value < 1000){
//        D4_LOW();
//         D3_LOW();  // Activate D2
//         D2_LOW();
//         D1_HIGH();
//    }
//    else{
//        D4_LOW();
//         D3_LOW();  // Activate D2
//         D2_LOW();
//         D1_LOW();
//    }
//



void DisplayTxData(uint32_t value) {
    // Extract individual digits
    uint8_t temp1 = (value / 1000) % 10; // Thousands place
    uint8_t temp2 = (value / 100) % 10;  // Hundreds place
    uint8_t temp3 = (value / 10) % 10;   // Tens place
    uint8_t temp4 = value % 10;          // Units place

    // Cycle through each digit
    //for (int i = 0; i < 100; i++) { // 100 iterations to display for ~500ms (adjust as needed)

        // Display thousands digit
        SevenSegment_Update(segmentNumber[temp1]);
        D1_LOW();  // Activate D1
        D2_HIGH();
        D3_HIGH();
        D4_HIGH();
        HAL_Delay(1); // Short delay for this digit
        D1_HIGH(); // Deactivate D1

        // Display hundreds digit
        SevenSegment_Update(segmentNumber[temp2]);
        D2_LOW();  // Activate D2
        D1_HIGH();
        D3_HIGH();
        D4_HIGH();
        HAL_Delay(1); // Short delay for this digit
        D2_HIGH(); // Deactivate D2

        // Display tens digit
        SevenSegment_Update(segmentNumber[temp3]);
        D3_LOW();  // Activate D3
        D1_HIGH();
        D2_HIGH();
        D4_HIGH();
        HAL_Delay(1); // Short delay for this digit
        D3_HIGH(); // Deactivate D3

        // Display units digit
        SevenSegment_Update(segmentNumber[temp4]);
        D4_LOW();  // Activate D4
        D1_HIGH();
        D2_HIGH();
        D3_HIGH();
        HAL_Delay(1); // Short delay for this digit
        D4_HIGH(); // Deactivate D4


    //}
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */




  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET); // Turn on decimal point
//
//	    // Display the number 5 on the 7-segment display
//	    SevenSegment_Update(segmentNumber[8]);
//	    D1_LOW();  // Activate the display for the single digit
//	    D2_HIGH();
//	    D3_HIGH(); // Change because this can cause problems
//	    D4_HIGH();
//
//	    HAL_Delay(100);


	  TxData[0] = 0x23;
      TxData[1] = 0x44;
      TxData[2] = 0x69;
//	  	  // Test code to flash LED
	  	  //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	      //HAL_Delay(300); // Blink every 500 ms

//	  if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, &TxData[0], &TxMailbox[0]) != HAL_OK)
//	  {
//	     Error_Handler ();
//	  }
//
//	  HAL_Delay(1000);





	 if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
		 //uint32_t can_error = HAL_CAN_GetError(&hcan1); // Can potentially use for debugging

		 Error_Handler();
	 }

	 for (int i = 0; i < 10; i++){
		 DisplayTxData(TxData[2]);
	 }



	   HAL_Delay(500);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */


	hcan1.Instance = CAN1;
	  hcan1.Init.Prescaler = 9;
	  hcan1.Init.Mode = CAN_MODE_NORMAL;
	  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	  hcan1.Init.TimeSeg1 = CAN_BS1_7TQ;
	  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
	  hcan1.Init.TimeTriggeredMode = DISABLE;
	  hcan1.Init.AutoBusOff = DISABLE;
	  hcan1.Init.AutoWakeUp = DISABLE;
	  hcan1.Init.AutoRetransmission = DISABLE;
	  hcan1.Init.ReceiveFifoLocked = DISABLE;
	  hcan1.Init.TransmitFifoPriority = DISABLE;
	  if (HAL_CAN_Init(&hcan1) != HAL_OK)
	  {
	    Error_Handler();
	  }
  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */
	  CAN_FilterTypeDef  sFilterConfig;


	  sFilterConfig.FilterBank = 1;
	  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	  sFilterConfig.FilterIdHigh = 0x0000;
	  sFilterConfig.FilterIdLow = 0x0000;
	  sFilterConfig.FilterMaskIdHigh = 0x0000;
	  sFilterConfig.FilterMaskIdLow = 0x0000;
	  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	  sFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
	  sFilterConfig.SlaveStartFilterBank = 14;

	  if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
	  {
	    /* Filter configuration Error */
	    Error_Handler();
	  }

	  /*##-3- Start the CAN peripheral ###########################################*/
	  if (HAL_CAN_Start(&hcan1) != HAL_OK)
	  {
	    /* Start Error */
	    Error_Handler();
	  }

	  /*##-4- Activate CAN RX notification #######################################*/
	  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	  {
	    /* Notification Error */
	    Error_Handler();
	  }

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 9;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_7TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
	        TxHeader.StdId = 0x0446;  // ID 2 (to match H7's filter)
	    	TxHeader.ExtId = 0x01; // may need depending on the size of the bits being sent from ecu
	        TxHeader.IDE = CAN_ID_STD;  // Standard ID
	        TxHeader.RTR = CAN_RTR_DATA;  // Data frame
	        TxHeader.DLC = 3;  // Length of data (3 bytes)
	        TxHeader.TransmitGlobalTime = DISABLE;

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_0|GPIO_PIN_1
                          |D4_Pin|GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_5|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, D1_Pin|D2_Pin|D3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC0 PC1
                           D4_Pin PC6 PC8 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_0|GPIO_PIN_1
                          |D4_Pin|GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA5 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_5|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : D1_Pin D2_Pin D3_Pin */
  GPIO_InitStruct.Pin = D1_Pin|D2_Pin|D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  /* Get RX message */
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
  {
    /* Reception Error */
    Error_Handler();
  }

}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
