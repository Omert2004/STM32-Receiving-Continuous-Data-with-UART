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
#include <string.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_RAWDATA_SIZE 300
#define RX_BUFFER_SIZE 16
#define	PROCESSED_DATA_SIZE 4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
__IO uint8_t ubButtonPress = 0;
__IO uint8_t ubSend = 0;

uint8_t RxRawData[RX_RAWDATA_SIZE];
uint8_t RxBuffer[RX_BUFFER_SIZE];
uint16_t ubNbDataToReceive = sizeof(RxRawData) - 1;
uint8_t ProcessedData[PROCESSED_DATA_SIZE];
uint8_t IndexRxRawData = 0;

__IO uint8_t ubReceptionComplete = 0;
uint8_t DataUsed=0;
int a;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void StartTransfer(void);
void LED_On(void);
void LED_Blinking(uint32_t Period);
void LED_Off(void);
void UserButton_Init(void);
void WaitForUserButtonPress(void);
void ExecuteTasksAndDisableChannel(void);
void ResetFlags(void);
void Task1(uint8_t *SmallBuffer);
void LightTask(uint8_t *ProcessedData);



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* SysTick_IRQn interrupt configuration */

  /** Disable the internal Pull-Up in Dead Battery pins of UCPD peripheral
  */
  LL_SYSCFG_DisableDBATT(LL_SYSCFG_UCPD1_STROBE | LL_SYSCFG_UCPD2_STROBE);

  /* USER CODE BEGIN Init */
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* Initialize User push-button in EXTI mode */
   UserButton_Init();

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  /*Wait for User push-button to start receive data*/
  WaitForUserButtonPress();

  StartTransfer(); // Enable DMA and its RX Channel

  ExecuteTasksAndDisableChannel(); // When data is received, execute the tasks and close the channel
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	 ResetFlags();
	 // Start the recursion
	/* Configure the DMA functional parameters for reception */
	LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_2,
							   LL_USART_DMA_GetRegAddr(USART2, LL_USART_DMA_REG_DATA_RECEIVE),
							   (uint32_t)RxRawData,
							   LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2));
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, ubNbDataToReceive);

	StartTransfer();

	ExecuteTasksAndDisableChannel();


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
  /* HSI configuration and activation */
  
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);

  /* HSI configuration and activation */
  LL_RCC_HSI_Enable();
  while(LL_RCC_HSI_IsReady() != 1)
  {
  }

  /* Main PLL configuration and activation */
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_4, 70, LL_RCC_PLLR_DIV_5);
  LL_RCC_PLL_Enable();
  LL_RCC_PLL_EnableDomain_SYS();
  while(LL_RCC_PLL_IsReady() != 1)
  {
  }

  /* Set AHB prescaler*/
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  /* Sysclk activation on the main PLL */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  }

  /* Set APB1 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_Init1msTick(56000000);
  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(56000000);
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{


  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  /**USART2 GPIO Configuration
  PA2   ------> USART2_TX
  PA3   ------> USART2_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART2 DMA Init */

  /* USART2_TX Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_1, LL_DMAMUX_REQ_USART2_TX);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_HIGH);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_BYTE);

  /* USART2_RX Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_2, LL_DMAMUX_REQ_USART2_RX);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2, LL_DMA_DIRECTION_PERIPH_TO_MEMORY); //Peripheral to memory

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PRIORITY_HIGH); //priority high

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MODE_NORMAL);  //Normal DMA

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MDATAALIGN_BYTE);

  /* USART2 interrupt Init */
  NVIC_SetPriority(USART2_IRQn, 0);
  NVIC_EnableIRQ(USART2_IRQn);

  /* USER CODE BEGIN USART2_Init 1 */
  /* Configure the DMA functional parameters for transmission */
//  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_1,
//                         (uint32_t)aTxBuffer,
//                         LL_USART_DMA_GetRegAddr(USART2, LL_USART_DMA_REG_DATA_TRANSMIT),
//                         LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1));
//  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, ubNbDataToTransmit);

  /* Configure the DMA functional parameters for reception */
  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_2,
                         LL_USART_DMA_GetRegAddr(USART2, LL_USART_DMA_REG_DATA_RECEIVE),
                         (uint32_t)RxRawData,
                         LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2));
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, ubNbDataToReceive);

  /* Enable DMA transfer complete/error interrupts  */
//  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
//  LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_1);
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_2);
  LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_2);
  /* USER CODE END USART2_Init 1 */
  USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
  USART_InitStruct.BaudRate = 19200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART2,&USART_InitStruct);
  LL_USART_SetTXFIFOThreshold(USART2, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_SetRXFIFOThreshold(USART2, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_DisableFIFO(USART2);
  LL_USART_ConfigAsyncMode(USART2);

  /* USER CODE BEGIN WKUPType USART2 */
  LL_USART_EnableRxTimeout(USART2);
  LL_USART_SetRxTimeout(USART2, 960);

  LL_USART_ClearFlag_RTO(USART2);
  LL_USART_EnableIT_RTO(USART2);
  /* USER CODE END WKUPType USART2 */

//  LL_USART_Enable(USART2);

  /* Polling USART2 initialisation */
//  while((!(LL_USART_IsActiveFlag_TEACK(USART2))) || (!(LL_USART_IsActiveFlag_REACK(USART2))))
//  {
//  }
  /* USER CODE BEGIN USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
//  NVIC_SetPriority(DMA1_Channel1_IRQn, 0);
//  NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0);
  NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */


  /* GPIO Ports Clock Enable */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

  /**/
  LL_GPIO_ResetOutputPin(LED4_GPIO_Port, LED4_Pin);

  /**/
  GPIO_InitStruct.Pin = LED4_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED4_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */

}

/* USER CODE BEGIN 4 */
void UserButton_Init(void)
{
  USER_BUTTON_GPIO_CLK_ENABLE();

  /* Configure GPIO for BUTTON */
  LL_GPIO_SetPinMode(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinPull(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, LL_GPIO_PULL_NO);

  /* Connect External Line to the GPIO*/
  USER_BUTTON_SYSCFG_SET_EXTI();

  /* Enable a rising trigger EXTI_Line4_15 Interrupt */
  USER_BUTTON_EXTI_LINE_ENABLE();
  USER_BUTTON_EXTI_FALLING_TRIG_ENABLE();

  /* Configure NVIC for USER_BUTTON_EXTI_IRQn */
  NVIC_SetPriority(USER_BUTTON_EXTI_IRQn, 3);
  NVIC_EnableIRQ(USER_BUTTON_EXTI_IRQn);
}

void LED_On(void)
{
  /* Turn LED4 on */
  LL_GPIO_SetOutputPin(LED4_GPIO_Port, LED4_Pin);
}

void LED_Off(void)
{
  /* Turn LED4 off */
  LL_GPIO_ResetOutputPin(LED4_GPIO_Port, LED4_Pin);
}

void LED_Blinking(uint32_t Period)
{
  /* Toggle LED4 in an infinite loop */
  while (1)
  {
    LL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
    LL_mDelay(Period);
  }
}


void WaitForUserButtonPress(void)
{

  while (ubButtonPress == 0)
  {

    LL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
    LL_mDelay(200);
  }
  /* Ensure that LED4 is turned Off */
  LL_USART_Enable(USART2);
  while((!(LL_USART_IsActiveFlag_TEACK(USART2))) || (!(LL_USART_IsActiveFlag_REACK(USART2))))
  //  {
  //  }
  LED_Off();
}

void StartTransfer(void)
{
  /* Enable DMA RX Interrupt */
  LL_USART_EnableDMAReq_RX(USART2);

  /* Enable DMA Channel Rx */
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);

  /* Enable DMA Channel Tx */

}


void ExecuteTasksAndDisableChannel(void){
	while(ubReceptionComplete != 1)
	{
		//Task1(RxBuffer);
	}

	Task1(RxBuffer);
	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);

}

void DMA1_ReceiveComplete_Callback(void)
{
	ubReceptionComplete=1;
}

void UserButton_Callback(void)
{
  /* Update User push-button variable : to be checked in waiting loop in main program */
	ubButtonPress = 1;
}

void ResetFlags(void){

	/* Buffer used for reception */
	ubReceptionComplete = 0;

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void USART_TransferError_Callback(void)
{
  /* Disable DMA1 Tx Channel */
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);

  /* Disable DMA1 Rx Channel */
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);

  /* Set LED4 to Blinking mode to indicate error occurs */
  LED_Blinking(500);
}
void USART2_IRQHandler(void)
{

	uint32_t isrflags=READ_REG(USART2->ISR);
	uint32_t errorflags=( isrflags & (uint32_t)(USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE) );

  /* USER CODE BEGIN USART2_IRQn 0 */
	if (LL_USART_IsActiveFlag_RTO(USART2)) //Check if Rx Timeou(RTO) flag is set.
	    {
	        // Clear the Rx Timeout flag
	        LL_USART_ClearFlag_RTO(USART2);
//	     if ( DataUsed== 1){
//	    	 memset(RxBuffer,0,sizeof(RxBuffer));
//	    	 DataUsed=0;
//	     }
	    	 memcpy(RxBuffer, RxRawData,sizeof(RxBuffer)); // Keep the input with RxBuffer
			 IndexRxRawData= 0; // Reset the RawData index
			 memset(RxRawData, 0, sizeof(RxRawData)); // Clear the RxRawData to get new input starting from index 0.
			 ubReceptionComplete = 1;
	      //Calls the interrupt then restarts DMA.
	    }

if( errorflags!=0 ){
	if( (isrflags & USART_ISR_PE)!=0 ){
		LL_USART_ClearFlag_PE(USART2);
	}
	if( (isrflags & USART_ISR_FE)!=0 ){
		LL_USART_ClearFlag_FE(USART2);
	}
	if( (isrflags & USART_ISR_NE)!=0 ){
		LL_USART_ClearFlag_NE(USART2);
	}
	if( (isrflags & USART_ISR_ORE)!=0 ){
		LL_USART_ClearFlag_ORE(USART2);
	}
}
//	if (LL_USART_IsActiveFlag_RXNE(USART2))
//	{
//		RxRawData[IndexRxRawData++] == LL_USART_ReceiveData8(USART2); //Store the received data in RxRawData
//
//
//	}
}

void Task1(uint8_t *SmallBuffer){ //

	for(int i=0; i<4;i++){ // Store the processed Data from RxBuffer
		ProcessedData[i]= SmallBuffer[i+6];
	}
	LightTask(ProcessedData);
	memset(ProcessedData,0, sizeof(ProcessedData)); // REset the ProcessedDAta
	DataUsed=1; //Raise Flag when the Task1 is done.
}
void LightTask(uint8_t *ProcessedData){
	int i, k = 0;
	int ProcessedData_Size= sizeof(ProcessedData);
	for (i = 0; i < ProcessedData_Size; i++){ // Convert the processed data to whole integer number
		k = 10 * k + (ProcessedData[i] - '0'); // Turn the character into integer by substracting '0'
	}
	a=k;
	if ( k > 1330 && k < 1340){ // Blink LED, if the proccessed integer is at between desired borders
		LED_On();
	}
	else{
		LED_Off();
	}

}
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
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
