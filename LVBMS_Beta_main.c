/*
                                            			 	 ~| LVBMS |~
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   Name: Johnathan Dekel                                                                          last time updated: 29/10/20

   LVBMS_Beta_main.c


  Table of Contents:

  1.  Headline
  2.  main
  3.  System Clock Configurations
  4.  GPIO Initialization
  5.  ADC1 Initialization
  6.  ADC2 Initialization
  7.  ADC Data Receiving
  8.  Sample-Temperature Converter
  9.  Sample-Volt Converter
  10.  UART Initialization
  11.  CAN Initialization
  12.  CAN Filter Configurations
  13. CAN Message Transmitting
  14. CAN Message Receiving
  15. Interrupt Management
  16. Error Handler

                                                                                                                           */
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 1.                                                     | Headline |
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#include "main.h"
#include <string.h>
#include <stdio.h>

/* Private function prototypes -----------------------------------------------*/

void SystemClock_Config(void);
void BGR_GPIO_Init(void);
void BGR_ADC1_Init(void);
void BGR_ADC2_Init(void);
int BGR_ADC_Rx(uint8_t ADC);
int BGR_Samp_To_Temp(uint16_t ADC_DATA_TEMP);
int BGR_Samp_To_Volt(uint16_t ADC_DATA_VOLT);
void BGR_USART2_UART_Init(void);
void BGR_CAN_Init(void);
void BGR_CAN_Filter_Config(void);
void BGR_CAN_Tx(uint8_t node);
void BGR_CAN_Rx(void);
void BGR_Error_Handler(void);




/* Private variables ---------------------------------------------------------*/

//	SystemClock variables:
RCC_OscInitTypeDef RCC_OscInitStruct = {0};
RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

//	GPIO variables:
GPIO_InitTypeDef GPIO_InitStruct = {0};

//	ADC1 variables:
ADC_HandleTypeDef hadc1;
ADC_ChannelConfTypeDef sConfig1 = {0};
ADC_MultiModeTypeDef multimode = {0};
	uint16_t ADC1_DATA1 = {0};
	uint16_t ADC1_DATA2 = {0};
	int16_t  ADC1_DATA4 = {0};
	uint16_t ADC1_DATA11 = {0};
	uint16_t  ADC1_DATA12 = {0};
	uint16_t ADC1_DATA1_Raw = {0};
	uint16_t ADC1_DATA2_Raw = {0};
	uint16_t ADC1_DATA4_Raw = {0};
	uint16_t ADC1_DATA11_Raw = {0};
	uint16_t ADC1_DATA12_Raw = {0};
	uint8_t Which_ADC = {0};

//	ADC2 variables:
ADC_HandleTypeDef hadc2;
ADC_ChannelConfTypeDef sConfig2 = {0};
	uint16_t ADC2_DATA1 = {0};
	uint16_t ADC2_DATA1_Raw = {0};
	uint16_t ADC2_DATA2 = {0};
	uint16_t ADC2_DATA2_Raw = {0};
	uint16_t ADC2_DATA4 = {0};
	uint16_t ADC2_DATA4_Raw = {0};

//	UART variables:
UART_HandleTypeDef huart2;
	char msg[100] = {0};

//	CAN variables:
CAN_HandleTypeDef hcan = {0};
CAN_FilterTypeDef can_filter_init = {0};
CAN_TxHeaderTypeDef TxHeader = {0};
CAN_RxHeaderTypeDef RxHeader = {0};
	uint32_t TxMailbox = {0};
	uint8_t Our_Message[8] = {0};
	uint8_t rcvd_msg[8] = {0};

//	Error Handler variables:
uint8_t Error_Number = 0;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 2.                                                     | main |
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  BGR_USART2_UART_Init();
  BGR_GPIO_Init();
  BGR_ADC1_Init();
  BGR_ADC2_Init();
  BGR_CAN_Init();
  BGR_CAN_Filter_Config();

  	  if( HAL_CAN_Start(&hcan) != HAL_OK)
	  {
		  Error_Number = 11;
		  BGR_Error_Handler();
	  }

	  if(HAL_CAN_ActivateNotification(&hcan,CAN_IT_TX_MAILBOX_EMPTY|CAN_IT_RX_FIFO0_MSG_PENDING|CAN_IT_BUSOFF)!= HAL_OK)
	  {
		  Error_Number = 12;
		  BGR_Error_Handler();
	  }

  while (1)
  {
	  //Timing Test: Set GPIO pin high
	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET);

//TEMP1
	  //Configure ADC1_CH1
	  sConfig1.Channel = ADC_CHANNEL_1;

	  //Get ADC1_CH1 value
	  Which_ADC = 1;
	  ADC1_DATA1_Raw = BGR_ADC_Rx(Which_ADC);
	  ADC1_DATA1 = BGR_Samp_To_Temp(ADC1_DATA1_Raw);

	  //Convert to string and print
	  sprintf(msg,"ADC1 CH1  (PA0-A0) Sample:  %d  Temp:  %d\r\n",ADC1_DATA1_Raw,ADC1_DATA1);
	  HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);

//TEMP2
	  //Configure ADC1_CH2
	  sConfig1.Channel = ADC_CHANNEL_2;

	  	  //Get ADC1_CH2 value
	  	  Which_ADC = 1;
	  	  ADC1_DATA2_Raw = BGR_ADC_Rx(Which_ADC);
	  	  ADC1_DATA2 = BGR_Samp_To_Temp(ADC1_DATA2_Raw);

	  	  //Convert to string and print
	  	  sprintf(msg,"ADC1 CH2  (PA1-A1) Sample:  %d  Temp:  %d\r\n",ADC1_DATA2_Raw,ADC1_DATA2);
	  	  HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);

//TEMP3
	  //Configure ADC1_CH4
	  sConfig1.Channel = ADC_CHANNEL_4;

	  	  //Get ADC1_CH4 value
	  	  Which_ADC = 1;
	  	  ADC1_DATA4_Raw = BGR_ADC_Rx(Which_ADC);
	  	  ADC1_DATA4 = BGR_Samp_To_Temp(ADC1_DATA4_Raw);

	  	  //Convert to string and print
	  	  sprintf(msg,"ADC1 CH4  (PA3-A2) Sample:  %d  Temp:  %d\r\n",ADC1_DATA4_Raw,ADC1_DATA4);
	  	  HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);

//TEMP4
	  //Configure ADC1_CH11
	  sConfig1.Channel = ADC_CHANNEL_11;

	  	  //Get ADC1_CH11 value
	  	  Which_ADC = 1;
	  	  ADC1_DATA11_Raw = BGR_ADC_Rx(Which_ADC);
	  	  ADC1_DATA11 = BGR_Samp_To_Temp(ADC1_DATA11_Raw);

	  	  //Convert to string and print
	  	  sprintf(msg,"ADC1 CH11 (PB0-D3) Sample:  %d  Temp:  %d\r\n\n",ADC1_DATA11_Raw,ADC1_DATA11);
	  	  HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);

//VOLT1
	  //Configure ADC1_CH12
	  sConfig1.Channel = ADC_CHANNEL_12;

	  	  //Get ADC1_CH12 value
	  	  Which_ADC = 1;
	  	  ADC1_DATA12_Raw = BGR_ADC_Rx(Which_ADC);
	  	  ADC1_DATA12 = BGR_Samp_To_Volt(ADC1_DATA12_Raw);

	  	  //Convert to string and print
	  	  sprintf(msg,"ADC1 CH12 (PB1-D6) Sample:  %d  Volt:  %d\r\n",ADC1_DATA12_Raw,ADC1_DATA12);
	  	  HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);

//VOLT2
	  //Configure ADC2_CH1
	  sConfig2.Channel = ADC_CHANNEL_1;

	  	  //Get ADC2_CH1 value
	  	  Which_ADC = 2;
	  	  ADC2_DATA1_Raw = BGR_ADC_Rx(Which_ADC);
	  	  ADC2_DATA1 = BGR_Samp_To_Volt(ADC2_DATA1_Raw);

	  	 //Convert to string and print
	  	  sprintf(msg,"ADC2 CH1  (PA4-A3) Sample:  %d  Volt:  %d\r\n",ADC2_DATA1_Raw,ADC2_DATA1);
	  	  HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);

//VOLT3
	  //Configure ADC2_CH2
	  sConfig2.Channel = ADC_CHANNEL_2;

	  	  //Get ADC2_CH2 value
	  	  Which_ADC = 2;
	  	  ADC2_DATA2_Raw = BGR_ADC_Rx(Which_ADC);
	  	  ADC2_DATA2 = BGR_Samp_To_Volt(ADC2_DATA2_Raw);

	  	  //Convert to string and print
	  	  sprintf(msg,"ADC2 CH2  (PA5-A4) Sample:  %d  Volt:  %d\r\n",ADC2_DATA2_Raw,ADC2_DATA2);
	  	  HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);

//VOLT4
	  //Configure ADC2_CH4
	  sConfig2.Channel = ADC_CHANNEL_4;

	  	  //Get ADC2_CH4 value
	  	  Which_ADC = 2;
	  	  ADC2_DATA4_Raw = BGR_ADC_Rx(Which_ADC);
	  	  ADC2_DATA4 = BGR_Samp_To_Volt(ADC2_DATA4_Raw);

	  	  //Convert to string and print
	  	  sprintf(msg,"ADC2 CH4  (PA7-A6) Sample:  %d  Volt:  %d\r\n\n",ADC2_DATA4_Raw,ADC2_DATA4);
	  	  HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);

	  sprintf(msg,"-----------------------------\r\n\n");
	  HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);

	  //Timing Test: Set GPIO pin low
	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET);
	  HAL_Delay(250);
  }
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 3.                                        | System Clock Configurations |
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void SystemClock_Config(void)
{
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.   */

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
	  BGR_Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks*/

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
	  BGR_Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
	  BGR_Error_Handler();
  }
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 4.                                           | GPIO - Initialization |
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void BGR_GPIO_Init(void)
{
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB4 PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 5.                                           | ADC1 - Initialization |
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void BGR_ADC1_Init(void)
{
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;

  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
	  BGR_Error_Handler();
  }

  /** Configure the ADC multi-mode*/
  multimode.Mode = ADC_MODE_INDEPENDENT;

  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
	  BGR_Error_Handler();
  }

  /** Configure Regular Channel*/
  sConfig1.Channel = ADC_CHANNEL_1;
  sConfig1.Rank = ADC_REGULAR_RANK_1;
  sConfig1.SingleDiff = ADC_SINGLE_ENDED;
  sConfig1.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig1.OffsetNumber = ADC_OFFSET_NONE;
  sConfig1.Offset = 0;

  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig1) != HAL_OK)
  {
	  BGR_Error_Handler();
  }
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 6.                                           | ADC2 - Initialization |
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void BGR_ADC2_Init(void)
{
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;

  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
	  BGR_Error_Handler();
  }

  /** Configure Regular Channel*/
  sConfig2.Channel = ADC_CHANNEL_1;
  sConfig2.Rank = ADC_REGULAR_RANK_1;
  sConfig2.SingleDiff = ADC_SINGLE_ENDED;
  sConfig2.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig2.OffsetNumber = ADC_OFFSET_NONE;
  sConfig2.Offset = 0;

  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig2) != HAL_OK)
  {
	  BGR_Error_Handler();
  }
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 7.                            	            | ADC - Data Receiving |
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

int BGR_ADC_Rx(uint8_t ADC)
{
	uint16_t ADC_DATA;

	switch(ADC)
	{
		case 1:

			if (HAL_ADC_ConfigChannel(&hadc1, &sConfig1) != HAL_OK)
			{
				BGR_Error_Handler();
			}
			// Get ADC_CHx value
			if (HAL_ADC_Start(&hadc1) != HAL_OK)
			{
				BGR_Error_Handler();
			}

			if ( HAL_ADC_PollForConversion(&hadc1,HAL_MAX_DELAY) != HAL_OK)
			{
				BGR_Error_Handler();
			}

			ADC_DATA = HAL_ADC_GetValue(&hadc1);

			return ADC_DATA;

		case 2:

			if (HAL_ADC_ConfigChannel(&hadc2, &sConfig2) != HAL_OK)
			{
				BGR_Error_Handler();
			}
			// Get ADC_CHx value
			if (HAL_ADC_Start(&hadc2) != HAL_OK)
			{
				BGR_Error_Handler();
			}

			if ( HAL_ADC_PollForConversion(&hadc2,HAL_MAX_DELAY) != HAL_OK)
			{
				BGR_Error_Handler();
			}

			ADC_DATA = HAL_ADC_GetValue(&hadc2);
			return ADC_DATA;
	}
	return 0;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 8.                            	            | Sample-Temperature Converter |
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

int BGR_Samp_To_Temp(uint16_t ADC_DATA_TEMP)
{
	float temp = (-30.66*log(((float)ADC_DATA_TEMP/4096)*3.3)+0.72);
	ADC_DATA_TEMP = (uint16_t)(temp);

	return ADC_DATA_TEMP;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 9.                            	            | Sample-Volt Converter |
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

int BGR_Samp_To_Volt(uint16_t ADC_DATA_VOLT)
{
	float volt = ((float)ADC_DATA_VOLT/4096)*3.3*100;
	ADC_DATA_VOLT = (uint16_t)(volt);

	return ADC_DATA_VOLT;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 10.                                          | UART - Initialization |
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void BGR_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
	  BGR_Error_Handler();
  }
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 11.                                           | CAN - Initialization |
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void BGR_CAN_Init(void)
{
	hcan.Instance = CAN;
	hcan.Init.Mode = CAN_MODE_NORMAL;
	hcan.Init.AutoBusOff = ENABLE;
	hcan.Init.AutoRetransmission = DISABLE;
	hcan.Init.AutoWakeUp = ENABLE;
	hcan.Init.ReceiveFifoLocked = DISABLE;
	hcan.Init.TimeTriggeredMode = DISABLE;
	hcan.Init.TransmitFifoPriority = DISABLE;

	//Settings related to CAN bit timings  - http://www.bittiming.can-wiki.info/
	hcan.Init.Prescaler = 2;
	hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
	hcan.Init.TimeSeg2 = CAN_BS2_2TQ;

	if ( HAL_CAN_Init (&hcan) != HAL_OK)
	{
		Error_Number = 6;
		BGR_Error_Handler();
	}
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 12.                                       | CAN - Filter Configurations |
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void BGR_CAN_Filter_Config(void)
{
	can_filter_init.FilterActivation = ENABLE;
	can_filter_init.FilterFIFOAssignment = CAN_RX_FIFO0;
	can_filter_init.FilterMode = CAN_FILTERMODE_IDMASK;
	can_filter_init.FilterScale = CAN_FILTERSCALE_32BIT;
	can_filter_init.SlaveStartFilterBank = 0;

/*	We need to mask only the first 11 MSB of register "FilterIdHigh" to filter the address:

	|		   FilterIdHigh (16 bit)			|				            FilterIdLow (16 bit)				   |
	| STID[10:0] (11 bit) | EXID[17:13] (5 bit) | EXID[12:0] (13 bit) |IDE[0] (1 bit) | RTR[0] (1 bit) | 0 (1 bit) |

	Note: even though FilterIdHigh/FilterIdLow are 32 bit, each one of them contains data of 16 bit.						*/

		can_filter_init.FilterBank  = 0;					// Filter number 0.
		can_filter_init.FilterIdHigh = 0x0400;				// id_0x20 - 00000100000|00000
		can_filter_init.FilterIdLow = 0x0000;
		can_filter_init.FilterMaskIdHigh = 0XFFE0;          // mask    - 11111111111|00000
		can_filter_init.FilterMaskIdLow = 0x0000;

		if( HAL_CAN_ConfigFilter(&hcan,&can_filter_init) != HAL_OK)
		{
			Error_Number = 7;
			BGR_Error_Handler();
		}

		can_filter_init.FilterBank  = 1;					// Filter number 1.
		can_filter_init.FilterIdHigh = 0x0600;  			// id_0x30 - 00000110000|00000
		can_filter_init.FilterIdLow = 0x0000;
		can_filter_init.FilterMaskIdHigh = 0XFFF0;          // mask    - 11111111111|00000
		can_filter_init.FilterMaskIdLow = 0x0000;

		if( HAL_CAN_ConfigFilter(&hcan,&can_filter_init) != HAL_OK)
		{
			Error_Number = 8;
			BGR_Error_Handler();
		}

		can_filter_init.FilterBank  = 2;					// Filter number 2.
		can_filter_init.FilterIdHigh = 0x0800;  			// id_0x40 - 00001000000|00000
		can_filter_init.FilterIdLow = 0x0000;
		can_filter_init.FilterMaskIdHigh = 0XFFF0;          // mask    - 11111111111|00000
		can_filter_init.FilterMaskIdLow = 0x0000;

		if( HAL_CAN_ConfigFilter(&hcan,&can_filter_init) != HAL_OK)
		{
			Error_Number = 9;
			BGR_Error_Handler();
		}

		can_filter_init.FilterBank  = 3;					// Filter number 3.
		can_filter_init.FilterIdHigh = 0x0A00;  			// id_0x50 - 00001010000|00000
		can_filter_init.FilterIdLow = 0x0000;
		can_filter_init.FilterMaskIdHigh = 0XFFF0;          // mask    - 11111111111|00000
		can_filter_init.FilterMaskIdLow = 0x0000;

		if( HAL_CAN_ConfigFilter(&hcan,&can_filter_init) != HAL_OK)
		{
			Error_Number = 10;
			BGR_Error_Handler();
		}
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 13.                                        | CAN - Message Transmitting |
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void BGR_CAN_Tx(uint8_t node)
{
	switch(node)
	{
		case 1:

			// the content of the massage - each block is 1 byte
			Our_Message[0] = (ADC1_DATA1 >> 8);
			Our_Message[1] =  ADC1_DATA1 & 0xff;
			Our_Message[2] = (ADC1_DATA2 >> 8);
			Our_Message[3] =  ADC1_DATA2 & 0xff;
			Our_Message[4] = (ADC1_DATA4 >> 8);
			Our_Message[5] =  ADC1_DATA4 & 0xff;
			Our_Message[6] = (ADC1_DATA11 >> 8);
			Our_Message[7] =  ADC1_DATA11 & 0xff;

			TxHeader.DLC = 8;
			TxHeader.StdId = 0x60;
			TxHeader.IDE   = CAN_ID_STD;
			TxHeader.RTR = CAN_RTR_DATA;
			break;

		case 2:

			// the content of the massage - each block is 1 byte
			Our_Message[0] = (ADC1_DATA12 >> 8);
			Our_Message[1] =  ADC1_DATA12 & 0xff;
			Our_Message[2] = (ADC2_DATA1 >> 8);
			Our_Message[3] =  ADC2_DATA1 & 0xff;
			Our_Message[4] = (ADC2_DATA2 >> 8);
			Our_Message[5] =  ADC2_DATA2 & 0xff;
			Our_Message[6] = (ADC2_DATA4 >> 8);
			Our_Message[7] =  ADC2_DATA4 & 0xff;

			TxHeader.DLC = 8;
			TxHeader.StdId = 0x70;
			TxHeader.IDE   = CAN_ID_STD;
			TxHeader.RTR = CAN_RTR_DATA;
			break;

		case 3:

			// the content of the massage - each block is 1 byte
			Our_Message[0] = (ADC1_DATA1_Raw >> 8);
			Our_Message[1] =  ADC1_DATA1_Raw & 0xff;
			Our_Message[2] = (ADC1_DATA2_Raw >> 8);
			Our_Message[3] =  ADC1_DATA2_Raw & 0xff;
			Our_Message[4] = (ADC1_DATA4_Raw >> 8);
			Our_Message[5] =  ADC1_DATA4_Raw & 0xff;
			Our_Message[6] = (ADC1_DATA11_Raw >> 8);
			Our_Message[7] =  ADC1_DATA11_Raw & 0xff;

			TxHeader.DLC = 8;
			TxHeader.StdId = 0x80;
			TxHeader.IDE   = CAN_ID_STD;
			TxHeader.RTR = CAN_RTR_DATA;
			break;

		case 4:

			// the content of the massage - each block is 1 byte
			Our_Message[0] = (ADC1_DATA12_Raw >> 8);
			Our_Message[1] =  ADC1_DATA12_Raw & 0xff;
			Our_Message[2] = (ADC2_DATA1_Raw >> 8);
			Our_Message[3] =  ADC2_DATA1_Raw & 0xff;
			Our_Message[4] = (ADC2_DATA2_Raw >> 8);
			Our_Message[5] =  ADC2_DATA2_Raw & 0xff;
			Our_Message[6] = (ADC2_DATA4_Raw >> 8);
			Our_Message[7] =  ADC2_DATA4_Raw & 0xff;

			TxHeader.DLC = 8;
			TxHeader.StdId = 0x90;
			TxHeader.IDE   = CAN_ID_STD;
			TxHeader.RTR = CAN_RTR_DATA;
			break;
	}

	if( HAL_CAN_AddTxMessage(&hcan,&TxHeader,Our_Message,&TxMailbox) != HAL_OK)
	{
		Error_Number = 17;
		BGR_Error_Handler();
	}
	else
	{
//		sprintf(msg,"CANBUS Massage transmitted!\r\n\n");
//		HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);
//		sprintf(msg,"-----------------------------\r\n\n");
//		HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);
	}
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 14.                                          | CAN - Message Receiving |
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void BGR_CAN_Rx(void)
{
	uint8_t CanNode;
	uint8_t Id_10 = 0x10;
	uint8_t Id_20 = 0x20;
	uint8_t Id_30 = 0x30;
	uint8_t Id_40 = 0x40;
	uint8_t Id_50 = 0x50;

	if( HAL_CAN_GetRxMessage(&hcan,CAN_RX_FIFO0,&RxHeader,rcvd_msg) != HAL_OK)
	{
		Error_Number = 16;
		BGR_Error_Handler();
	}
	else
	{
//		sprintf(msg,"CANBUS massage received!\r\n");
//		HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);
	}

	if(RxHeader.StdId != Id_10)
	{
		if(RxHeader.StdId == Id_20)
		{
			CanNode = 1;
			BGR_CAN_Tx(CanNode);

			CanNode = 2;
			BGR_CAN_Tx(CanNode);
			return;
		}

		if(RxHeader.StdId == Id_30)
		{

			CanNode = 3;
			BGR_CAN_Tx(CanNode);

			CanNode = 4;
			BGR_CAN_Tx(CanNode);
			return;
		}

		if(RxHeader.StdId == Id_40)
		{
//			CanNode = 3;
//			BGR_CAN_Tx(CanNode);
			return;
		}

		if(RxHeader.StdId == Id_50)
		{
//			CanNode = 4;
//			BGR_CAN_Tx(CanNode);
			return;
		}
	}
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 15.                                            | Interrupt Management |
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_4);
	BGR_CAN_Rx();
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 16.                                                 | Error Handler |
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void BGR_Error_Handler(void)
{
	while(1);
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~| FIN |~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
