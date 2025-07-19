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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "memorymap.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DRAM __attribute__((section (".DRAM")))
#define IRAM __attribute__((section (".IRAM")))
#define RAM1 __attribute__((section (".RAM1")))

#define LED_G_1 HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET)
#define LED_G_0 HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET)

#define LED_R_1 HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET)
#define LED_R_0 HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET)

#define LED_Y_1 HAL_GPIO_WritePin(LED_Y_GPIO_Port, LED_Y_Pin, GPIO_PIN_SET)
#define LED_Y_0 HAL_GPIO_WritePin(LED_Y_GPIO_Port, LED_Y_Pin, GPIO_PIN_RESET)

#define LED_5_1 HAL_GPIO_WritePin(LED_5_GPIO_Port, LED_5_Pin, GPIO_PIN_SET) // GY-87 LED
#define LED_5_0 HAL_GPIO_WritePin(LED_5_GPIO_Port, LED_5_Pin, GPIO_PIN_RESET)

#define LED_6_1 HAL_GPIO_WritePin(LED_6_GPIO_Port, LED_6_Pin, GPIO_PIN_SET) // NRF24 LED
#define LED_6_0 HAL_GPIO_WritePin(LED_6_GPIO_Port, LED_6_Pin, GPIO_PIN_RESET)

#define LED_7_1 HAL_GPIO_WritePin(LED_7_GPIO_Port, LED_7_Pin, GPIO_PIN_SET) // USART LED
#define LED_7_0 HAL_GPIO_WritePin(LED_7_GPIO_Port, LED_7_Pin, GPIO_PIN_RESET)

#define LED_uSD_1 HAL_GPIO_WritePin(uSD_LED_GPIO_Port, uSD_LED_Pin, GPIO_PIN_SET) // uSD LED
#define LED_uSD_0 HAL_GPIO_WritePin(uSD_LED_GPIO_Port, uSD_LED_Pin, GPIO_PIN_RESET)

#define BUZZ_1 HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_SET)
#define BUZZ_0 HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_RESET)

#define ESC_POWER_1 HAL_GPIO_WritePin(ESC_Power_GPIO_Port, ESC_Power_Pin, GPIO_PIN_SET)//1 = unucitve
#define ESC_POWER_0 HAL_GPIO_WritePin(ESC_Power_GPIO_Port, ESC_Power_Pin, GPIO_PIN_RESET)//0 = active

#define NRF24_CSN_HIGH		HAL_GPIO_WritePin(NRF24_CSN_GPIO_Port, NRF24_CSN_Pin, GPIO_PIN_SET)
#define NRF24_CSN_LOW		HAL_GPIO_WritePin(NRF24_CSN_GPIO_Port, NRF24_CSN_Pin, GPIO_PIN_RESET)

#define NRF24_CE_HIGH		HAL_GPIO_WritePin(NRF24_CE_GPIO_Port, NRF24_CE_Pin, GPIO_PIN_SET)
#define NRF24_CE_LOW		HAL_GPIO_WritePin(NRF24_CE_GPIO_Port, NRF24_CE_Pin, GPIO_PIN_RESET)

#define DegToRad 0.01745329251994329576923690768489f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
IRAM MPU6050_Struct MPU6050;
IRAM BMP180_Struct BMP180;
IRAM NRF24_Struct NRF24;
IRAM Dron MYDRON;
IRAM HMC5883L_Struct HMC5883L;
IRAM USART_Struct USART;
IRAM Madgwick_Struct Madgwick;
IRAM HC_SR04_Struct HC_SR04;
IRAM Inactivity_Struct Inactivity;

///////// main
IRAM int TIM_inte = 0;
IRAM uint8_t STARTUP = 1;
IRAM uint32_t NRF_TIM_Inte = 0;
RAM1 uint32_t analogmess;
IRAM uint32_t i = 0, loopnum = 0;
IRAM uint8_t NRF24_Messages_SC = 0;
IRAM uint8_t NRF24_inte = 0;
IRAM uint16_t FDP_D_Gain_AR = 0;
IRAM uint16_t FDP_D_Gain = 0;
const float looptime = 0.001; //todo #define
IRAM float wobble_strenght = 1; //todo ?
const uint16_t DRON_SLOWFALING = 2700;
IRAM uint8_t DRON_ON_GRUND;
IRAM uint8_t ESC_Start;
IRAM uint32_t uIOC_MASK; //todo ?
IRAM uint32_t uDZC_MASK; //todo ?
IRAM uint64_t _1_ms_count;

#define FDP_Mag_Z_FQ 2
#define FDP_FQ 2
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C5_Init();
  MX_ADC2_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();
  MX_TIM24_Init();
  MX_TIM5_Init();
  MX_TIM17_Init();
  MX_TIM16_Init();
  MX_TIM13_Init();
  /* USER CODE BEGIN 2 */
  	  ESC_POWER_1;


  	STARTUP = 1;
  	DRON_ON_GRUND = 1;
  	ESC_Start = 0;



	USART.Received = 0;
	USART.command[0] = 0;
	USART.command_ch_num = 0;
	USART.commandready = 0;

	Inactivity.Compare_Value = 24991806;
	Inactivity.Counter = 0;
	Inactivity.Inact_Percent = 0;

    TIM_inte = 0;
  	NRF_TIM_Inte = 0;
  	FDP_D_Gain_AR = 0;
  	FDP_D_Gain = 0;
  	wobble_strenght = 1;
  	i = 0, loopnum = 0;
  	NRF24_inte = 0;
  	_1_ms_count = 0;


  	LED_5_1;
  	LED_6_1;
  	LED_7_1;
  	LED_uSD_1;
  	LED_G_1;
  	LED_Y_1;
  	LED_R_1;
  	HAL_Delay(1000);
  	LED_G_0;
  	LED_Y_0;
  	LED_R_0;
  	LED_5_0;
  	LED_6_0;
  	LED_7_0;
  	LED_uSD_0;

  	Dron_Init();

	Madgwick.beta = 0.1f;
	Madgwick.q0 = 1.0f;
	Madgwick.q1 = 0.0f;
	Madgwick.q2 = 0.0f;
	Madgwick.q3 = 0.0f;
	Madgwick.z_offset = 0;
	Madgwick.x = 0;
	Madgwick.y = 0;
	Madgwick.z = 0;


  	analogmess = 0;


  	HAL_TIM_Base_Start(&htim8);
  	HAL_ADC_Start_DMA(&hadc2, &analogmess, 1);
  	LED_R_1;
  	while(analogmess == 0){

  	}
  	LED_R_0;

  	Get_batteryvalue();

  	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);//pwm do diodt RGB
  	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);


  	RGB_LED_For_BAT(MYDRON.batterysize);
  	if(MYDRON.Status.Battery == DRON_BATTERY_RUN_OUT){
  		LED_R_1;
  		while(1){

  		}
  	}
  	if(MYDRON.Status.Battery == DRON_BATTERY_CRIT_VAL){
  		LED_R_1;

  		while(1){
  		}
  	}

	/////////////////////////////// MPU6050
		LED_5_1;
		if(MPU6050_INIT(&hi2c5) == 0){
			LED_R_1;
			NVIC_SystemReset();
			while(1){
			}
		}
		LED_Y_1;
		MPU6050_CALIBRATION();
		LED_Y_0;

	/////////////////////////////// BMP180
		if(BMP180_init(&hi2c5, &htim16) == 0){
			LED_R_1;
			NVIC_SystemReset();
			while(1){
			}
		}
		LED_Y_1;
		BMP180_CALIBRATION(&BMP180.startpres);
		LED_Y_0;
	/////////////////////////////// HMC5883L
		if(HMC5883L_Init(&hi2c5) == 0){
			LED_R_1;
			NVIC_SystemReset();
			while(1){
			}
		}
		LED_Y_1;
		HMC5883L.Off_Set_Values.Z = HMC5883L_Calibration();
		LED_Y_0;

		LED_5_0;


		HC_SR04_Init(&HC_SR04, &htim17, &htim5);
	/////////////////////////////// nRF24
		nRF24_Start_Up();
	///////////////////////////////////////////////////////////////////////
		HAL_TIM_Base_Start_IT(&htim2); // przerwanie co 1 ms

		ESC_INT(&htim3);
//	  	while(_1_ms_count < 3000){//3sec for esc to start up
//
//	  	}


		LED_7_1;
		HAL_UART_Receive_IT(&huart1, &USART.Received, 1);
		LED_7_0;


		HAL_TIM_Base_Start_IT(&htim24);//przerwanie co 100ms

		Get_batteryvalue();
	  	RGB_LED_For_BAT(MYDRON.batterysize);

	  	if(MYDRON.Status.Battery == DRON_BATTERY_RUN_OUT)
	  	{
	  		LED_R_1;
	  		while(1){

	  		}
	  	}

	  	if(MYDRON.Status.Battery == DRON_BATTERY_CRIT_VAL){
	  		LED_R_1;
	  		while(1){
	  		}
	  	}

	  	while(_1_ms_count < 54000){//1min waiting for Madgwick "stratup"
	  		BUZZ_1;
	  		if(_1_ms_count%2 == 0){
	  			BUZZ_0;
	  		}
	  	}
	  	BUZZ_1;
	  	while(_1_ms_count != 60000){

	  	}
	  	HC_SR04_Init(&HC_SR04, &htim17, &htim5);
	  	Madgwick.z_offset = Madgwick.z;
	  	while(_1_ms_count != 64000){

	  	}
	  	BUZZ_0;
	  	ESC_Start = 1;
	  	HAL_TIM_Base_Start_IT(&htim13);// inactivity clock, irq every 1s
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) //todo add camera
  {
	  Inactivity.Counter++;

	  if(MYDRON.Status.Battery == DRON_BATTERY_RUN_OUT){
		  LED_R_1;
		  MYDRON.Thrust.Thrust_Limit = MYDRON.Thrust.Now;
	  }
	  if(MYDRON.Status.Battery == DRON_BATTERY_OK && MYDRON.Thrust.Thrust_Limit != 10000){
		  MYDRON.Thrust.Thrust_Limit = 10000;
	  }
	  if(MYDRON.Status.Battery == DRON_BATTERY_CRIT_VAL){
		  ESC_POWER_1;
		  LED_R_1;
		  while(MYDRON.Status.Battery == DRON_BATTERY_CRIT_VAL){
			  HAL_Delay(10);
			  Get_batteryvalue();
		  }
	  }

	  if(USART.commandready == 1){
		  LED_7_1;
		  interpretcommand();
		  executecommand(USART.command, USART.UASRT_PID_VAL);
		  LED_7_0;
	  }

	  nRF24_While_Process();
	  if(TIM_inte == 1){
		  LED_R_0;
		  TIM_inte = 0;
	  }
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_CSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.CSIState = RCC_CSI_ON;
  RCC_OscInitStruct.CSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 3;
  RCC_OscInitStruct.PLL.PLLN = 68;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 6144;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_SPI1;
  PeriphClkInitStruct.PLL2.PLL2M = 22;
  PeriphClkInitStruct.PLL2.PLL2N = 192;
  PeriphClkInitStruct.PLL2.PLL2P = 3;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_0;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL2;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void FPU_IRQ_CallBack(){
	for(int i = 0; i < 2; i++){
		LED_R_1;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim == &htim13){
		Inactivity.Inact_Percent = ((float)Inactivity.Counter/Inactivity.Compare_Value)*100;
		Inactivity.Counter = 0;
	}
	if(htim == &htim24){//100ms
		//BMP180_100_ms_Process(); //same, but not working
		BMP180_start_measurment_temp_IT();
		BMP180.Step = 1;//Step 1
	}
	if(htim == &htim16){//0.5 ms
		BMP180_05_ms_Process();
	}
	if(htim == &htim17){
		HC_SR04_End_Messurment(&HC_SR04);
	}
	if(htim == &htim2){// 1 ms
		_1_ms_count++;
		TIM_inte = 1;
		NRF_TIM_Inte++;

		nRF24_1ms_Process();


		if(i == 0){// na calosc 100ms
			RGB_LED_For_BAT(MYDRON.batterysize);
			HC_SR04_Start_Messurment(&HC_SR04);
		}
		if(i == 36){
			LED_5_1;
			HMC5883L_Get_Z_Start_IT();
			LED_5_0;
		}

		if(i == 40){//5
			Get_batteryvalue();
			convert_value_to_array(MYDRON.barometrick_height, NRF24.TxData, 0, 3);
			convert_value_to_array(MYDRON.batterysize, NRF24.TxData, 3, 6);

			for(int i = 0; i < 10; i++){
				NRF24.TxData[22+i] = NRF24.Txcode[22+i];
			}
		}


		LED_5_1;
		MPU6050_GET_ACCANDGYR_CALANDSCL_IT();
		LED_5_0;

		i = (i == 100) ? 0 : i+1;

		if(NRF_TIM_Inte >= 1000){
			LED_R_1;
			MYDRON.Status.Connection = DRON_DISCONNECTED;
		}
		if(MYDRON.Status.Connection == DRON_DISCONNECTED){
			MYDRON.Rool.Wanted = 0;
			MYDRON.Pitch.Wanted = 0;
			MYDRON.Yaw.Wanted = Madgwick.z_offset;
			MYDRON.Thrust.Wanted = DRON_SLOWFALING;
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_15){
		nRF24_GPIO_EXTI_Process();
	}
	if(GPIO_Pin == GPIO_PIN_1){
		HC_SR04_Get_Echo(&HC_SR04);
		MYDRON.to_ground_height = HC_SR04.Distance;
		IS_DRON_ON_GROUND(&MYDRON, HC_SR04);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)//pobieranie znakw z uart
{
	USART.words[USART.command_ch_num] = USART.Received;

	if(USART.words[USART.command_ch_num] == '\r')
	{
		USART.words[USART.command_ch_num] = 0;
		USART.commandready = 1;
	}
	HAL_UART_Receive_IT(&huart1, &USART.Received, 1);
	USART.command_ch_num++;

	if(USART.commandready == 1)
	{
		USART.command_ch_num = 0;
	}
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c){
	BMP180_I2C_Tx_IT_Process();
}


void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){
	if(MPU6050.MPU6050_IRQ == 1){
			MPU6050.MPU6050_IRQ = 0;//todo takeoff mode

			LED_G_1;

			MPU6050_GET_CALANDSCL_IT();
			MPU6050_GET_ACCEL_TO_ANGLE();
			MadgwickAHRSupdate(MPU6050.Gyr.gx*DegToRad, MPU6050.Gyr.gy*DegToRad, MPU6050.Gyr.gz*DegToRad, MPU6050.Acc.ax, MPU6050.Acc.ay, MPU6050.Acc.az, HMC5883L.Directions.X, HMC5883L.Directions.Y, HMC5883L.Directions.Z);
			Madgwick_Get_Angles();
			Angle_Filter(&MYDRON, Madgwick);
			MPU6050_VerAcc_Cal(&MPU6050, Madgwick);



			MYDRON.Rool.Wanted = (float)MYDRON.Rool.Wanted_rx/10;
			MYDRON.Pitch.Wanted = (float)MYDRON.Pitch.Wanted_rx/10;
			MYDRON.Yaw.Wanted = Madgwick.z_offset;//(float)MYDRON.Yaw.Wanted_rx/10;
				  			/*
				  				 * FDP
				  				 */
			MYDRON.Rool.Wanted = (MYDRON.Rool.Wanted * (FDP_FQ * looptime) / (1 + (FDP_FQ * looptime))) + (MYDRON.Rool.Last_Wanted_rx * (1 / (1 + (FDP_FQ * looptime))));
			MYDRON.Pitch.Wanted = (MYDRON.Pitch.Wanted * (FDP_FQ * looptime) / (1 + (FDP_FQ * looptime))) + (MYDRON.Pitch.Last_Wanted_rx * (1 / (1 + (FDP_FQ * looptime))));
			//MYDRON.Yaw.Wanted = (MYDRON.Yaw.Wanted * (FDP_FQ * looptime) / (1 + (FDP_FQ * looptime))) + (MYDRON.Yaw.Last_Wanted_rx * (1 / (1 + (FDP_FQ * looptime))));


			MYDRON.Rool.Last_Wanted_rx = MYDRON.Rool.Wanted;
			MYDRON.Pitch.Last_Wanted_rx = MYDRON.Pitch.Wanted;
			//MYDRON.Yaw.Last_Wanted_rx= MYDRON.Yaw.Wanted;


			MYDRON.Pitch.Angle_Error = MYDRON.Pitch.Wanted - MYDRON.Pitch.Now;
			MYDRON.Rool.Angle_Error = MYDRON.Rool.Wanted - MYDRON.Rool.Now;
		  	MYDRON.Yaw.Angle_Error = MYDRON.Yaw.Wanted - MYDRON.Yaw.Now;


		  	if(MYDRON.Status.On_Grund == 0 || MYDRON.Thrust.Wanted > 0){
				MYDRON.Pitch.Angle_Error_Sum = (MYDRON.PID_Pitch.Status != 0) ? MYDRON.Pitch.Angle_Error_Sum : MYDRON.Pitch.Angle_Error_Sum + (MYDRON.Pitch.Angle_Error);//pitch_error -> pitch_error
				MYDRON.Rool.Angle_Error_Sum = (MYDRON.PID_Rool.Status != 0) ? MYDRON.Rool.Angle_Error_Sum : MYDRON.Rool.Angle_Error_Sum + (MYDRON.Rool.Angle_Error);//rool_error
				MYDRON.Yaw.Angle_Error_Sum = (MYDRON.PID_Yaw.Status != 0) ? MYDRON.Yaw.Angle_Error_Sum : MYDRON.Yaw.Angle_Error_Sum + (MYDRON.Yaw.Angle_Error);//yaw_error

				MYDRON.Pitch.Angular_Rate_Error_Sum = (MYDRON.PID_Pitch.Status != 0) ? MYDRON.Pitch.Angular_Rate_Error_Sum : MYDRON.Pitch.Angular_Rate_Error_Sum + (MYDRON.Pitch.Angular_Rate_Error);//pitch_ar_error
				MYDRON.Rool.Angular_Rate_Error_Sum = (MYDRON.PID_Rool.Status != 0) ? MYDRON.Rool.Angular_Rate_Error_Sum : MYDRON.Rool.Angular_Rate_Error_Sum + (MYDRON.Rool.Angular_Rate_Error);
				MYDRON.Yaw.Angular_Rate_Error_Sum = (MYDRON.PID_Yaw.Status != 0) ? MYDRON.Yaw.Angular_Rate_Error_Sum : MYDRON.Yaw.Angular_Rate_Error_Sum + (MYDRON.Yaw.Angular_Rate_Error);
		  	}	else{
		  		MYDRON.Pitch.Angle_Error_Sum = 0;
		  		MYDRON.Rool.Angle_Error_Sum = 0;
		  		MYDRON.Yaw.Angle_Error_Sum = 0;

		  		MYDRON.Pitch.Angular_Rate_Error_Sum = 0;
		  		MYDRON.Rool.Angular_Rate_Error_Sum = 0;
		  		MYDRON.Yaw.Angular_Rate_Error_Sum = 0;
		  	}

			PID_call(&MYDRON);


			MYDRON.Pitch.Old_Angle_Error = MYDRON.Pitch.Angle_Error;
			MYDRON.Rool.Old_Angle_Error = MYDRON.Rool.Angle_Error;
			MYDRON.Yaw.Old_Angle_Error = MYDRON.Yaw.Angle_Error;

			MYDRON.Pitch.Old_Angular_Rate_Error = MYDRON.Pitch.Angular_Rate_Error;
			MYDRON.Rool.Old_Angular_Rate_Error = MYDRON.Rool.Angular_Rate_Error;
			MYDRON.Yaw.Old_Angular_Rate_Error = MYDRON.Yaw.Angular_Rate_Error;


			MYDRON.Pitch.Value  	= (MYDRON.PID_Pitch.Angular_Rate_Value > 5000) ? PITCH_MAX_VAL(): (MYDRON.PID_Pitch.Angular_Rate_Value < -5000) ? PITCH_MIN_VAL(): PITCH_GOOD_VAL();
			MYDRON.Rool.Value 	    = (MYDRON.PID_Rool.Angular_Rate_Value > 5000) ? ROOL_MAX_VAL(): (MYDRON.PID_Rool.Angular_Rate_Value < -5000) ? ROOL_MIN_VAL(): ROOL_GOOD_VAL();
			MYDRON.Yaw.Value  		= (MYDRON.PID_Yaw.Angular_Rate_Value > 5000) ? YAW_MAX_VAL(): (MYDRON.PID_Yaw.Angular_Rate_Value < -5000) ? YAW_MIN_VAL(): YAW_GOOD_VAL();



			Thrust_filter(1);
			if(MYDRON.Thrust.Now > MYDRON.Thrust.Thrust_Limit){
				MYDRON.Thrust.Now = MYDRON.Thrust.Thrust_Limit;
			}

			MYDRON.Thrust.Speed_1 = (((uint32_t)((MYDRON.Thrust.Now*0.7) + MYDRON.Rool.Value - MYDRON.Pitch.Value + MYDRON.Yaw.Value + min_speed + 500)) < max_speed) ? ((MYDRON.Thrust.Now*0.7) + MYDRON.Rool.Value - MYDRON.Pitch.Value + MYDRON.Yaw.Value + min_speed + 500) : max_speed;//trust 7000 max
			MYDRON.Thrust.Speed_2 = (((uint32_t)((MYDRON.Thrust.Now*0.7) - MYDRON.Rool.Value - MYDRON.Pitch.Value - MYDRON.Yaw.Value + min_speed + 500)) < max_speed) ? ((MYDRON.Thrust.Now*0.7) - MYDRON.Rool.Value - MYDRON.Pitch.Value - MYDRON.Yaw.Value + min_speed + 500) : max_speed;//
			MYDRON.Thrust.Speed_3 = (((uint32_t)((MYDRON.Thrust.Now*0.7) + MYDRON.Rool.Value + MYDRON.Pitch.Value - MYDRON.Yaw.Value + min_speed + 500)) < max_speed) ? ((MYDRON.Thrust.Now*0.7) + MYDRON.Rool.Value + MYDRON.Pitch.Value - MYDRON.Yaw.Value + min_speed + 500) : max_speed;//
			MYDRON.Thrust.Speed_4 = (((uint32_t)((MYDRON.Thrust.Now*0.7) - MYDRON.Rool.Value + MYDRON.Pitch.Value + MYDRON.Yaw.Value + min_speed + 500)) < max_speed) ? ((MYDRON.Thrust.Now*0.7) - MYDRON.Rool.Value + MYDRON.Pitch.Value + MYDRON.Yaw.Value + min_speed + 500) : max_speed;//

			if(ESC_Start == 1){
				if( MYDRON.Thrust.Speed_1 != MYDRON.Thrust.Old_Speed_1){
					ESC_1_SPEED( MYDRON.Thrust.Speed_1);
				}
				if( MYDRON.Thrust.Speed_2 != MYDRON.Thrust.Old_Speed_2){
					ESC_2_SPEED( MYDRON.Thrust.Speed_2);
				}
				if( MYDRON.Thrust.Speed_3 != MYDRON.Thrust.Old_Speed_3){
					ESC_3_SPEED( MYDRON.Thrust.Speed_3);
				}
				if( MYDRON.Thrust.Speed_4 != MYDRON.Thrust.Old_Speed_4){
					ESC_4_SPEED( MYDRON.Thrust.Speed_4);
				}
			}

			MYDRON.Thrust.Old_Speed_1 = MYDRON.Thrust.Speed_1;
			MYDRON.Thrust.Old_Speed_2 = MYDRON.Thrust.Speed_2;
			MYDRON.Thrust.Old_Speed_3 = MYDRON.Thrust.Speed_3;
			MYDRON.Thrust.Old_Speed_4 = MYDRON.Thrust.Speed_4;

			LED_G_0;
	}
	if(HMC5883L.HMC583L_IRQ == 1){
		HMC5883L.HMC583L_IRQ = 0;
		HMC5883L_Get_Z_End_IT();
		HMC5883L.Directions.Z = (HMC5883L.Directions.Z * (FDP_Mag_Z_FQ * 0.1) / (1 + (FDP_Mag_Z_FQ * 0.1))) + (HMC5883L.Directions.Old_Z * (1 / (1 + (FDP_Mag_Z_FQ * 0.1)))); // 0.1 to looptime, co 100ms odczyt
		HMC5883L.Directions.Old_Z = HMC5883L.Directions.Z;
	}
	BMP180_I2C_Rx_IT_Process();
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef * hspi){
	nRF24_SPI_Tx_IT_Process();
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef * hspi){
	nRF24_SPI_Rx_IT_Process();
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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
