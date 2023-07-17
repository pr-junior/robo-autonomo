/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "MusicPlayer.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define IN1_Pin  			GPIO_PIN_1
#define IN2_Pin  			GPIO_PIN_0
#define IN3_Pin  			GPIO_PIN_7
#define IN4_Pin 			GPIO_PIN_6
#define ButtonA_Pin    		GPIO_PIN_0
#define ButtonB_Pin    		GPIO_PIN_12
#define EncoderA_Pin    	GPIO_PIN_5
#define EncoderB_Pin    	GPIO_PIN_4
#define Led_Pin    			GPIO_PIN_13
#define TriggerE_Pin   		GPIO_PIN_15
#define TriggerC_Pin    	GPIO_PIN_11
#define TriggerD_Pin    	GPIO_PIN_12

#define LedE_Pin    		GPIO_PIN_5
#define LedC_Pin    		GPIO_PIN_6
#define LedD_Pin    		GPIO_PIN_7

#define IN12_GPIO_Port   	GPIOB
#define IN34_GPIO_Port   	GPIOA
#define ButtonA_GPIO_Port   GPIOA
#define ButtonB_GPIO_Port   GPIOB
#define EncoderA_GPIO_Port  GPIOA
#define EncoderB_GPIO_Port  GPIOA
#define Led_GPIO_Port    	GPIOC
#define TriggerE_GPIO_Port  GPIOB
#define TriggerC_GPIO_Port  GPIOA
#define TriggerD_GPIO_Port  GPIOA
#define LedD_GPIO_Port   	GPIOB
#define LedC_GPIO_Port    	GPIOB
#define LedE_GPIO_Port    	GPIOB


#define PWM_MotorA    		TIM_CHANNEL_2
#define PWM_MotorB    		TIM_CHANNEL_3
#define Buzzer    			TIM_CHANNEL_4




/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

osThreadId Estado1Handle;
osThreadId Estado2Handle;
osThreadId Estado3Handle;
/* USER CODE BEGIN PV */



int melodia3[] =
{ b4f, b4f, a4f, a4f,
  f5, f5, e5f, b4f, b4f, a4f, a4f, e5f, e5f, c5s, c5, b4f,
  c5s, c5s, c5s, c5s,
  c5s, e5f, c5, b4f, a4f, a4f, a4f, e5f, c5s,
  b4f, b4f, a4f, a4f,
  f5,  f5, e5f, b4f, b4f, a4f, a4f, a5f, c5, c5s, c5, b4f,
  c5s, c5s, c5s, c5s,
  c5s, e5f, c5, b4f, a4f, rest, a4f, e5f, c5s, rest
};


int tempo3[]  =
{ 100, 100, 100, 100,
  300, 300, 800, 100, 100, 100, 100, 400, 400, 400, 100, 200,
  100, 100, 100, 100,
  300, 300, 300, 100, 200, 200, 200, 400, 800,
  100, 100, 100, 100,
  300, 300, 800, 100, 100, 100, 100, 400, 400, 300,  100, 200,
  100, 100, 100, 100,
  300, 300, 300, 100, 200, 200, 200, 400, 800, 40
};



	int melodia2[] = {

			a, a, a, f,
			cH, a, f, cH,
			a,
			eH, eH, eH,
			fH, cH, gS, f,
			cH, a,

			aH, a, a, aH,
			gSH, gH, fSH, fH,
			fSH,
			aS, dSH, dH, cSH,
			cH, bb, cH,

			f, gS, f, a, cH, eH,

			aH, a, a, aH,
			gSH, gH, fSH, fH,
			fSH,
			aS, dSH, dH, cSH,
			cH, bb, cH,

			f, gS, f, cH, a, f, cH, a, cH, a, a
	};

	// Star Wars Tempo

	int tempo2[] = {

			500, 500, 500, 350,
			150, 500, 350, 150,
			650,
			500, 500, 500,
			350, 150, 500, 350,
			150, 1000,

			500, 300, 150, 500,
			325, 175, 125, 125,
			500,
			250, 500, 325, 175,
			125, 125, 500,

			250, 500, 350, 125, 500, 375, 125, 650,

			500, 300, 150, 500,
			325, 175, 125, 125,
			500,
			250, 500, 325, 175,
			125, 125, 500,

			250, 500, 375, 125, 500, 375, 125, 1000, 125, 1000, 1000
	};




int aux, Start  = 0;
int isGoalA, isGoalB, ligaencoderE, ligaencoderD;

int estadoButtonA, estadoButtonB, estadoAnteriorA, estadoAnteriorB, estadoAtualA, estadoAtualB, estadoMotorA, estadoMotorB = 0;
int pulsoA, pulsoB, grausA, grausB, auxA, auxB = 0, pulsos_por_voltaA = 19, pulsos_por_voltaB = 19;

uint16_t dc = 0;

uint32_t IC_Val1E = 0;
uint32_t IC_Val2E = 0;//
uint32_t DifferenceE = 0;//
uint8_t Is_First_CapturedE = 0;  // is the first value captured ?x
uint8_t DistanceE  = 0;


uint32_t IC_Val1C = 0;
uint32_t IC_Val2C = 0;//
uint32_t DifferenceC = 0;//
uint8_t Is_First_CapturedC = 0;  // is the first value captured ?x
uint8_t DistanceC  = 0;


uint32_t IC_Val1D = 0;
uint32_t IC_Val2D = 0;//
uint32_t DifferenceD = 0;//
uint8_t Is_First_CapturedD = 0;  // is the first value captured ?x
uint8_t DistanceD  = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
void StartEstado1(void const * argument);
void StartEstado2(void const * argument);
void StartEstado3(void const * argument);

/* USER CODE BEGIN PFP */

// Motor Direito
void dcA(){

	//TIM2->CCR4 = 375;
	  dc = 1010;
	  TIM2->CCR2 = dc;

}

// Motor Esquerdo
void dcB(){

	  dc = 1005;
	  TIM2->CCR3 = dc;

}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



void delay (uint16_t time) {
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while (__HAL_TIM_GET_COUNTER(&htim1) < time);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
//ESRQUERDA
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // if the interrupt source is channel1
	{
		if (Is_First_CapturedE==0) // if the first value is not captured
		{
			IC_Val1E = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
			Is_First_CapturedE = 1;  // set the first captured as true
			// Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else if (Is_First_CapturedE==1)   // if the first is already captured
		{
			IC_Val2E = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // read second value
			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

			if (IC_Val2E > IC_Val1E)
			{
				DifferenceE = IC_Val2E-IC_Val1E;
			}

			else if (IC_Val1E > IC_Val2E)
			{
				DifferenceE = (0xffff - IC_Val1E) + IC_Val2E;
			}

			DistanceE = DifferenceE * .034/2;
			Is_First_CapturedE = 0; // set it back to false

			// set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);
		}
	}

//	CENTRAL

	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)  // if the interrupt source is channel1
			{
				if (Is_First_CapturedC==0) // if the first value is not captured
				{
					IC_Val1C = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2); // read the first value
					Is_First_CapturedC = 1;  // set the first captured as true
					// Now change the polarity to falling edge
					__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
				}

				else if (Is_First_CapturedC==1)   // if the first is already captured
				{
					IC_Val2C = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);  // read second value
					__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

					if (IC_Val2C > IC_Val1C)
					{
						DifferenceC = IC_Val2C-IC_Val1C;
					}

					else if (IC_Val1C > IC_Val2C)
					{
						DifferenceC = (0xffff - IC_Val1C) + IC_Val2C;
					}

					DistanceC = DifferenceC * .034/2;
					Is_First_CapturedC = 0; // set it back to false

					// set polarity to rising edge
					__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
					__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC2);
				}
			}
//	DIREITA

	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)  // if the interrupt source is channel1
				{
					if (Is_First_CapturedD==0) // if the first value is not captured
					{
						IC_Val1D = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3); // read the first value
						Is_First_CapturedD = 1;  // set the first captured as true
						// Now change the polarity to falling edge
						__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);
					}

					else if (Is_First_CapturedD==1)   // if the first is already captured
					{
						IC_Val2D = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);  // read second value
						__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

						if (IC_Val2D > IC_Val1D)
						{
							DifferenceD = IC_Val2D-IC_Val1D;
						}

						else if (IC_Val1D > IC_Val2D)
						{
							DifferenceD = (0xffff - IC_Val1D) + IC_Val2D;
						}

						DistanceD = DifferenceD * .034/2;
						Is_First_CapturedD = 0; // set it back to false

						// set polarity to rising edge
						__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
						__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC3);
					}
				}



}

void HCSR04_ReadE (void)
{
//	ESQUERDO
	HAL_GPIO_WritePin(TriggerE_GPIO_Port, TriggerE_Pin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	delay(10);  // wait for 10 us
	HAL_GPIO_WritePin(TriggerE_GPIO_Port, TriggerE_Pin, GPIO_PIN_RESET);  // pull the TRIG pin low

	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);

}

void HCSR04_ReadC (void)
{

//	CENTRAL
	HAL_GPIO_WritePin(TriggerC_GPIO_Port, TriggerC_Pin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	delay(10);  // wait for 10 us
	HAL_GPIO_WritePin(TriggerC_GPIO_Port, TriggerC_Pin, GPIO_PIN_RESET);  // pull the TRIG pin low

	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC2);


}

void HCSR04_ReadD (void)
{


	//	Direito
	HAL_GPIO_WritePin(TriggerD_GPIO_Port, TriggerD_Pin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	delay(10);  // wait for 10 us
	HAL_GPIO_WritePin(TriggerD_GPIO_Port, TriggerD_Pin, GPIO_PIN_RESET);  // pull the TRIG pin low
	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC3);

}

/*
void Andar(void ){

	dcA();											 // PWM A - Direito
	dcB();											 // PWM B - Esquerdo
	HAL_GPIO_WritePin(IN12_GPIO_Port, IN2_Pin, SET); // Liga motor A - Direito
	HAL_GPIO_WritePin(IN34_GPIO_Port, IN4_Pin, SET); // Liga motor B - Esquerdo

	osDelay(200);

	HAL_GPIO_WritePin(IN12_GPIO_Port, IN2_Pin, RESET); // Desliga motor A - Direito
	HAL_GPIO_WritePin(IN34_GPIO_Port, IN4_Pin, RESET); // Desliga motor B - Esquerdo


	HAL_GPIO_WritePin(IN12_GPIO_Port, IN1_Pin, SET); // Liga motor A - Direito
	HAL_GPIO_WritePin(IN34_GPIO_Port, IN3_Pin, SET); // Liga motor B - Esquerdo


	osDelay(200);

	HAL_GPIO_WritePin(IN12_GPIO_Port, IN1_Pin, RESET); // Desliga motor A - Direito
	HAL_GPIO_WritePin(IN34_GPIO_Port, IN3_Pin, RESET); // Desliga motor B - Esquerdo

	osDelay(1);
}
*/

void ResetarMotor(){

	HAL_GPIO_WritePin(IN12_GPIO_Port, IN1_Pin, RESET);
	HAL_GPIO_WritePin(IN12_GPIO_Port, IN2_Pin, RESET);
	HAL_GPIO_WritePin(IN34_GPIO_Port, IN3_Pin, RESET);
	HAL_GPIO_WritePin(IN34_GPIO_Port, IN4_Pin, RESET);
	ligaencoderE=0;
	ligaencoderD=0;
}

void LigarMotorFrente(){

	dcA();											 // PWM A - Direito
	dcB();											 // PWM B - Esquerdo
	HAL_GPIO_WritePin(IN12_GPIO_Port, IN1_Pin, RESET);
	HAL_GPIO_WritePin(IN12_GPIO_Port, IN2_Pin, SET); // Liga motor A - Direito
	HAL_GPIO_WritePin(IN34_GPIO_Port, IN3_Pin, RESET);
	HAL_GPIO_WritePin(IN34_GPIO_Port, IN4_Pin, SET); // Liga motor B - Esquerdo
	ligaencoderE=0;
	ligaencoderD=0;
}

void LigarMotorRe(){

	dcA();											 // PWM A - Direito
	dcB();											 // PWM B - Esquerdo
	HAL_GPIO_WritePin(IN12_GPIO_Port, IN1_Pin, SET); // Liga motor A - Direito
	HAL_GPIO_WritePin(IN12_GPIO_Port, IN2_Pin, RESET);
	HAL_GPIO_WritePin(IN34_GPIO_Port, IN3_Pin, SET); // Liga motor B - Esquerdo
	HAL_GPIO_WritePin(IN34_GPIO_Port, IN4_Pin, RESET);
	ligaencoderE=0;
	ligaencoderD=0;
}

void VirarE(){

	dcA();											 // PWM A - Direito
	dcB();											 // PWM B - Esquerdo
	ligaencoderE = 1;
	HAL_GPIO_WritePin(IN12_GPIO_Port, IN1_Pin, RESET); // Liga motor A - Direito
	HAL_GPIO_WritePin(IN12_GPIO_Port, IN2_Pin, SET); // Liga motor A - Direito
	HAL_GPIO_WritePin(IN34_GPIO_Port, IN3_Pin, SET); // Liga motor B - Esquerdo
	HAL_GPIO_WritePin(IN34_GPIO_Port, IN4_Pin, RESET); // Liga motor B - Esquerdo

}


void VirarD(){

	dcA();											 // PWM A - Direito
	dcB();											 // PWM B - Esquerdo
	ligaencoderD = 1;
	HAL_GPIO_WritePin(IN12_GPIO_Port, IN1_Pin, SET); // Liga motor A - Direito
	HAL_GPIO_WritePin(IN12_GPIO_Port, IN2_Pin, RESET); // Liga motor A - Direito
	HAL_GPIO_WritePin(IN34_GPIO_Port, IN3_Pin, RESET); // Liga motor B - Esquerdo
	HAL_GPIO_WritePin(IN34_GPIO_Port, IN4_Pin, SET); // Liga motor B - Esquerdo

}


void EncoderD(){


	if (ligaencoderD == 1){
	estadoAnteriorA = estadoAtualA;
	  osDelay(1);

	  if(HAL_GPIO_ReadPin(EncoderA_GPIO_Port,EncoderA_Pin) == GPIO_PIN_SET){
		  estadoAtualA = 0;

	  }else if(HAL_GPIO_ReadPin(EncoderA_GPIO_Port,EncoderA_Pin) == GPIO_PIN_RESET){
		  estadoAtualA = 1;
	  }

	  if(estadoAtualA != estadoAnteriorA){
		  pulsoA++;

	  }

	  if(pulsoA == pulsos_por_voltaA){

		  HAL_GPIO_TogglePin(LedD_GPIO_Port, LedD_Pin);
		  pulsoA = 0;
		  grausA = 1;
		  osDelay(1);
	  }

	  osDelay(1);
	}
}



void EncoderE(){


	if (ligaencoderE == 1){
	  estadoAnteriorB = estadoAtualB;
	  osDelay(1);

	  if(HAL_GPIO_ReadPin(EncoderB_GPIO_Port,EncoderB_Pin) == GPIO_PIN_SET){
		  estadoAtualB = 0;

	  }else if(HAL_GPIO_ReadPin(EncoderB_GPIO_Port,EncoderB_Pin) == GPIO_PIN_RESET){
	  	  estadoAtualB = 1;
	  }

	  if(estadoAtualB != estadoAnteriorB){
		  pulsoB++;

	  }

	  if(pulsoB == pulsos_por_voltaB){

		  HAL_GPIO_TogglePin(LedE_GPIO_Port, LedE_Pin);
		  pulsoB = 0;
		  grausB = 1;
		  osDelay(1);
	  }

  osDelay(1);
	}
}



/* Interrupção Externa (Falling Edge): Incrementa contador do Encoder */
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
//
//	if (GPIO_Pin == ButtonA_Pin){
//	//	Start = !Start;
//		HAL_GPIO_TogglePin(LedC_GPIO_Port, LedC_Pin);
//	}
//
////	if (GPIO_Pin == EncoderA_Pin){
////		pulsoA++;
//////		HAL_GPIO_TogglePin(LedD_GPIO_Port, LedD_Pin);
////
////	}
////	if (GPIO_Pin == EncoderB_Pin){
//////		HAL_GPIO_TogglePin(LedE_GPIO_Port, LedE_Pin);
////		pulsoB++;
////
////	}
//	else{
//		__NOP ();
//	}
//}

//void volta (){
//
//
//
//
//
//}
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start(&htim2, PWM_MotorA);
  HAL_TIM_PWM_Start(&htim2, PWM_MotorB);
  HAL_TIM_PWM_Start(&htim2, Buzzer);

  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of Estado1 */
  osThreadDef(Estado1, StartEstado1, osPriorityRealtime, 0, 128);
  Estado1Handle = osThreadCreate(osThread(Estado1), NULL);

  /* definition and creation of Estado2 */
  osThreadDef(Estado2, StartEstado2, osPriorityRealtime, 0, 128);
  Estado2Handle = osThreadCreate(osThread(Estado2), NULL);

  /* definition and creation of Estado3 */
  osThreadDef(Estado3, StartEstado3, osPriorityRealtime, 0, 128);
  Estado3Handle = osThreadCreate(osThread(Estado3), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 15;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xffff-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, IN4_Pin|IN3_Pin|TriggerC_Pin|TriggerD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, IN2_Pin|IN1_Pin|TriggerE_Pin|LedE_Pin
                          |LedC_Pin|LedD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Led_Pin */
  GPIO_InitStruct.Pin = Led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Led_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ButtonA_Pin */
  GPIO_InitStruct.Pin = ButtonA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ButtonA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EncoderB_Pin EncoderA_Pin */
  GPIO_InitStruct.Pin = EncoderB_Pin|EncoderA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : IN4_Pin IN3_Pin TriggerC_Pin TriggerD_Pin */
  GPIO_InitStruct.Pin = IN4_Pin|IN3_Pin|TriggerC_Pin|TriggerD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : IN2_Pin IN1_Pin TriggerE_Pin LedE_Pin
                           LedC_Pin LedD_Pin */
  GPIO_InitStruct.Pin = IN2_Pin|IN1_Pin|TriggerE_Pin|LedE_Pin
                          |LedC_Pin|LedD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ButtonB_Pin */
  GPIO_InitStruct.Pin = ButtonB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ButtonB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : InfraRedA_Pin InfraRedB_Pin */
  GPIO_InitStruct.Pin = InfraRedA_Pin|InfraRedB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartEstado1 */
/**
  * @brief  Function implementing the Estado1 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartEstado1 */


void StartEstado1(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {

	  HCSR04_ReadC();
//	  HCSR04_ReadD();
//	  HCSR04_ReadE();



    osDelay(20);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartEstado2 */
/**
* @brief Function implementing the Estado2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartEstado2 */
void StartEstado2(void const * argument)
{
  /* USER CODE BEGIN StartEstado2 */
  /* Infinite loop */
  for(;;)
  {
	  if(HAL_GPIO_ReadPin(ButtonB_GPIO_Port, ButtonB_Pin)==0){


		  if(isGoalA !=1){//aqui ele nao encontrou o objetivo
			  HAL_GPIO_TogglePin(LedC_GPIO_Port, LedC_Pin);


			  if (DistanceC >= 20) {


					  LigarMotorFrente();
					 osDelay(1);

			  }
			  if  (DistanceC < 20) {


	//				  HAL_GPIO_WritePin(LedC_GPIO_Port, LedC_Pin, GPIO_PIN_RESET);
					  aux = 0;
					  ResetarMotor();
					  VirarE();
					  osDelay(500);

					  if (grausA ==1){
						  ResetarMotor();
						  osDelay(500);
						  grausA=0;
					}

			  }



		  }else{

		//		  HAL_GPIO_TogglePin(Led_GPIO_Port, Led_Pin);
				  HAL_GPIO_TogglePin(LedE_GPIO_Port, LedE_Pin);
//				  ResetarMotor();
				  osDelay(500);



			  if (isGoalB==1){

				  HAL_GPIO_TogglePin(LedD_GPIO_Port, LedD_Pin);
				  ResetarMotor();
				  int DimensionMusical3 = sizeof(melodia3)/sizeof(melodia3[0]);
				  for (int i = 0; i< DimensionMusical3; i++){

				  	TIM2->CCR4 = melodia3[i];
				  	osDelay(tempo3[i]);
				  	TIM2->CCR4 = 0;
				  	osDelay(40);

				  }

				  osDelay(500);

			  }

		  }
  }else if(HAL_GPIO_ReadPin(ButtonB_GPIO_Port, ButtonB_Pin)==1){




	  LigarMotorFrente();
	  osDelay(600);
	  LigarMotorRe();
	  osDelay(600);
	  ResetarMotor();

  }


  /* USER CODE END StartEstado2 */
}
}

/* USER CODE BEGIN Header_StartEstado3 */
/**
* @brief Function implementing the Estado3 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartEstado3 */
void StartEstado3(void const * argument)
{
//	essa task vai ler o encoder
  /* USER CODE BEGIN StartEstado3 */
  /* Infinite loop */
  for(;;)
  {

	  if(HAL_GPIO_ReadPin(InfraRedA_GPIO_Port, InfraRedA_Pin)==0){
//		  HAL_GPIO_WritePin(LedD_GPIO_Port, LedD_Pin, GPIO_PIN_RESET);
		  isGoalA=0;

	  }else{

//		  HAL_GPIO_WritePin(LedD_GPIO_Port, LedD_Pin, GPIO_PIN_SET);
		  isGoalA=1;
	  }

//	  infraRedB é o de tras

	  if(HAL_GPIO_ReadPin(InfraRedB_GPIO_Port, InfraRedB_Pin)==0){
//	  		  HAL_GPIO_WritePin(LedE_GPIO_Port, LedE_Pin, GPIO_PIN_RESET);
	  		isGoalB=0;

	  	  }else{

//	  		  HAL_GPIO_WritePin(LedE_GPIO_Port, LedE_Pin, GPIO_PIN_SET);
	  		isGoalB=1;
	  	  }



    osDelay(100);
  }
  /* USER CODE END StartEstado3 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
