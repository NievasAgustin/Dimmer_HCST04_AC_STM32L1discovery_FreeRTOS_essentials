/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "math.h"
#include "lcd16x2.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct {
	uint8_t 			captureIdx;
	uint32_t 			edge1Time;
	uint32_t 			edge2Time;
	uint8_t 			icFlag;
	float 				distance;
}HCSR04;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define PI_short 3.14159265
#define Dos_311 0.00643087
#define speedOfSound 0.0343/2;


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

osThreadId Task_HCSR04Handle;
uint32_t Task_HCSR04Buffer[ 128 ];
osStaticThreadDef_t Task_HCSR04ControlBlock;
osThreadId Task_MATHHandle;
uint32_t Task_MATHBuffer[ 128 ];
osStaticThreadDef_t Task_MATHControlBlock;
osThreadId Task_LCDHandle;
uint32_t Task_LCDBuffer[ 128 ];
osStaticThreadDef_t Task_LCDControlBlock;
osThreadId Task_DIACHandle;
uint32_t Task_DIACBuffer[ 128 ];
osStaticThreadDef_t Task_DIACControlBlock;
osThreadId Task_FSMHandle;
uint32_t Task_FSMBuffer[ 128 ];
osStaticThreadDef_t Task_FSMControlBlock;
osMessageQId Queue_FSM_to_DIACHandle;
uint8_t Queue_FSM_to_DIACBuffer[ 16 * sizeof( uint16_t ) ];
osStaticMessageQDef_t Queue_FSM_to_DIACControlBlock;
osMessageQId Queue_HCST04_to_MATHHandle;
uint8_t Queue_HCST04_to_MATHBuffer[ 16 * sizeof( float ) ];
osStaticMessageQDef_t Queue_HCST04_to_MATHControlBlock;
osMessageQId Queue_DIAC_to_LCDHandle;
uint8_t Queue_DIAC_to_LCDBuffer[ 16 * sizeof( float ) ];
osStaticMessageQDef_t Queue_DIAC_to_LCDControlBlock;
osMessageQId Queue_MATH_to_DIACHandle;
uint8_t Queue_MATH_to_DIACBuffer[ 16 * sizeof( float ) ];
osStaticMessageQDef_t Queue_MATH_to_DIACControlBlock;
osMessageQId Queue_IC_to_HCST04Handle;
uint8_t Queue_IC_to_HCST04Buffer[ 16 * sizeof( uint32_t ) ];
osStaticMessageQDef_t Queue_IC_to_HCST04ControlBlock;
osMessageQId Queue_MATH_to_LCDHandle;
uint8_t Queue_MATH_to_LCDBuffer[ 16 * sizeof( uint32_t ) ];
osStaticMessageQDef_t Queue_MATH_to_LCDControlBlock;
osSemaphoreId BinarySem_FSMHandle;
osStaticSemaphoreDef_t BinarySem_FSMControlBlock;
/* USER CODE BEGIN PV */

osSemaphoreId BinarySem_LCDHandle;
osStaticSemaphoreDef_t BinarySem_LCDControlBlock;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C1_Init(void);
void StartTask_HCSR04(void const * argument);
void StartTask_MATH(void const * argument);
void StartTask_LCD(void const * argument);
void StartTask_DIAC(void const * argument);
void StartTask_FSM(void const * argument);

static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/

/* USER CODE BEGIN 0 */
osEvent Queue_HCSR04_Recive_on_task;
osEvent Queue_MATH_Recive_on_task;
osEvent Queue_LCD_Recive_on_task;


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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_I2C1_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  lcd16x2_i2c_init(&hi2c1);

  lcd16x2_i2c_clear();
  lcd16x2_i2c_1stLine();
  lcd16x2_i2c_printf("Hola Profe");
  lcd16x2_i2c_2ndLine();
  lcd16x2_i2c_printf("Esto Si anda");



  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of BinarySem_FSM */
  osSemaphoreStaticDef(BinarySem_FSM, &BinarySem_FSMControlBlock);
  BinarySem_FSMHandle = osSemaphoreCreate(osSemaphore(BinarySem_FSM), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */

  /* definition and creation of BinarySem_FSM */
  osSemaphoreStaticDef(BinarySem_LCD, &BinarySem_LCDControlBlock);				//Este semaforo no se utiliza al final.
  BinarySem_LCDHandle = osSemaphoreCreate(osSemaphore(BinarySem_LCD), 1);

  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of Queue_FSM_to_DIAC */
  osMessageQStaticDef(Queue_FSM_to_DIAC, 16, uint16_t, Queue_FSM_to_DIACBuffer, &Queue_FSM_to_DIACControlBlock);
  Queue_FSM_to_DIACHandle = osMessageCreate(osMessageQ(Queue_FSM_to_DIAC), NULL);

  /* definition and creation of Queue_HCST04_to_MATH */
  osMessageQStaticDef(Queue_HCST04_to_MATH, 16, float, Queue_HCST04_to_MATHBuffer, &Queue_HCST04_to_MATHControlBlock);
  Queue_HCST04_to_MATHHandle = osMessageCreate(osMessageQ(Queue_HCST04_to_MATH), NULL);

  /* definition and creation of Queue_DIAC_to_LCD */
  osMessageQStaticDef(Queue_DIAC_to_LCD, 16, float, Queue_DIAC_to_LCDBuffer, &Queue_DIAC_to_LCDControlBlock);
  Queue_DIAC_to_LCDHandle = osMessageCreate(osMessageQ(Queue_DIAC_to_LCD), NULL);

  /* definition and creation of Queue_MATH_to_DIAC */
  osMessageQStaticDef(Queue_MATH_to_DIAC, 16, float, Queue_MATH_to_DIACBuffer, &Queue_MATH_to_DIACControlBlock);
  Queue_MATH_to_DIACHandle = osMessageCreate(osMessageQ(Queue_MATH_to_DIAC), NULL);

  /* definition and creation of Queue_IC_to_HCST04 */
  osMessageQStaticDef(Queue_IC_to_HCST04, 16, uint32_t, Queue_IC_to_HCST04Buffer, &Queue_IC_to_HCST04ControlBlock);
  Queue_IC_to_HCST04Handle = osMessageCreate(osMessageQ(Queue_IC_to_HCST04), NULL);

  /* definition and creation of Queue_MATH_to_LCD */
  osMessageQStaticDef(Queue_MATH_to_LCD, 16, uint32_t, Queue_MATH_to_LCDBuffer, &Queue_MATH_to_LCDControlBlock);
  Queue_MATH_to_LCDHandle = osMessageCreate(osMessageQ(Queue_MATH_to_LCD), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */




  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of Task_HCSR04 */
  osThreadStaticDef(Task_HCSR04, StartTask_HCSR04, osPriorityNormal, 0, 128, Task_HCSR04Buffer, &Task_HCSR04ControlBlock);
  Task_HCSR04Handle = osThreadCreate(osThread(Task_HCSR04), NULL);

  /* definition and creation of Task_MATH */
  osThreadStaticDef(Task_MATH, StartTask_MATH, osPriorityHigh, 0, 128, Task_MATHBuffer, &Task_MATHControlBlock);
  Task_MATHHandle = osThreadCreate(osThread(Task_MATH), NULL);

  /* definition and creation of Task_LCD */
  osThreadStaticDef(Task_LCD, StartTask_LCD, osPriorityBelowNormal, 0, 128, Task_LCDBuffer, &Task_LCDControlBlock);
  Task_LCDHandle = osThreadCreate(osThread(Task_LCD), NULL);

  /* definition and creation of Task_DIAC */
  osThreadStaticDef(Task_DIAC, StartTask_DIAC, osPriorityBelowNormal, 0, 128, Task_DIACBuffer, &Task_DIACControlBlock);
  Task_DIACHandle = osThreadCreate(osThread(Task_DIAC), NULL);

  /* definition and creation of Task_FSM */
  osThreadStaticDef(Task_FSM, StartTask_FSM, osPriorityAboveNormal, 0, 128, Task_FSMBuffer, &Task_FSMControlBlock);
  Task_FSMHandle = osThreadCreate(osThread(Task_FSM), NULL);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(TIM4_IRQn);
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 32-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 16;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 32-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 3500;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim3, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 2000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 32-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 5;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : Pulsador_FSM_EXTI_Pin */
  GPIO_InitStruct.Pin = Pulsador_FSM_EXTI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(Pulsador_FSM_EXTI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Zero_Cross_EXTI_Pin */
  GPIO_InitStruct.Pin = Zero_Cross_EXTI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Zero_Cross_EXTI_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

}

/* USER CODE BEGIN 4 */


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	switch(GPIO_Pin){
	case Zero_Cross_EXTI_Pin:

	      HAL_TIM_PWM_Stop_IT(&htim3, TIM_CHANNEL_2);
	      HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_2);

	 	 break;
	case Pulsador_FSM_EXTI_Pin:
		osSemaphoreRelease(BinarySem_FSMHandle);
		 break;

	}

}



void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){

	static HCSR04 ultrasonido;
	uint32_t distanciacm = 0;

	if(htim->Instance == TIM4){
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){


			if(ultrasonido.captureIdx == 0) //Fisrt edge
			{
				ultrasonido.edge1Time = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); //__HAL_TIM_GetCounter(&htim3);    using TIM4 Channel 1

				TIM4->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC1NP);
				TIM4->CCER |= (TIM_INPUTCHANNELPOLARITY_FALLING & (TIM_CCER_CC1P | TIM_CCER_CC1NP));


				ultrasonido.captureIdx = 1;
			}
			else //Second edge
			{
			ultrasonido.edge2Time = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			ultrasonido.captureIdx = 0;
			TIM4->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC1NP);
			TIM4->CCER |= (TIM_INPUTCHANNELPOLARITY_RISING & (TIM_CCER_CC1P | TIM_CCER_CC1NP));

			if(ultrasonido.edge2Time > ultrasonido.edge1Time )
				{

				distanciacm = (ultrasonido.edge2Time - ultrasonido.edge1Time);


				osMessagePut(Queue_IC_to_HCST04Handle, distanciacm, 0);

			}
		}
	}
}

}



/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartTask_HCSR04 */
/**
  * @brief  Function implementing the Task_HCSR04 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTask_HCSR04 */
void StartTask_HCSR04(void const * argument)
{
  /* USER CODE BEGIN 5 */

	float_t	muestraN[5];
	uint8_t	i=0;
	float_t	prom;

  /* Infinite loop */
  for(;;)
  {
	  Queue_HCSR04_Recive_on_task=osMessageGet(Queue_IC_to_HCST04Handle, osWaitForever);
	  	     if (Queue_HCSR04_Recive_on_task.status == osEventMessage){						//Se puede colocar un filtro como en la Task_MATH
	  	     	muestraN[i]=((float_t)Queue_HCSR04_Recive_on_task.value.v)/100;
	  	     	i++;
	  	     }
	  	     if(i==4){
	  	    	 i=0;
	  	    	 prom=0;
	  	    	 for (int muestra = 0; muestra < 5; ++muestra) {
	  				prom=muestraN[muestra]+prom;
	  			}
	  	    	 prom=prom/5;
	  	    	 prom=prom*100;
	  	    	 osMessagePut(Queue_HCST04_to_MATHHandle, (uint32_t)prom, 0);
	  	     	 	 }
 	}


  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask_MATH */
/**
* @brief Function implementing the Task_MATH thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_MATH */
void StartTask_MATH(void const * argument)
{
  /* USER CODE BEGIN StartTask_MATH */
	static	uint32_t porcentaje =0;
	static	uint32_t tiempoTriac =0.0;

	static const uint32_t linealizacion[101]={0, 14, 18, 21, 24, 26, 27, 29, 30, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 44, 45, 46, 47, 48, 48, 49, 50, 50, 51, 52, 53, 53, 54, 55, 55, 56, 56, 57, 58, 58,
			59, 60, 60, 61, 61, 62, 62,	63,	64,	64,	65,	66,	66,	67,	67,	68,	69,	69,	70,	70,	71,	72,	72,	73,	73,	74,	74,	75,	76,	76,	77,	78,	78,	79,	79,	80,	81,	82,	82,	83,	83,	84,	85,	85,	86,	87,	88,	88,	89,	90,	91,
			92,	93,	94,	95,	96,	98, 100};


	static	const uint32_t liminf = 3;
	static	const uint32_t limsup = 30;
	static	float_t	distanciamath = 15.0;

  /* Infinite loop */
  for(;;)
  {
    Queue_MATH_Recive_on_task=osMessageGet(Queue_HCST04_to_MATHHandle, osWaitForever);
    if (Queue_MATH_Recive_on_task.status == osEventMessage){
    	distanciamath=((float_t)Queue_MATH_Recive_on_task.value.v)/100;			//58 para centimetros de verdad, pero con 100 es mejor para el usuario

    	  if ( liminf< distanciamath || distanciamath<limsup){
    		  distanciamath = distanciamath - liminf;
    		  porcentaje = (uint32_t)(distanciamath *100 / (limsup-liminf));
    	  }
    	  if (porcentaje<101){

    		  tiempoTriac=100*linealizacion[100-(uint32_t)porcentaje];	//Esta en orden inverso el array de linealizacion

    		  osMessagePut(Queue_MATH_to_DIACHandle, tiempoTriac, 0);
    		  osMessagePut(Queue_MATH_to_LCDHandle, porcentaje, 0);

		}

    }



  }

  /* USER CODE END StartTask_MATH */
}

/* USER CODE BEGIN Header_StartTask_LCD */
/**
* @brief Function implementing the Task_LCD thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_LCD */
void StartTask_LCD(void const * argument)
{
  /* USER CODE BEGIN StartTask_LCD */
	volatile uint32_t LCD_all=0;
	static volatile uint32_t porcentaje=0;

	uint16_t xReceivedFromQueue_DIAC_to_LCDHandle=1;
	uint32_t xReceivedFromQueue_MATH_to_LCDHandle=0;

	static QueueSetHandle_t QueueSet_LCD;

	QueueSet_LCD = xQueueCreateSet(32);
	xQueueAddToSet(Queue_MATH_to_LCDHandle, QueueSet_LCD);
	xQueueAddToSet(Queue_DIAC_to_LCDHandle, QueueSet_LCD);

	QueueSetMemberHandle_t xActivatedMember_LCD;

	/* Infinite loop */
  for(;;)
  {
	  	  xActivatedMember_LCD = xQueueSelectFromSet( QueueSet_LCD,osWaitForever);

	  	  if(xActivatedMember_LCD==Queue_MATH_to_LCDHandle){
	  		  xQueueReceive( xActivatedMember_LCD, &xReceivedFromQueue_MATH_to_LCDHandle, 0 );

	  		  porcentaje= xReceivedFromQueue_MATH_to_LCDHandle ;
	  	  }else{
	    		  xQueueReceive( xActivatedMember_LCD, &xReceivedFromQueue_DIAC_to_LCDHandle, 0 );

	    		  LCD_all= xReceivedFromQueue_DIAC_to_LCDHandle;
	    	  }


	      	switch(LCD_all){
	      		case 1:
	      			lcd16x2_i2c_clear();
	      			lcd16x2_i2c_1stLine();
	      			lcd16x2_i2c_printf("Buenas");
	      			lcd16x2_i2c_2ndLine();
	      			lcd16x2_i2c_printf("Luz On");
	      			break;
	      		case 2:
	      			lcd16x2_i2c_clear();
	      			lcd16x2_i2c_1stLine();
	      			lcd16x2_i2c_printf("Modo Dinamico");
	      			lcd16x2_i2c_2ndLine();
	      			lcd16x2_i2c_printf("Luz %d pot.", porcentaje);
	      			break;
	      		case 3:
	      			lcd16x2_i2c_clear();
	      			lcd16x2_i2c_1stLine();
	      			lcd16x2_i2c_printf("Adios");
	      			lcd16x2_i2c_2ndLine();
	      			lcd16x2_i2c_printf("Luz Off");
	      			break;
	      		default:
	      			lcd16x2_i2c_clear();
	      			lcd16x2_i2c_1stLine();
	      			lcd16x2_i2c_printf("Error %d", LCD_all );
	      			lcd16x2_i2c_2ndLine();
	      			lcd16x2_i2c_printf("Luz On");
	      			break;
	      	}


  }
  /* USER CODE END StartTask_LCD */
}

/* USER CODE BEGIN Header_StartTask_DIAC */
/**
* @brief Function implementing the Task_DIAC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_DIAC */
void StartTask_DIAC(void const * argument)
{
  /* USER CODE BEGIN StartTask_DIAC */
	uint16_t xReceivedFromQueue_FSM_to_DIACHandle=1;
	uint32_t xReceivedFromQueue_MATH_to_DIACCHandle=0;
	uint16_t estado;
//	static float *pulsetimepointer;

	static QueueSetHandle_t QueueSet;
	QueueSet = xQueueCreateSet(32);
	xQueueAddToSet(Queue_FSM_to_DIACHandle, QueueSet);
	xQueueAddToSet(Queue_MATH_to_DIACHandle, QueueSet);

	QueueSetMemberHandle_t xActivatedMember;

	uint32_t PULSE = 0;

  /* Infinite loop */
  for(;;)
  {
	  xActivatedMember = xQueueSelectFromSet( QueueSet,osWaitForever);

	  if(xActivatedMember==Queue_FSM_to_DIACHandle){
		  xQueueReceive( xActivatedMember, &xReceivedFromQueue_FSM_to_DIACHandle, 0 );

		  estado= xReceivedFromQueue_FSM_to_DIACHandle ;
		  osMessagePut(Queue_DIAC_to_LCDHandle, estado, 0);

	  }else{
  		  xQueueReceive( xActivatedMember, &xReceivedFromQueue_MATH_to_DIACCHandle, 0 );

  		  PULSE= xReceivedFromQueue_MATH_to_DIACCHandle;
  	  }

	  switch(estado){
	  case 1:
		  TIM3->CCR2 = 700;
		  TIM3->ARR = 700+1500;
		  break;
	  case 2:

		  TIM3->CCR2 = PULSE;
		  TIM3->ARR = PULSE+1500;
		  break;
	  case 3:
		  TIM3->CCR2 = 10000;
		  TIM3->ARR = 10000+1000;
		  break;
	  default:
		  TIM3->CCR2 = 7500;
		  TIM3->ARR = 7500+1500;
	  	  break;
	  }



  }
  /* USER CODE END StartTask_DIAC */
}

/* USER CODE BEGIN Header_StartTask_FSM */
/**
* @brief Function implementing the Task_FSM thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_FSM */
void StartTask_FSM(void const * argument)
{
  /* USER CODE BEGIN StartTask_FSM */

	static uint16_t counter=0;

	/* Infinite loop */
  for(;;)
  {
    osSemaphoreWait(BinarySem_FSMHandle, osWaitForever);				//Posible usar Binario contador?
    counter++;
    switch (counter){
    	case 1:
    		HAL_TIM_IC_Stop_IT(&htim4, TIM_CHANNEL_1);
    		HAL_TIM_PWM_Stop_IT(&htim2, TIM_CHANNEL_3);
    		break;
    	case 2:
    		HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
    		HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_3);

    		break;
    	case 3:
    		HAL_TIM_IC_Stop_IT(&htim4, TIM_CHANNEL_1);
    		HAL_TIM_PWM_Stop_IT(&htim2, TIM_CHANNEL_3);
    		break;
    	default:
    		counter=1;
    		HAL_TIM_IC_Stop_IT(&htim4, TIM_CHANNEL_1);
    		HAL_TIM_PWM_Stop_IT(&htim2, TIM_CHANNEL_3);
		  break;
    				}

    osMessagePut(Queue_FSM_to_DIACHandle, counter, osWaitForever);				//Duda de inHandlerMode() si estoy bien y me va por FromISR bien o no.



  }
  /* USER CODE END StartTask_FSM */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM9 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM9) {
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

