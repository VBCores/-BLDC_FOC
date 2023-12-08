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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "AS5147.h"
#include "arm_math.h"
#include "my_helpers.h"
    
#include "Transform.h"
#include "math_ops.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct 
{
  float I_A;
  float I_B;
  float I_C;
  float I_D;
  float I_Q;
  float busV;
} Inverter_instance;

typedef enum {
  FOC, // main working mode
  ROTATE, // motor calibration mode
  CURRENT // set electrical angle to PI
} drive_modes ; 
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CPR 16384 // encoder Counts-Per-Revolution
#define ppairs 20u // number of motor rotor pole pairs
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim7;

/* USER CODE BEGIN PV */
extern uint64_t TIM7_ITs;

float IA_offset = 0.0f;
float IB_offset = 0.0f;
float IC_offset = 0.0f;

#define ADC_buf_size 4
uint32_t ADC1_buf[ADC_buf_size] = {0};

uint32_t SPI3_CS_PIN = SPI3_CS0_Pin;
GPIO_TypeDef * SPI3_CS_PORT = SPI3_CS0_GPIO_Port;

// these values are for references only. you should run a calibration procedure for your setup.
config_struct manual_data = {
  .lookup = {-5,-6,-7,-9,-10,-12,-13,-15,-16,-18,-19,-20,-21,-21,-22,-22,-21,-21,-20,-19,-18,-17,-16,-15,-14,-13,-13,-12,-11,-11,-11,-12,-12,-13,-14,-15,-16,-17,-19,-20,-21,-21,-22,-22,-22,-22,-21,-20,-19,-17,-15,-13,-11,-9,-8,-6,-4,-3,-2,-2,-2,-3,-4,-5,-7,-9,-11,-14,-16,-18,-21,-23,-25,-26,-27,-28,-28,-28,-27,-26,-25,-23,-22,-20,-18,-17,-15,-14,-13,-13,-13,-13,-13,-14,-15,-16,-17,-19,-20,-21,-22,-23,-24,-24,-24,-24,-23,-22,-21,-19,-18,-16,-14,-12,-10,-8,-7,-5,-4,-3,-2,-2,-1,-1,-2,-2,-3,-4},
  .elec_offset = 775,
  .gear_ratio = 1,
};

config_struct * config = &manual_data;
float ext_enc_mech_zero = 0.0f;

uint16_t SPI_TX_MSG = ANGLECOM | 0x4000 | 0x8000; // SPI TX message to encoder. 

uint16_t encoder_raw_angle = 0; // stores the most recent reading from the encoder

// structures to store drive variables
DriverState_instance Dev = { .State = FOC }; // see drive_modes
Inverter_instance Inv;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI3_Init(void);
static void MX_CORDIC_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma optimize s=none
uint64_t micros()
#elif defined ( __GNUC__ ) /*!< GNU Compiler */
uint64_t __attribute__((optimize("O0"))) micros()
#endif
{ 
  return (uint64_t)(__HAL_TIM_GET_COUNTER(&htim7) + 50000u * TIM7_ITs);
}

#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma optimize s=none
void micros_delay( uint64_t delay )
#elif defined ( __GNUC__ ) /*!< GNU Compiler */
void __attribute__((optimize("O0"))) micros_delay( uint64_t delay )
#endif
{
  uint64_t timestamp = micros();
  while( micros() < timestamp + delay );
}

float mfmod(float x, float y) { return x-((int)(x/y))*y; };
void calibration_sequence(void);
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
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI3_Init();
  MX_CORDIC_Init();
  MX_TIM1_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim7); // enable microseconds timesource

  // ADC clock settings result in ~1% error
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED); // apply factory ADC calibration settings
  HAL_ADC_Start_DMA(&hadc1, ADC1_buf, ADC_buf_size); // enable ADC and bind DMA to the memory buffer

  float accum1 = 0.0f;
  float accum2 = 0.0f;
  float accum3 = 0.0f;
  for( int i = 0; i < 100; i++ )
  {
    micros_delay( 1000 );
    accum1 += ( 3.3f*(float)ADC1_buf[1] / ( 16.0f*4096.0f ));
    accum2 += ( 3.3f*(float)ADC1_buf[2] / ( 16.0f*4096.0f ));
    accum3 += ( 3.3f*(float)ADC1_buf[3] / ( 16.0f*4096.0f ));
  }
  
  IA_offset = accum1 / 100.0f;
  IB_offset = accum2 / 100.0f;
  IC_offset = accum3 / 100.0f;
  
  // Configure CORDIC for fast calculation of sine & cosine for FOC transform functions. Offloads CPU.
  LL_CORDIC_Config(CORDIC, LL_CORDIC_FUNCTION_COSINE,   // cosine function 
                           LL_CORDIC_PRECISION_6CYCLES, // max precision for q1.31 cosine 
                           LL_CORDIC_SCALE_0,           // no scale 
                           LL_CORDIC_NBWRITE_1,         // One input data: angle. Second input data (modulus) is 1 after cordic reset 
                           LL_CORDIC_NBREAD_2,          // Two output data: cosine, then sine 
                           LL_CORDIC_INSIZE_32BITS,     // q1.31 format for input data 
                           LL_CORDIC_OUTSIZE_32BITS);   // q1.31 format for output data  

  // bind motor encoder
  SPI3_CS_PIN = SPI3_CS0_Pin;
  SPI3_CS_PORT = SPI3_CS0_GPIO_Port;
  
  // enable SPI3 peripheral for encoder
  LL_SPI_Enable(SPI3);

  // enable DRV8328B driver IC
  LL_GPIO_SetOutputPin(DRV_INLA_GPIO_Port, DRV_INLA_Pin);
  LL_GPIO_SetOutputPin(DRV_INLB_GPIO_Port, DRV_INLB_Pin);
  LL_GPIO_SetOutputPin(DRV_INLC_GPIO_Port, DRV_INLC_Pin);
  
  // cycle a few times (simetimes driver does not want to start right away)
  for( int i = 0; i < 3; i++)
  {
    LL_GPIO_ResetOutputPin(DRV_EN_GPIO_Port, DRV_EN_Pin);
    micros_delay(10000);
    LL_GPIO_SetOutputPin(DRV_EN_GPIO_Port, DRV_EN_Pin);
    micros_delay(10000);
  }
  
  // enable PWM generation for motor control
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // A-phase
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); // B-phase
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); // C-phase    
  
  // run encoder calibration sequense if needed 
  if( Dev.State == ROTATE )
  {
    __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_UPDATE); 
    calibration_sequence();
  }
  
  // enable SPI RX-Not-Empty interrupt. Encoder angle is read in interrupt now.
  LL_SPI_EnableIT_RXNE(SPI3); 

  // enable HAL_TIM_PeriodElapsedCallback. All realtime motor-controll calculations should be processed in this callback. 
  __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);  

  /* USER CODE END 2 */

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.OversamplingMode = ENABLE;
  hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_16;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_NONE;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CORDIC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CORDIC_Init(void)
{

  /* USER CODE BEGIN CORDIC_Init 0 */

  /* USER CODE END CORDIC_Init 0 */

  /* Peripheral clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_CORDIC);

  /* USER CODE BEGIN CORDIC_Init 1 */

  /* USER CODE END CORDIC_Init 1 */

  /* nothing else to be configured */

  /* USER CODE BEGIN CORDIC_Init 2 */

  /* USER CODE END CORDIC_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI3);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
  /**SPI3 GPIO Configuration
  PC10   ------> SPI3_SCK
  PC11   ------> SPI3_MISO
  PC12   ------> SPI3_MOSI
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_12;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* SPI3 interrupt Init */
  NVIC_SetPriority(SPI3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(SPI3_IRQn);

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_16BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_2EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV16;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 7;
  LL_SPI_Init(SPI3, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI3, LL_SPI_PROTOCOL_MOTOROLA);
  LL_SPI_DisableNSSPulseMgt(SPI3);
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 160;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 50000;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOF);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);

  /**/
  LL_GPIO_ResetOutputPin(DRV_INLA_GPIO_Port, DRV_INLA_Pin);

  /**/
  LL_GPIO_ResetOutputPin(DRV_INLB_GPIO_Port, DRV_INLB_Pin);

  /**/
  LL_GPIO_ResetOutputPin(DRV_INLC_GPIO_Port, DRV_INLC_Pin);

  /**/
  LL_GPIO_ResetOutputPin(SPI3_CS0_GPIO_Port, SPI3_CS0_Pin);

  /**/
  LL_GPIO_ResetOutputPin(DRV_EN_GPIO_Port, DRV_EN_Pin);

  /**/
  GPIO_InitStruct.Pin = DRV_INLA_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(DRV_INLA_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = DRV_INLB_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(DRV_INLB_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = DRV_INLC_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(DRV_INLC_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = SPI3_CS0_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(SPI3_CS0_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = DRV_EN_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(DRV_EN_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = DRV_FLT_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(DRV_FLT_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
float naive_velocity = 0.0f;

// https://hrcak.srce.hr/6760
void CalculateAngles(int32_t raw_angle, DriverState_instance *Driver)
{
  const float a = 0.0f; // expected acceleration, rad/s^2
  const float T = 25E-6; // encoder value sampling period, s
  
  const float g1 = 0.1f; // 0.015f;
  const float g2 = 75.0f; // 3.8f;
  const float g3 = 1000.0f; // 390.0f;
  
  static float Th_hat = 0.0f; // Theta hat, rad
  static float W_hat = 0.0f; // Omega hat, rad/s
  static float E_hat = 0.0f; // Epsilon hat, rad/s^2
  
  static float old_angle = 0.0f;

  // compensate encoder-magnet nonlinearity  error
  if( Driver->State != ROTATE ) 
  {
    raw_angle += config->lookup[raw_angle >> 8];
  }
  
  // elec_offset shows differense between encoder zero-point and electrical angle of Pi,
  // to get the electrical angle value we need to subtract this Pi = CPR/(2*ppairs) in electrical radians
  raw_angle += config->elec_offset;
  raw_angle -= CPR/(2*ppairs); 
  
  // limit the angle within the [0; CPR] segment 
  if( raw_angle > (CPR-1) ) {
    raw_angle -= CPR;
  }
  else if( raw_angle < 0 ) {
    raw_angle += CPR;
  }

  // rotor angular position calculation
  float mech_theta = raw_angle * (2.0f*PI/(float)CPR);
  
  // all this is floating-point math. 
  // (11)
  float nTh = Th_hat + W_hat*T + ( E_hat + a )*(T*T)/2.0f;
  float nW = W_hat + ( E_hat + a )*T;
  float nE = E_hat;
  
  // limit the angle within the [0; 2Pi ) segment 
  nTh = mfmod(nTh, 2.0f*PI);
  if( nTh < 0.0f ){
    nTh += 2.0f*PI;
  }

  float angle_error = mech_theta - nTh;
  
  // check rotation 
  if( angle_error > PI ){
    angle_error -= 2.0f*PI;
  }
  else if( angle_error < -PI ){
    angle_error += 2.0f*PI;
  }

  // (19)
  Th_hat = nTh + g1*angle_error;
  W_hat = nW + g2*angle_error;
  E_hat = nE + g3*angle_error;

  // electrical theta zero matches rotor angular position zero
  const float ab = 2.0f*PI / (float)ppairs;
  Driver->ElecTheta = (float)ppairs * mfmod( nTh, ab );

  // has the rotor made a full turn?
  if( old_angle - nTh > PI ){
    Driver->RotorTurns++;
  }
  else  if( old_angle - nTh < - PI ){
    Driver->RotorTurns--;
  }
  else
  {
    naive_velocity = (nTh - old_angle)/T;
  }
  
  old_angle = nTh;
  
  Driver->RotorAngle = nTh;
  Driver->ShaftVelocity = nW / (float)config->gear_ratio;
  Driver->ShaftAngle = (nTh + (float)Driver->RotorTurns*2.0f*PI) / (float)config->gear_ratio;
}

void ProcessADC(void)
{
  // take into account ADCs x16 hardware oversamiling.
  const float curr_sns_gain = 0.045f; // V/A
  Inv.I_A = (( 3.3f*(float)ADC1_buf[1] / ( 16.0f*4096.0f )) - IA_offset ) / curr_sns_gain;
  Inv.I_B = (( 3.3f*(float)ADC1_buf[2] / ( 16.0f*4096.0f )) - IB_offset ) / curr_sns_gain;
  Inv.I_C = (( 3.3f*(float)ADC1_buf[3] / ( 16.0f*4096.0f )) - IC_offset ) / curr_sns_gain;
}

#define TORQUE_CONST 1.0f // N*m/A // wrong value
#define max_torque 30.0f

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if( htim->Instance == TIM1 )
  {
    uint16_t DQA = 0;
    uint16_t DQB = 0;
    uint16_t DQC = 0;
    
    // commands encoder to start waiting for new command
    LL_GPIO_ResetOutputPin(SPI3_CS_PORT, SPI3_CS_PIN);
    
    // convert raw encoder angle reading into floats - electrical angle, mechanical angle, speeds etc.
    CalculateAngles(encoder_raw_angle, &Dev);

    // calculate sin and cos of electrical angle with the help of CORDIC.
    // convert electrical angle from float to q31. electrical theta should be [-pi, pi]
    int32_t ElecTheta_q31 = (int32_t)((Dev.ElecTheta/PI - 1.0f) * 2147483648.0f);
    // load angle value into CORDIC. Input value is in PIs!
    LL_CORDIC_WriteData(CORDIC, ElecTheta_q31);

    // read and process ADC readings from DMA linked buffer
    ProcessADC(); 
    Inv.busV = 12.0f*3.3f*ADC1_buf[0] / ( 16.0f*4096.0f ); // drive input voltage
    
    // commutate motor windings depending on drive current state
    if( Dev.State == FOC )
    {
      static float I_d_integral = 0.0f;
      static float I_q_integral = 0.0f;
      
      int32_t cosOutput = (int32_t)LL_CORDIC_ReadData(CORDIC); // Read cosine 
      int32_t sinOutput = (int32_t)LL_CORDIC_ReadData(CORDIC); // Read sine 

      // the values are negative to level out [-pi, pi] representation of electrical angle at the CORDIC input
      float c = -(float32_t)cosOutput / 2147483648.0f; // convert to float from q31
      float s = -(float32_t)sinOutput / 2147483648.0f; // convert to float from q31

      // LPF for motor current
      float tempD = Inv.I_D;
      float tempQ = Inv.I_Q;
      dq0(s, c, Inv.I_A, Inv.I_B, Inv.I_C, &tempD, &tempQ);    //dq0 transform on currents
      Inv.I_D = Inv.I_D - (0.0925f * (Inv.I_D - tempD));  
      Inv.I_Q = Inv.I_Q - (0.0925f * (Inv.I_Q - tempQ));  
      
      Dev.ShaftTorque = Inv.I_Q * TORQUE_CONST * (float)config->gear_ratio;

      float i_d_set = 0.0f;
      float i_q_set = (-1.0f/TORQUE_CONST)*( Dev.mech_gain*(Dev.mech_SetP - Dev.ShaftAngle) + Dev.vel_gain*(Dev.vel_SetP - Dev.ShaftVelocity) + (Dev.torq_SetP / (float)config->gear_ratio) ); // Kp[Amp/rad], Kd[Amp/rad*s]

      if( i_q_set > max_torque )
      {
        i_q_set = max_torque;
      }
      else if( i_q_set < -max_torque )
      {
        i_q_set = -max_torque;
      }

      // absolute limit on currents defined by the hardware safe operation region
      if( i_q_set > 30.0f )
      {
        i_q_set = 30.0f;
      }
      else if( i_q_set < -30.0f )
      {
        i_q_set = -30.0f;
      }
      
      float i_d_error = i_d_set - Inv.I_D;
      float i_q_error = i_q_set - Inv.I_Q;

      I_d_integral += 0.1f*0.0455f*i_d_error;
      I_q_integral += 0.1f*0.0455f*i_q_error;

      I_d_integral = fmaxf(fminf(I_d_integral, 1.0f*Inv.busV), - 1.0f*Inv.busV);
      I_q_integral = fmaxf(fminf(I_q_integral, 1.0f*Inv.busV), - 1.0f*Inv.busV); 

      float V_d = 1.0f*i_d_error + I_d_integral;// + v_d_ff;  
      float V_q = 1.0f*i_q_error + I_q_integral;// + v_q_ff; 

      limit_norm(&V_d, &V_q, 1.0f*Inv.busV);

      float v_u = 0, v_v = 0, v_w = 0;
      float dtc_u = 0, dtc_v = 0, dtc_w = 0;

      abc(s, c, V_d, V_q, &v_u, &v_v, &v_w); //inverse dq0 transform on voltages
      svm(Inv.busV, v_u, v_v, v_w, &dtc_u, &dtc_v, &dtc_w); //space vector modulation

      DQA = (uint16_t)(2000.0f*dtc_u);
      DQB = (uint16_t)(2000.0f*dtc_v);
      DQC = (uint16_t)(2000.0f*dtc_w);
    }
    else if( Dev.State == CURRENT )
    {
      float my_angle = 1.0f*PI;
      float s = arm_sin_f32(my_angle);
      float c = arm_cos_f32(my_angle);

      float V_d = -0.25f; // set the electrical angle to -pi, rotor should be pointing at +pi
      float V_q = 0.0f;
      
      float DVA = 0;
      float DVB = 0;
      float DVC = 0;

      abc(s, c, V_d, V_q, &DVA, &DVB, &DVC); //inverse dq0 transform on voltages

      DQA = 1000 + (int16_t)(1000.0f*DVA);
      DQB = 1000 + (int16_t)(1000.0f*DVB);
      DQC = 1000 + (int16_t)(1000.0f*DVC);  
    }

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, DQA);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, DQB);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, DQC);

    LL_SPI_TransmitData16(SPI3, SPI_TX_MSG);//load new value to transmit into SPI register i.e. initialize new receive sequence
  }
  
  return ;
}

float calib_elec_angle = 0;
void set_windings(void)
{
  float s = arm_sin_f32(calib_elec_angle);
  float c = arm_cos_f32(calib_elec_angle);

  const float V_d = -0.2f;
  const float V_q = 0.0f;

  float DVA, DVB, DVC;

  abc(s, c, V_d, V_q, &DVA, &DVB, &DVC); //inverse dq0 transform on voltages

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1000 + (int16_t)(1000.0f*DVA));
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 1000 + (int16_t)(1000.0f*DVB));
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 1000 + (int16_t)(1000.0f*DVC));
}

uint8_t encoder_inversion = 0;

uint16_t get_hall_angle(void)
{
  LL_GPIO_ResetOutputPin(SPI3_CS_PORT, SPI3_CS_PIN);

  micros_delay( 10 );

  LL_SPI_TransmitData16(SPI3, SPI_TX_MSG);
  while(!LL_SPI_IsActiveFlag_RXNE(SPI3)) {};

  uint16_t RX = LL_SPI_ReceiveData16(SPI3);//read SPI receive register to obtain value and clear pending interrupt
  RX &= 0x3FFF;//leave least 14 bits
  
  if( encoder_inversion )
  {
    int16_t inverted_RX = -RX;
    inverted_RX += CPR;
    
    RX = inverted_RX;
  }

  LL_GPIO_SetOutputPin(SPI3_CS_PORT, SPI3_CS_PIN);

  micros_delay( 10 );
  
  return RX;
}

float radians( void )
{
  return (float)get_hall_angle() * 2.0f * PI / (float)CPR;
}

#define offset_samples_num 1024
uint16_t ppair_counter = 0;
uint16_t offset_samples[offset_samples_num] = {0};
uint16_t meas_elec_offset = 0;

float old_angle = 0.0f;
float actual_angle = 0.0f;

#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma optimize s=none
void calibration_sequence(void)
#elif defined ( __GNUC__ ) /*!< GNU Compiler */
void __attribute__((optimize("O0"))) calibration_sequence(void)
#endif
{
  #define cal_buf_len 1024
  float CalBuf_Fwd[cal_buf_len] = {0};
  float CalBuf_Bckwd[cal_buf_len] = {0};

  float d_delta = 0.25f*(float)ppairs*2.0*PI / (float)cal_buf_len; // calculate angle step based on data array length
  
  // find the zero-point of encoder
  old_angle = actual_angle = radians();
  while( (old_angle - actual_angle) < PI ) // find the encoder zero-point
  {
    old_angle = actual_angle;
    
    calib_elec_angle += d_delta;

    set_windings();
    HAL_Delay(5);
    
    actual_angle = radians();
  }
  
  float d_offset = calib_elec_angle;

  uint32_t timestamp = HAL_GetTick();
  old_angle = actual_angle;
  // rotate in positive direction until you make full mechanical rotation
  while( ( old_angle - actual_angle ) < PI || HAL_GetTick() < timestamp + 100 ) // f_elec_angle < 2*PI
  {
    old_angle = actual_angle;
    
    uint16_t index = (uint16_t)( (float)cal_buf_len*actual_angle / (2.0f*PI) );
    CalBuf_Fwd[ index ] = ( calib_elec_angle - d_offset )/ppairs - actual_angle;
    
    // current algorithm sometimes leaves empty values in calibration buffer
    // this part fills the upcoming array memmber with the current value, which is overwritten in case of successfull new reading
    if( index < cal_buf_len )
    {
      CalBuf_Fwd[ index + 1 ] = CalBuf_Fwd[ index ];
    }

    calib_elec_angle += d_delta;
    
    set_windings();
    HAL_Delay(5);
    
    actual_angle = radians();
    
    if( fabs( mfmod( calib_elec_angle, 2.0f*PI ) - PI ) < d_delta/2.0f )
    {
      offset_samples[ppair_counter] = get_hall_angle();
      ppair_counter++;
    }    
  }
  
  for( int i = 0; i < 256; i++)
  {
    calib_elec_angle += d_delta;
    
    set_windings();
    HAL_Delay(5);
  }
  
  // find the zero-point of encoder
  old_angle = actual_angle = radians();
  while( ( old_angle - radians() ) > -PI ) // find the encoder zero-point
  {
    old_angle = actual_angle;
    
    calib_elec_angle -= d_delta;

    set_windings();
    HAL_Delay(5);
    
    actual_angle = radians();
  }
  
  d_offset = calib_elec_angle - (2.0f*PI)*ppairs;

  timestamp = HAL_GetTick();
  old_angle = actual_angle = radians();
  // rotate in negative direction until you return to the begining
  while( ( old_angle - actual_angle ) > -PI || HAL_GetTick() < timestamp + 100  )
  {
    old_angle = actual_angle;
    
    uint16_t index = (uint16_t)( (float)cal_buf_len*actual_angle / (2.0f*PI) );
    CalBuf_Bckwd[ index ] = ( calib_elec_angle - d_offset )/ppairs - actual_angle;
    
    // current algorithm sometimes leaves empty values in calibration buffer
    // this part fills the upcoming array memmber with the current value, which is overwritten in case of successfull new reading
    if( index > 0 )
    {
      CalBuf_Bckwd[ index - 1 ] = CalBuf_Bckwd[ index ];
    }

    calib_elec_angle -= d_delta;
    
    set_windings();
    HAL_Delay(5);
    
    actual_angle = radians();
    
    if( fabs( mfmod( calib_elec_angle, 2.0f*PI ) - PI ) < d_delta/2.0f )
    {
      offset_samples[ppair_counter] = get_hall_angle();
      ppair_counter++;
    }    
  }
  
  // encoder-magnet nonlinearity compensation lookup table should be calculated here 

  for( int i = 0; i < ppair_counter; i++ )
  {
    int16_t enc_angle = offset_samples[i];
    meas_elec_offset += (CPR / ppairs ) - ( enc_angle % (CPR / ppairs ) );
  }
  meas_elec_offset /= ppair_counter;

  while(1){};
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
