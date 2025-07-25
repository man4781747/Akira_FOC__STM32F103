/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/**
 * 待研究閱讀:
 *  - 不同 Timer 同步: https://shequ.stmicroelectronics.cn/thread-622883-1-1.html
 *  - PWM觸發ADC轉換並經由DMA搬運 : https://ithelp.ithome.com.tw/articles/10282007
 *  - DMA控制I2C : https://blog.csdn.net/KASIXA/article/details/136001090
 *  - USB 2.0 使用 : https://blog.csdn.net/qq_36347513/article/details/127404464
*   - PWM觸發ADC轉換並經由DMA搬運 : https://blog.51cto.com/u_16213585/12185463
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdio.h> // for printf
#include "setting.h"
#include "driver.h"
#include "M24C64_ctrl.h"
#include "sensor.h"
#include "micro_timer.h"
#include "lowpass_filter.h"
#include "PID_Ctrl.h"
#include "qfplib-m3.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define ADC_BUF_SIZE   2 // DMA 緩衝區大小 (單個通道的採樣點數)
#define ADC_BUF_SIZE_BUFFER 10 
uint16_t adc_dma_buffer[ADC_BUF_SIZE*ADC_BUF_SIZE_BUFFER];
uint16_t adc_bios = 0;

uint16_t adc_bios_W;
uint16_t adc_bios_U;
uint16_t adc_bios_V;

float angShift = 0;

LowPassFilter filter_U;
float current_U;
LowPassFilter filter_V;
float current_V;
LowPassFilter filter_W;
float current_W;
LowPassFilter filter_Speed;

float u_alpha, u_beta;
float I_alpha, I_beta;
float cosRad, sinRad;
float I_d, I_q;
float uq;
float ud = 0;
float ang_temp, angToRad;
float U1, U2, U3;

PIDController PID__current_Id;
PIDController PID__current_Iq;
PIDController PID__velocity;

uint16_t pwmPluse_1[1] = {0};
uint16_t pwmPluse_2[1] = {0};
uint16_t pwmPluse_3[1] = {0};


int isSend = 1;

void I2C_Scan_Bus(void)
{
  uint8_t i;
  HAL_StatusTypeDef ret;
  // uint8_t StartMSG[] = "Starting I2C Scanning: \n";
  // HAL_UART_Transmit(&huart2, StartMSG, sizeof(StartMSG), 10000);
  printf("Scanning I2C bus...\n");
  for (i = 1; i < 128; i++) // 遍历所有可能的 7 位地址 (1 到 127)
  {
      // 尝试检查设备是否就绪。I2C地址需要左移1位，并设置为8位地址
      // TrialByte: 用于发送的虚拟字节，通常可以为1，或者不需要实际的写入数据
      // Timeout: 等待应答的超时时间
      ret = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i << 1), 3, 5);

      if (ret == HAL_OK)
      {
        printf("Found device at address 0x%02X\n",i);
      }
      else if (ret == HAL_ERROR)
      {
          // HAL_I2C_IsDeviceReady返回HAL_ERROR通常表示总线错误，需要检查硬件连接
          // 或I2C配置，但在扫描时，这可能是因为没有设备应答，所以通常不做特殊处理
      }
      else if (ret == HAL_BUSY)
      {
          // I2C总线忙，可以等待或重试
      }
      else if (ret == HAL_TIMEOUT)
      {
          // 超时，表示没有设备应答
      }
  }
  // uint8_t EndMSG[] = "Done! \n";
  // HAL_UART_Transmit(&huart2, EndMSG, sizeof(EndMSG), 10000);
  printf("I2C scan complete.\n");
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_CAN_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  // 時鐘計時設定
  init_micros_timer();

  // M24C64
  uint8_t M24C64_Data[100];
  uint8_t WriteDataTest[100];
  // HAL_I2C_Mem_Write 函數會處理 START, 從設備位址, 記憶體位址, 資料, STOP, ACK/NACK
  for (int i =0;i<100;i++){
    WriteDataTest[i] = i;
    M24C64_Data[i] = 0;
  }
  M24C64_WriteData(WriteDataTest, 0, 32);
  M24C64_WriteData(WriteDataTest+32, 1, 32);
  M24C64_ReadData(M24C64_Data, 0, 32);
  M24C64_ReadData(M24C64_Data+32, 1, 32);

  float test_ = 0.000095f;
  LowPassFilter_Init(&filter_U, test_);
  LowPassFilter_Init(&filter_V, test_);
  LowPassFilter_Init(&filter_W, test_);
  LowPassFilter_Init(&filter_Speed, 0.008f);
  float test = 6.3 ; // 6.3
  PIDController_init(&PID__current_Id, test, test*50, 0, 0, 4.5);
  PIDController_init(&PID__current_Iq, test, test*50, 0, 0, 4.5);  
  // PIDController_init(&PID__velocity, 0.15, 0.515, 0, 0, 1);
  PIDController_init(&PID__velocity, 0.153, 0.35 , 0., 0, .7);

  // adc設定
  // https://blog.csdn.net/tangxianyu/article/details/121149981
  // HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_buffer, ADC_BUF_SIZE);



  // https://blog.csdn.net/qq_45854134/article/details/134181326
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, htim1.Init.Period);
  // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 225);
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); 
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  // HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t*)&pwmPluse_1, 1); 
  // HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_2, (uint32_t*)&pwmPluse_2, 1);
  // HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_3, (uint32_t*)&pwmPluse_3, 1);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); 
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_4);

  HAL_TIM_Base_Start(&htim4); 
  // HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_buffer, ADC_BUF_SIZE);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

  GPIO_InitTypeDef GPIO_InitStruct = {0}; // 1-V/A
  GPIO_InitStruct.Pin = GAIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG; // 設定為類比模式 (高阻態)
  GPIO_InitStruct.Pull = GPIO_NOPULL;      // 不啟用內部上下拉電阻 (通常類比模式不需要)
  HAL_GPIO_Init(GAIN_GPIO_Port, &GPIO_InitStruct);

  HAL_GPIO_WritePin(SLEW_GPIO_Port, SLEW_Pin, GPIO_PIN_SET);   // 200-V/µ
  HAL_GPIO_WritePin(PWM_ACTIVE_GPIO_Port, PWM_ACTIVE_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(SLEEP_GPIO_Port, SLEEP_Pin, GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(SLEEP_GPIO_Port, SLEEP_Pin, GPIO_PIN_SET);
  HAL_Delay(10);
  adc_bios_W = adc_dma_buffer[0];
  adc_bios_U = adc_dma_buffer[1];
  printf("ADC Bios: %d, %d\n", adc_bios_W, adc_bios_U);
  HAL_GPIO_WritePin(PWM_ACTIVE_GPIO_Port, PWM_ACTIVE_Pin, GPIO_PIN_SET);
  SetAng(0);
  
  HAL_Delay(500);
  angShift = readAng(angShift);

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

  int logCount = 0;
  float test123 = 1;
  float speed__Target = 0;
  float u_q__Target = 0.1;
  uint64_t old_ang_time = micros();
  uint64_t new_ang_time;
  float ang_get = readAng(angShift);
  float d_ang;
  float ang_speed = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    new_ang_time = micros();
    ang_temp = readAng(angShift);
    d_ang = ang_temp - ang_get;
    if (d_ang > 180) {
      d_ang -= 360;
    } else if (d_ang < -180) {
      d_ang += 360;
    }
    int dTime = (int)new_ang_time-(int)old_ang_time;
    if (dTime < 0) {
      dTime += 65535;
    }
      
    ang_speed = LowPassFilter_Update(&filter_Speed, d_ang/dTime*2777.777777777778f);
    // ang_speed = d_ang/dTime*2777.777777777778f;
    
    ang_get = ang_temp;
    old_ang_time = new_ang_time;
    // angToRad = ang_temp*7*0.01745329251f;
    angToRad = qfp_fmul(qfp_fmul(ang_temp, 7), 0.01745329251f);

    uint16_t W_add = 0;
    uint16_t U_add = 0;
    for (int i = 0; i < ADC_BUF_SIZE_BUFFER; i++) {
      W_add += adc_dma_buffer[2*i];
      U_add += adc_dma_buffer[2*i+1];
    }
    W_add /= ADC_BUF_SIZE_BUFFER;
    U_add /= ADC_BUF_SIZE_BUFFER;

    current_W = qfp_fmul(W_add-adc_bios_W, 0.0008056640625f);
    current_U = qfp_fmul(U_add-adc_bios_U, 0.0008056640625f);
    current_V = qfp_fsub(-current_U, current_W);
    // current_W = LowPassFilter_Update(&filter_W, (float)(W_add-adc_bios_W)*0.0008056640625f);
    // current_U = LowPassFilter_Update(&filter_U, (float)(U_add-adc_bios_U)*0.0008056640625f);
    // current_V = LowPassFilter_Update(&filter_V, -current_U - current_W);



    I_alpha = current_V;
    I_beta = qfp_fmul(
      qfp_fadd(
        qfp_fmul(current_U,2), 
        current_V
      ),_1_SQRT3
    );
    // I_beta = qfp_fadd(current_U*2, current_V);

    cosRad = qfp_fcos(angToRad);
    sinRad = qfp_fsin(angToRad);
    // cosRad = cos(angToRad);
    // sinRad = sin(angToRad);

    I_d = qfp_fadd( qfp_fmul(I_alpha, cosRad), qfp_fmul(I_beta, sinRad));
    I_q = qfp_fsub( qfp_fmul(I_beta, cosRad), qfp_fmul(I_alpha, sinRad));
    // I_d = I_alpha*cosRad + I_beta*sinRad;
    // I_q = -I_alpha*sinRad + I_beta*cosRad;
    

    // speed__Target = 5;
    u_q__Target = PIDController_process(&PID__velocity, speed__Target-ang_speed);
    // u_q__Target = 0.1;
    // uq = 1;
    uq = PIDController_process(&PID__current_Iq, u_q__Target - I_q);
    // uq = 3;
    // ud = 0;
    ud = PIDController_process(&PID__current_Id, -I_d);
    // if (ang_speed > 4 || ang_speed < -4) {
    //   ud = PIDController_process(&PID__current_Id, -I_d);
      
    // } else {
    //   ud = 0;
    // }
    u_alpha = qfp_fsub( qfp_fmul(ud, cosRad), qfp_fmul(uq, sinRad));
    u_beta = qfp_fadd( qfp_fmul(ud, sinRad), qfp_fmul(uq, cosRad));
    // u_alpha = ud*cosRad-uq*sinRad;
    // u_beta = ud*sinRad+uq*cosRad;

    if (logCount++ > 500) {
      // printf("%.2f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f, %.2f, %.2f\n", 
      //   ang_temp,I_q, I_d,current_W, current_U, current_V, ang_speed, U2, U3
      // );
      logCount = 0;
      speed__Target += test123;
      if (speed__Target > 10 || speed__Target < -10) {
        speed__Target = -speed__Target;
      }
      // if (speed__Target > 5 || speed__Target < -5) {
      //   test123 = -test123;
      // }
    }
    speed__Target = 10;
    Svpwm(u_alpha, u_beta);
    // printf("%.2f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f, %d\n", 
    //   ang_temp,I_q, I_d,current_W, current_U, current_V, ang_speed, adc_dma_buffer[2]
    // );
    // printf("%.2f,%.2f\n", 
    //   ang_temp, angShift
    // );
    // HAL_Delay(1);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T4_CC4;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 16;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 40000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 1800;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_ENABLE;
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
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 250;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim4.Init.Period = 1800;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim4, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SLEEP_Pin|SLEW_Pin|GAIN_Pin|PWM_ACTIVE_Pin
                          |LED_Status_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(EEPROM_RW__GPIO_Port, EEPROM_RW__Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : nFAULT_Pin */
  GPIO_InitStruct.Pin = nFAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(nFAULT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SLEEP_Pin SLEW_Pin GAIN_Pin PWM_ACTIVE_Pin
                           LED_Status_Pin */
  GPIO_InitStruct.Pin = SLEEP_Pin|SLEW_Pin|GAIN_Pin|PWM_ACTIVE_Pin
                          |LED_Status_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : EEPROM_RW__Pin */
  GPIO_InitStruct.Pin = EEPROM_RW__Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(EEPROM_RW__GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len)
{
    // 透過 UART1 將數據發送出去
    // huart1 是由 CubeMX 生成的 UART1 句柄
    // HAL_MAX_DELAY 表示阻塞式發送，直到發送完成
    // HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    // HAL_UART_Transmit_IT(&huart1, (uint8_t*)ptr, len);
    HAL_UART_Transmit_DMA(&huart1, (uint8_t*)ptr, len);
    return len;
}

void TIM1_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim1);
}

// void TIM4_IRQHandler(void)
// {
//   HAL_TIM_IRQHandler(&htim4);
// }

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) // 檢查是否是 TIM1 的 Channel 4 觸發
  {
    // 在這裡改變 GPIO 狀態
    // 假設你想控制 PA5 (連接到一個 LED)
    // HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)(adc_dma_buffer+adc_bios*2), ADC_BUF_SIZE);
    adc_bios ++;
    if (adc_bios >= ADC_BUF_SIZE_BUFFER) {
      adc_bios = 0;
    }
    // HAL_Delay(1);
    // HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

    // 如果需要持續在每個週期中點觸發，你可能需要重新設定下一個比較值
    // 對於這個應用，因為定時器是循環計數的，這個回呼函數會在每次達到 CCR2 時被觸發
    // 所以通常不需要手動重新設定 CCR2
  }
}

// DMA 傳輸完成回呼函數 (Full Transfer Complete)
// void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
// {
//     if (hadc->Instance == ADC1) // 檢查是否是 ADC1 的 DMA 觸發
//     {
//       // ang_temp = readAng(angShift);
//       // angToRad = qfp_fmul(qfp_fmul(ang_temp, 7), 0.01745329251f);
//       // current_W = qfp_fmul(adc_dma_buffer[0]-adc_bios_W, 0.0008056640625f);
//       // current_U = qfp_fmul(adc_dma_buffer[1]-adc_bios_U, 0.0008056640625f);
//       // current_V = qfp_fsub(-current_U, current_W);
//       // I_alpha = current_V;
//       // I_beta = qfp_fmul(
//       //   qfp_fadd(
//       //     qfp_fmul(current_U,2), 
//       //     current_V
//       //   ),_1_SQRT3
//       // );
//       // cosRad = qfp_fcos(angToRad);
//       // sinRad = qfp_fsin(angToRad);
//       // I_d = qfp_fadd( qfp_fmul(I_alpha, cosRad), qfp_fmul(I_beta, sinRad));
//       // I_q = qfp_fsub( qfp_fmul(I_beta, cosRad), qfp_fmul(-I_alpha, sinRad));
//       // uq = 1;
//       // ud = 0;
//       // u_alpha = qfp_fsub( qfp_fmul(ud, cosRad), qfp_fmul(uq, sinRad));
//       // u_beta = qfp_fadd( qfp_fmul(ud, sinRad), qfp_fmul(uq, cosRad));
//       // Svpwm(u_alpha, u_beta);
//       // float U1, U2, U3;
//       // float center = 5.f;
//       // U1 = u_alpha;
//       // U2 = qfp_fsub(qfp_fmul(_SQRT3_2, u_beta),qfp_fmul(u_alpha, 0.5f));
//       // // U2 = -0.5f * uAlpha + _SQRT3_2 * uBeta;
//       // U3 = qfp_fsub(qfp_fmul(-
//       //   _SQRT3_2, u_beta),qfp_fmul(u_alpha, 0.5f));
//       // U3 = -0.5f * uAlpha - _SQRT3_2 * uBeta;
//       // float Umin = fmin(U1, fmin(U2, U3));
//       // float Umax = fmax(U1, fmax(U2, U3));
//       // center = qfp_fsub(
//       //   center,
//       //   qfp_fmul(
//       //     qfp_fadd(Umax, Umin), 0.5f
//       //   )
//       // );
//       // center -= (Umax+Umin) / 2;

//       // U1 = qfp_fadd(U1, center);
//       // // U1 += center;
//       // U2 = qfp_fadd(U2, center);
//       // // U2 += center;
//       // U3 = qfp_fadd(U3, center);
//       // // U3 += center;
//       // U1 = qfp_fdiv(U1, 10);
//       // // U1 /= 10;
//       // U2 = qfp_fdiv(U2, 10);
//       // // U2 /= 10;
//       // U3 = qfp_fdiv(U3, 10);
//       // if (U1>1) {U1=1.;}
//       // else if (U1<0) {U1=0.;}
//       // if (U2>1) {U2=1.;}
//       // else if (U2<0) {U2=0.;}
//       // if (U3>1) {U3=1.;}
//       // else if (U3<0) {U3=0.;}


//     }
// }
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
#ifdef USE_FULL_ASSERT
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
