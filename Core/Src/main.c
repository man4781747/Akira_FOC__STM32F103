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
#include <string.h> 
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
#include "canbus_communication.h"
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

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;

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
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define ADC_BUF_SIZE   2 // DMA 緩衝區大小 (單個通道的採樣點數)
#define ADC_BUF_SIZE_BUFFER 1
uint16_t adc_dma_buffer[ADC_BUF_SIZE*ADC_BUF_SIZE_BUFFER];
uint16_t adc_bios = 0;

uint16_t adc_bios_W;
uint16_t adc_bios_U;
uint16_t adc_bios_V;


float ang_pre;
float d_ang;
float ang_speed = 0;

uint64_t old_ang_time, new_ang_time;
float angShift = 0;

float Iq__Target = 0.1;
float speed__Target = 0;
float positon_Target = 0;

enum DeviceMode deviceMode = DeviceMode_Stop;

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
// PIDController PID__velocity_05_1;
// PIDController PID__velocity_01_05;
PIDController PID__position;

// UART RX 相關變數
uint8_t uart_rx_buffer[32];


// 定義傳輸標頭和數據
CAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8] = {};
uint32_t TxMailbox; // 用來儲存發送的郵箱號

// CAN接收消息头和数据
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];

void CAN_Filter_Config(){
  CAN_FilterTypeDef sFilterConfig;
  sFilterConfig.FilterBank = 0;      //筛选器编号, CAN1是0-13, CAN2是14-27
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK; //采用掩码模式
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; //设置筛选器的尺度, 采用32位
  sFilterConfig.FilterIdHigh = 0X0000;    //过滤器ID高16位,即CAN_FxR1寄存器的高16位
  sFilterConfig.FilterIdLow = 0X0000;     //过滤器ID低16位,即CAN_FxR1寄存器的低16位
  sFilterConfig.FilterMaskIdHigh = 0X0000;   //过滤器掩码高16位,即CAN_FxR2寄存器的高16位
  sFilterConfig.FilterMaskIdLow = 0X0000;    //过滤器掩码低16位,即CAN_FxR2寄存器的低16位
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0; //设置经过筛选后数据存储到哪个接收FIFO
  sFilterConfig.FilterActivation = ENABLE;   //是否使能本筛选器
  sFilterConfig.SlaveStartFilterBank = 14;   //指定为CAN1分配多少个滤波器组
  if(HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  /* USER CODE BEGIN 2 */

  // UART1 RX 初始化 - 使用 DMA
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_rx_buffer, sizeof(uart_rx_buffer));

  // 時鐘計時設定
  init_micros_timer();

  // M24C64
  // uint8_t M24C64_Data[100];
  // uint8_t WriteDataTest[100];
  // for (int i =0;i<100;i++){
  //   WriteDataTest[i] = i;
  //   M24C64_Data[i] = 0;
  // }
  // M24C64_WriteData(WriteDataTest, 0, 32);
  // M24C64_WriteData(WriteDataTest+32, 1, 32);
  // M24C64_ReadData(M24C64_Data, 0, 32);
  // M24C64_ReadData(M24C64_Data+32, 1, 32);

  float adcSimpleRate = 1.f/20000.f;
  LowPassFilter_Init(&filter_U, adcSimpleRate);
  LowPassFilter_Init(&filter_V, adcSimpleRate);
  LowPassFilter_Init(&filter_W, adcSimpleRate);
  LowPassFilter_Init(&filter_Speed, 0.008f);

  /**
   * https://www.bilibili.com/video/BV1VT421k782/?spm_id_from=333.337.search-card.all.click&vd_source=43115a2a3d33edfb64a16bbcb1b2fda3
   * R = 5.1
   * L = 0.0028
   * Kp = 0.0028(L)*350*2*pi = 6.157521601035994
   * Ki = 5.1*350*2*pi/10000 = 1.121548577331556
   * 
   * 後續精實測，此做法與實際調適的參數差異極大，可能PID的方法是不同的?
   */
  PIDController_init(&PID__current_Id, 4.6, 1475, 0, 0, 4.5);  
  PIDController_init(&PID__current_Iq, 4.6, 1475, 0, 0, 4.5);  


  PIDController_init(&PID__velocity, 0.048 , 0.0925 , 0., 0, .7);  // 1~10
  PIDController_init(&PID__position,0.049, 0., 0.000, 0, 10);
  // adc設定
  // https://blog.csdn.net/tangxianyu/article/details/121149981
  // HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_buffer, ADC_BUF_SIZE);



  // https://blog.csdn.net/qq_45854134/article/details/134181326
  /**
   * 設置各通道的PWM占空比
   * U、V、W 通道設為 0
   * ADC量測觸發設為 htim1.Init.Period
   *  - 但 ADC 量測怎麼調都會比在馬達驅動的PWM週期中心多偏移一點
   */
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);    
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, htim1.Init.Period);

  __HAL_TIM_SET_COUNTER(&htim1, 0);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); 
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_4);


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

  HAL_GPIO_WritePin(PWM_ACTIVE_GPIO_Port, PWM_ACTIVE_Pin, GPIO_PIN_SET);

  // SetAng(0);
  // HAL_Delay(500);
  // angShift = readAng(angShift);
  angShift = ReadAngShift();
  printf("%d\n", (int)(angShift*100));

  // for (int i=0;i<360*7;i=i+30) {
  //   SetAng(i);
  //   printf("%d, %.2f\n", i, readAng(angShift));
  //   HAL_Delay(250);
  // }
  // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
  // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

  int logCount = 0;

  old_ang_time = micros();
  ang_pre = readAng(angShift);

  // 啟動CAN週邊

  CAN_Filter_Config();
	if (HAL_CAN_Start(&hcan) != HAL_OK) 
  {
    Error_Handler();
  }
	else{
    printf("HAL_CAN1_Star1 is HAL_OK\r\n"); 
  }

  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }
  // 設定傳輸標頭
  TxHeader.StdId = 0x123; // 你的CAN ID (標準ID)
  TxHeader.RTR = CAN_RTR_DATA; // 數據幀
  TxHeader.IDE = CAN_ID_STD; // 標準ID
  TxHeader.DLC = 8; // 數據長度，最多8個位元組



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // if (deviceMode == DeviceMode_Stop) {
    //   __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    //   __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    //   __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    //   ang_pre = -100;
    // }
    // else if (deviceMode == DeviceMode_SetAngShift) {
    //   printf("Try to set angShift value, set motor to 0 ang\n");
    //   SetAng(0);
    //   HAL_Delay(500);
    //   WriteAngShift(readAng(0));
    //   angShift = ReadAngShift();
    //   printf("New angShift: %d.%d\n", (int)(angShift*100)/100, (int)(angShift*100)%100);
    //   deviceMode = DeviceMode_Stop;
    // } else {
    //   DoFoc();
    // }
    DoFoc();
    // printf("%.2f,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,%.4f, %.4f, %.2f\n", 
    //   ang_temp,positon_Target,
    //   ang_speed, speed__Target,
    //   I_q, Iq__Target,current_W, current_U, current_V, I_d
    // );
    





    // // 設定要發送的數據
    // TxData[0] = TxData[0] + 1;
    // for (int i = 0 ; i< 6 ; i++){
    //   if (TxData[i] == 0xFF) {
    //     TxData[i] = 0x00; // 重置計數器
    //     TxData[i+1] = TxData[i+1] + 1;
    //   }
    // }
    // if (TxData[7] == 0xFF) {
    //   TxData[0] = 0x00; // 重置計數器
    // }
    memcpy(TxData, &ang_speed, 4);

    // 發送訊息
    if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
        // 處理發送失敗的情況
      // printf("Cnan send message failed!\n");
      // Error_Handler();
    }
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
  hcan.Init.Prescaler = 6;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
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
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  // HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
  // HAL_UART_Transmit_IT(&huart1, (uint8_t*)ptr, len);
  HAL_UART_Transmit_DMA(&huart1, (uint8_t*)ptr, len);
  return len;
}

void TIM1_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim1);
}


void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) // 檢查是否是 TIM1 的 Channel 4 觸發
  {
    /**
     * @brief 有感流程需要量測ADC
     * 
     */
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_buffer, 2);

    // HAL_ADC_Start_DMA(&hadc1, (uint32_t*)(adc_dma_buffer+adc_bios*2), ADC_BUF_SIZE);
    // adc_bios ++;
    // if (adc_bios >= ADC_BUF_SIZE_BUFFER) {
    //   adc_bios = 0;
    // }
  }
}

// DMA 傳輸完成回呼函數 (Full Transfer Complete)
// void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
// {
//     if (hadc->Instance == ADC1) // 檢查是否是 ADC1 的 DMA 觸發
//     {
//       ang_temp = readAng(angShift);
//       angToRad = qfp_fmul(qfp_fmul(ang_temp, 7), 0.01745329251f);
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
//       uq = 1;
//       ud = 0;
//       u_alpha = qfp_fsub( qfp_fmul(ud, cosRad), qfp_fmul(uq, sinRad));
//       u_beta = qfp_fadd( qfp_fmul(ud, sinRad), qfp_fmul(uq, cosRad));
//       // Svpwm(u_alpha, u_beta);
//       float U1, U2, U3;
//       float center = 5.f;
//       U1 = u_alpha;
//       U2 = qfp_fsub(qfp_fmul(_SQRT3_2, u_beta),qfp_fmul(u_alpha, 0.5f));
//       U3 = qfp_fsub(qfp_fmul(-_SQRT3_2, u_beta),qfp_fmul(u_alpha, 0.5f));
//       float Umin = fmin(U1, fmin(U2, U3));
//       float Umax = fmax(U1, fmax(U2, U3));
//       center = qfp_fsub(center,qfp_fmul(qfp_fadd(Umax, Umin), 0.5f));
//       center -= (Umax+Umin) / 2;

//       U1 = qfp_fadd(U1, center);
//       // U1 += center;
//       U2 = qfp_fadd(U2, center);
//       // U2 += center;
//       U3 = qfp_fadd(U3, center);
//       // U3 += center;
//       U1 = qfp_fdiv(U1, 10);
//       // U1 /= 10;
//       U2 = qfp_fdiv(U2, 10);
//       // U2 /= 10;
//       U3 = qfp_fdiv(U3, 10);
//       if (U1>1) {U1=1.;}
//       else if (U1<0) {U1=0.;}
//       if (U2>1) {U2=1.;}
//       else if (U2<0) {U2=0.;}
//       if (U3>1) {U3=1.;}
//       else if (U3<0) {U3=0.;}


//     }
// }

// void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
// {
//   if (huart->Instance == USART1) {
//     // 停止 DMA 接收以獲取已接收的資料長度
//     HAL_UART_DMAStop(huart);

//     // 將接收到的資料從 DMA 緩衝區複製到處理緩衝區

//     // printf("UART Receive: %s\n", uart_rx_buffer);
//     int intPart = 0;
//     // int result = sscanf((char*)uart_rx_buffer, "ST:%d", &intPart);
//     if (sscanf((char*)uart_rx_buffer, "ST:%d", &intPart) == 1) {
//       // printf("Set Speed: %d.%02d\n", intPart/100, intPart%100);
//       speed__Target = ((float)intPart)/100; 
//       deviceMode = DeviceMode_SpeedMode;
//     }
//     else if (sscanf((char*)uart_rx_buffer, "PT:%d", &intPart) == 1) {
//       // printf("Set ang: %d.%d\n", intPart/10, intPart%10);
//       positon_Target = ((float)intPart)/10; 
//       deviceMode = DeviceMode_PositionMode;
//     }
//     else if (strcmp((char*)uart_rx_buffer, "STOP") == 0) {
//       deviceMode = DeviceMode_Stop;
//     }

//     // 重新啟動 DMA 接收，繼續監聽下一次事件
//     memset(uart_rx_buffer, 0, sizeof(uart_rx_buffer)); // 清空接收緩衝區
//     HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_rx_buffer, sizeof(uart_rx_buffer));
//   }
// }


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    // 从FIFO0中接收消息
    if (HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO0, &RxHeader, RxData) != HAL_OK)
    {
      // 接收失败处理
      Error_Handler();
    }
    if (RxData[0] == STOP_MOTOR) {
      deviceMode = DeviceMode_Stop;
    }
    else if (RxData[0] == SET_ANG_SHIFT) {
      deviceMode = DeviceMode_SetAngShift;
    }
    else if (RxData[0] == SET_SPEED) {
      if (RxData[1] == Set_TargetVale) {
        memcpy(&speed__Target, &RxData[2], sizeof(float));
        deviceMode = DeviceMode_SpeedMode;
      }
    }
    



    // // 在这里处理接收到的数据
    // // 例如：检查CAN ID，解析数据，更新变量等
    // if (RxHeader.StdId == 0x100)
    // {
    //   sprintf("%s", RxData);
    //     // 打印接收到的数据
    //     // ...
    // }
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
