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
 * STM32F103 FOC 驅動 (STM32_FOC)。程式架構、指令與擷取格式沿用後代 STM32F446RE_TEST 的 main.c，
 * 讓後代的電腦端工具 (log_capture.py、swd_cmd.py、motor_ui) 可以直接使用。與後代不同的地方：
 *  - 角度：AS5600 (I2C2, 12 bit) 以 DMA 在背景連續讀取，換成 65536 count/rev (左移 4 bit) 後與後代同單位
 *  - FOC：TIM1 CH4 (OC4REF) 經 TRGO 觸發 ADC1 injected 轉換，JEOC 中斷內執行 (Foc_AdcIsr)
 *  - 電流：ADC1 依序轉 CH1 (PB1, IN9)、CH2 (PB0, IN8)，CH3 由 KCL 推算
 *  - 沒有 MT6835 自校正、編碼器補償 (ENCC)、EEPROM 參數區塊、棘輪 / 限位 / 無摩擦 / 軌跡規劃
 *  - CAN 指令沿用舊的處理方式 (HAL_CAN_RxFifo0MsgPendingCallback)；只收 CANID 的訊框，並以同一個 ID 定時回報狀態 (這一代才有)
 *  - 力矩模式 MODE 5 (這一代才有，開機預設，IQREF = 0)：配合平衡車 ESP32S3_MCP2515_CANBbus_Test 的 CAN SET_IQ 命令
 * 詳見 doc/設計文件.html。
 *
 * 待研究閱讀:
 *  - 不同 Timer 同步: https://shequ.stmicroelectronics.cn/thread-622883-1-1.html
 *  - PWM觸發ADC轉換並經由DMA搬運 : https://ithelp.ithome.com.tw/articles/10282007
 *  - DMA控制I2C : https://blog.csdn.net/KASIXA/article/details/136001090
 *  - USB 2.0 使用 : https://blog.csdn.net/qq_36347513/article/details/127404464
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "setting.h"
#include "qfplib-m3.h"
#include "micro_timer.h"
#include "PID_Ctrl.h"
#include "pi_fix.h"
#include "lowpass_filter.h"
#include "driver.h"
#include "sin_lut.h"
#include "AS5600.h"
#include "M24C64.h"
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

CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c2_rx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

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
/* USER CODE BEGIN PFP */
// ---- FOC 頻率：TIM1 中心對齊 20 kHz，ADC 每個 PWM 週期轉換一次，FOC 每 FOC_DIV 次執行一次 ----
// 電流環每次都算 (定點數)；速度環、位置環 (浮點) 的計算分散在 VEL_DIV 次中斷內 (Foc_Slot0 ~ 3)
#define FOC_DIV            1
#define VEL_DIV            4   // 時槽數，Foc_Step() 的 switch 寫死是 4 個
#define ADC_TO_A           0.0008056640625f  // 3.3 V / 4096，DRV8317 GAIN = Hi-Z (1 V/A)
#define DRV_WAKE_SETTLE_MS 250  // DRV8317 喚醒後，量測偏置與對齊前的等待時間 (同後代)
#define ADC_BIAS_SAMPLES   256  // 開機量測電流偏置時平均的筆數，每筆間隔 1 ms (同後代)

uint32_t pwm_freq;       // PWM 頻率 (Hz)，由 TIM1 設定算出
uint32_t foc_hz;         // FOC 執行頻率 (Hz) = pwm_freq / FOC_DIV
float foc_dt, foc_inv_dt; // FOC 週期 (秒) 與其倒數
volatile uint16_t adc_raw1, adc_raw2;  // 最近一次 (每個 PWM 週期) 的 CH1、CH2 讀值，開機量偏置用
uint16_t adc_foc1, adc_foc2;           // FOC 這一次用的讀值
uint8_t foc_div_cnt;
volatile uint32_t foc_isr_cyc, foc_isr_cyc_max;  // FOC 中斷的執行時間 (CPU 週期，72 MHz)，經 SWD 讀取
float adc_bios_CH1;
float adc_bios_CH2;

LowPassFilter filter_Speed;  // 只用 Tf (速度回授低通的時間常數，SET VELTF)

// ---- 電流環 (定點數)：電流 1 單位 = ADC 1 LSB / 16 (約 50 µA)，電壓 1 單位 = 比較值 1/16 ----
#define CUR_SHIFT          4
#define I_UNIT_A           (ADC_TO_A / (1 << CUR_SHIFT))
#define V_SHIFT            4
float v_fix_per_volt;        // 電壓命令 1 (10 V 基準的單位，同後代) 對應的定點值 = ARR × 0.1 × 16
int32_t bias1_fix, bias2_fix;          // 電流偏置 × 16
int32_t cur1_fix, cur2_fix;            // CH1、CH2 電流
int32_t id_fix, iq_fix;                // Id、Iq
int32_t vd_fix, vq_fix, va_fix, vb_fix;  // 電壓命令 (d、q、α、β)
volatile int32_t iq_ref_fix;           // Iq 命令，速度環 / MIT 算出 uq_Target 時更新
int32_t id_ref_fix;                    // Id 命令，只有 ITEST_CUR 期間不是 0
PiFix pi_iq, pi_id;
uint8_t foc_slot;                      // 0 ~ VEL_DIV-1
float ang_temp;              // 機械角 (度，0 ~ 360)，FOC 這一次用的角度 (含外插)
float ang_speed;             // 速度回授 (圈/秒)
int32_t speed_q16;           // 速度 (count/µs，Q16)，電角度外插用
int32_t spd_d_sum;           // 上一次算速度之後累計的角度變化 (count) 與時間 (µs)
uint32_t spd_dt_sum;
float spd_meas;              // 最近一次量到的速度 (低通之前，圈/秒)
volatile float spd_alpha;    // 速度低通係數 Tf / (Tf + 速度環週期)，Ctrl_Sync() 算出
float uq_Target = 0.f;       // Iq 命令 (A)：速度環或位置環 (MIT) 的輸出
float speed_Target = 0.1f;   // 圈/秒
uint64_t new_ang_time;       // 這一次 FOC 中斷開始的時間 (µs)

volatile int FocDo = 0;

PIDController PID__current_Id;
PIDController PID__current_Iq;
PIDController PID__velocity;
PIDController PID__position;

// ---- 角度：AS5600 12 bit 左移 4 bit，和後代同樣是一圈 65536 count；零點 (zero_raw) 由開機對齊決定 ----
#define ENC_CPR            65536
#define ENC_SHIFT          4        // AS5600 讀值 (4096/圈) × 16 = ENC_CPR
// AS5600 讀值增加的方向和磁場 CH1→CH2→CH3 的旋轉方向相反 (2026-10-14 實測)，所以角度取 zero_raw − 讀值。
// 這樣正轉 (SET SPEED > 0) 從軸端看是逆時針，與後代相同
#define ENC_DIR            (-1)
// 1 = 馬達三條線中有兩條接反 (平衡車，2026-10-15)：以軟體對調兩相，α 不變、β 取負 (Clarke 與 SVPWM 兩處)。
// 開機 ALIGN 會吸收接反哪兩條造成的 120° 偏移，所以 ENC_DIR 與正轉方向 (從軸端看逆時針) 都不變
#define PHASE_SWAP         1
#define POLE_PAIRS         7
#define ANG_TIMEOUT_US     5000     // 超過這麼久沒有新的角度：關閉輸出 (滑行)，要 RUN 才恢復
#define ANG_EXTRAP_MAX_US  1000     // 電角度以速度往前外插，最多外插這麼久
#define ANG_STALL_MS       20       // 主迴圈：超過這麼久沒有新的角度就重新初始化 I2C2
AS5600_t as5600;
uint16_t zero_raw;                  // 電氣零點的 AS5600 讀值
volatile uint8_t foc_reseed = 1;    // 1 = FOC 中斷下一次重新起算速度與多圈位置 (開機、ALIGN 之後)
uint32_t ang_seq_prev;              // 上一次用過的 AS5600 讀值編號
uint32_t ang_t_prev;                // 上一筆角度的時間 (µs)
volatile uint8_t ang_fault;         // 1 = 角度逾時，FOC 中斷已關閉輸出，主迴圈回報後清成 0
uint32_t ang_stall_tick;            // 主迴圈最後一次看到新角度的 HAL_GetTick()
uint32_t ang_stall_seq;

// ---- 位置環：MODE 1。POSTYPE 0 = 串級 (位置 P → 速度 PI，同後代)，1 = MIT (彈簧 + 阻尼直接算 Iq) ----
#define CTRL_MODE_VEL      0
#define CTRL_MODE_POS      1
#define POS_TYPE_CASCADE   0
#define POS_TYPE_MIT       1
#define POS_DEG_PER_COUNT  (360.0f / ENC_CPR)
#define POS_COUNT_PER_DEG  (ENC_CPR / 360.0f)
#define POS_TARGET_MAX     2.0e9f  // 位置目標的上限 (count)，留在 int32 範圍內，約 ±30517 圈
#define POS_DIV_DEFAULT    8       // 位置環每幾次 FOC 中斷算一次 (串級)，以 VEL_DIV 為單位進位；8 = 2.5 kHz
#define POS_DIV_MAX        1000
#define MIT_KP_DEFAULT     0.01f   // A/度，舊程式的值
#define MIT_KD_DEFAULT     0.15f   // A/(圈/秒)，舊程式的值
// ---- 力矩模式：MODE 5 (這一代才有，後代 MODE 2 ~ 4 是棘輪 / 限位 / 無摩擦)。Iq 命令 = IQREF (A)，限幅同 VEL.LIMIT。開機預設 ----
// 平衡車 (ESP32S3_MCP2515_CANBbus_Test) 以 CAN SET_IQ 下命令；CAN 命令超過 TRQ_CAN_TIMEOUT_MS 沒更新時 IQREF 歸零
#define CTRL_MODE_TRQ      5
#define TRQ_CAN_TIMEOUT_MS 100
volatile uint8_t ctrl_mode = CTRL_MODE_TRQ;
volatile float trq_iq = 0.f;        // IQREF (A)
volatile uint8_t can_trq_live;      // 1 = IQREF 來自 CAN，逾時會歸零 (SET IQREF 會清成 0)
volatile uint32_t can_trq_tick;     // 最後一筆 CAN 力矩命令的 HAL_GetTick()
volatile uint8_t can_trq_req;       // CAN 力矩命令要求切到 MODE 5 / RUN，主迴圈 (Can_Process) 處理
volatile uint8_t pos_type = POS_TYPE_CASCADE;
float mit_kp = MIT_KP_DEFAULT;
float mit_kd = MIT_KD_DEFAULT;
volatile int32_t pos_count;         // 多圈位置 (count)，0 在電氣零點；超過 ±32768 圈會回繞
volatile int32_t pos_target_count;  // 位置目標 (count)
int32_t pos_err_count;              // 位置誤差 (count)，速度模式時為 0
uint16_t pos_enc_prev;              // 上一筆角度 (count，0 ~ 65535)
volatile uint16_t pos_div = POS_DIV_DEFAULT;
uint16_t pos_div_cnt;
float pos_speed_cmd;                // 串級位置環輸出 (圈/秒)，兩次計算之間維持不變
volatile float pos_ts, pos_inv_ts;  // 位置環的週期 (秒) 與其倒數，Ctrl_Sync() 算出

// Id 環：預設啟用，目標固定為 0；SET IDLOOP 0 關閉 (ud 固定為 0)
volatile uint8_t id_loop = 1;
float id_Target;                    // Id 命令 (A)，固定 0 (後代只有 ITEST 會改)

// ---- 電流環測試 (ITEST，同後代)：速度模式下暫時取代速度環，d、q 兩軸各在 A、B 兩個準位之間切換 (方波) ----
//   ITEST_CUR：準位是電流命令 (A)，交給 Iq / Id 兩個 PI，測試期間 Id 環一定啟用
//   ITEST_VOLT：準位是電壓 (V)，不經過 PI 直接輸出，用來量 R、L
// FOC 中斷內只用定點數：準位由 Ctrl_Sync() 換算 (測試中 SET 改準位最慢 5 ms 後生效)
#define ITEST_OFF          0
#define ITEST_CUR          1
#define ITEST_VOLT         2
#define ITEST_I_MAX        1.0f    // 電流準位的上限 (A)
#define ITEST_U_MAX        3.0f    // 電壓準位的上限 (V)
#define ITEST_I_TRIP       2.0f    // 實測 Id / Iq 超過就結束 (A)
#define ITEST_SPEED_TRIP   40.0f   // 轉速超過就結束 (圈/秒)
#define ITEST_HALF_MAX     40000   // 半週期的上限 (FOC 中斷次數)，20 kHz 時 2 秒
#define ITEST_MS_DEFAULT   1000
#define ITEST_MS_MAX       10000
#define ITEST_END_TIME     1       // itest_end：上一次測試結束的原因
#define ITEST_END_STOP     2
#define ITEST_END_SPEED    3
#define ITEST_END_CURRENT  4
volatile uint8_t itest_mode = ITEST_OFF;
volatile uint8_t itest_end;
volatile uint32_t itest_half;       // 半週期 (FOC 中斷次數)
volatile uint32_t itest_left;       // 還剩幾次 FOC 中斷
uint32_t itest_cnt;                 // 以下只在 FOC 中斷內使用
uint8_t itest_phase;                // 0 = A 準位，1 = B 準位
int32_t itest_q, itest_d;           // 這一次中斷兩軸的準位 (定點：電流或電壓單位)
float it_qa, it_qb, it_da, it_db;   // 兩軸的 A、B 準位 (SET IT.QA 等)，測試中也可以改
volatile int32_t it_fix[2][4];      // [0] 電流單位、[1] 電壓單位：QA、QB、DA、DB，已限制在上限內。Ctrl_Sync() 算出
int32_t itest_i_trip_fix;           // ITEST_I_TRIP 換成電流單位
int32_t itest_spd_trip_q16;         // ITEST_SPEED_TRIP 換成 speed_q16 的單位

// ---- 速度環的附加項 (同後代) ----
// 速度回授低通的時間常數 (秒)，SET VELTF 以 ms 為單位。AS5600 只有 12 bit，低速時一個 LSB 的跳動就很大，
// 預設 0.5 ms (後代是 0.4 ms)：速度環 I = 25 時 1.5 ms 的延遲已不夠穩 (3 ms 會振盪)，見速度環參數的說明
#define VEL_TF_DEFAULT  0.5e-3f
#define VEL_TF_MAX      50e-3f
volatile uint8_t vel_aw = 1;        // 積分抗飽和，SET VELAW 0 關閉
volatile float vel_b = 0.7f;        // 目標權重 (0 ~ 1)
float vel_cmd_lp;                   // 目標權重低通的狀態，只在 FOC 中斷內使用
volatile float vel_lp_k;            // 目標權重低通的係數，Ctrl_Sync() 算出
uint8_t iq_pi_on, id_pi_on;         // 上一次 FOC 中斷有沒有執行該 PI，只在 FOC 中斷內使用

// ---- 電流回授低通 (同後代)，預設都關閉 ----
#define CUR_TF_DEFAULT  50e-6f
#define CUR_TF_MAX      5.0e-3f
volatile uint8_t iph_lpf, idq_lpf;
volatile float iph_tf = CUR_TF_DEFAULT;
volatile float idq_tf = CUR_TF_DEFAULT;
int32_t iph_lp1, iph_lp2, idq_lp_d, idq_lp_q;  // 低通的狀態 (電流單位 × 4096)
volatile int32_t iph_k, idq_k;      // 低通係數 dt / (Tf + dt) (Q16)，Ctrl_Sync() 算出

// ---- 齒槽補償前饋 exf_map：舊程式在電流回授無效時量的，先停用；要重新量測後再打開 (EXF_MAP_ENABLE 1) ----
// 打開時開機從 EEPROM 讀出 360 筆 (A，索引 = AS5600 絕對角度的整數度)，速度模式 |SPEED| <= 0.5 時加到 Iq 命令
#define EXF_MAP_ENABLE  0
#if EXF_MAP_ENABLE
float exf_map[PRE_F_LEN];
#endif

// ---- Serial 指令接收 (USART1 RX 中斷，一行一個指令，格式同後代 doc/serial指令.html) ----
#define CMD_LINE_MAX 32
uint8_t cmd_rx_byte;
char cmd_rx_buf[CMD_LINE_MAX];
uint8_t cmd_rx_len = 0;
uint8_t cmd_rx_overflow = 0;
char cmd_line[CMD_LINE_MAX];
volatile uint8_t cmd_line_ready = 0;
static char uart_tx_buf[160];       // printf 經 DMA 送出時的緩衝區

// ---- SWD 指令：除錯器把指令編號寫進 swd_cmd，主迴圈執行後清回 0 (後代 swd_cmd.py)。編號同後代 ----
#define SWD_CMD_NONE       0
#define SWD_CMD_MT6835_CAL 1   // 這一代沒有，回 [CMD] UNKNOWN
#define SWD_CMD_EE_SAVE    2   // 階段 4 才做，目前回 [CMD] UNKNOWN
#define SWD_CMD_EE_LOAD    3
#define SWD_CMD_EE_ERASE   4
#define SWD_CMD_ZERO_CLR   5
#define SWD_CMD_ALIGN      6
volatile uint32_t swd_cmd = SWD_CMD_NONE;

// ---- 輸出狀態：STOP / BRAKE / RUN 指令切換。停止期間 FOC 中斷只量測不輸出 (同後代) ----
#define OUT_RUN   0
#define OUT_COAST 1   // PWM_ACTIVE = Low，三相高阻，馬達滑行
#define OUT_BRAKE 2   // 三相比較值 0，下橋全開，馬達被動煞車
volatile uint8_t out_state = OUT_RUN;

// ---- 高速擷取 / 串流 (同後代)：FOC 中斷內每 log_div 次存一筆到 log_buf ----
#define LOG_BUF_BYTES   8192      // RAM 只有 20 KB；預設 14 byte/筆時是 585 筆
#define LOG_DIV_MAX     2000      // t_us 只有 16 bit，兩筆的間隔需小於 65 ms
#define LOG_SCALE_I     20000.0f  // dq 電流：1 LSB = 0.05 mA
#define LOG_SCALE_SPEED 2000.0f   // 速度：1 LSB = 0.0005 圈/秒
#define LOG_SCALE_IPH   10000.0f  // 相電流：1 LSB = 0.1 mA
#define LOG_SCALE_U     2000.0f   // 電壓命令：1 LSB = 0.5 mV

// 欄位編號就是 log_mask 的 bit，一筆紀錄內依編號由小到大排列。編號、大小、比例同後代 (motor_ui/fields.py)
enum {
  LOG_F_ENC = 0,    // uint16  角度 (0 ~ ENC_CPR-1)，AS5600 讀值 × 16
  LOG_F_T_US,       // uint16  micros() 的低 16 bit
  LOG_F_IQ_CMD,     // int16   uq_Target (速度環或 MIT 的輸出)，平均，LOG_SCALE_I
  LOG_F_IQ,         // int16   Iq，平均，LOG_SCALE_I
  LOG_F_ID,         // int16   Id，平均，LOG_SCALE_I
  LOG_F_SPEED,      // int16   ang_speed，平均，LOG_SCALE_SPEED (超過 ±16.38 圈/秒 飽和)
  LOG_F_POS_ERR,    // int16   pos_err_count，平均，1 LSB = 1 count，超過約 ±180 度時飽和
  LOG_F_I_CH1,      // int16   CH1 電流，LOG_SCALE_IPH
  LOG_F_I_CH2,      // int16   CH2 電流，LOG_SCALE_IPH
  LOG_F_I_CH3,      // int16   CH3 電流 (CH1、CH2 推算)，LOG_SCALE_IPH
  LOG_F_UQ,         // int16   q 軸電壓命令，平均，LOG_SCALE_U
  LOG_F_UD,         // int16   d 軸電壓命令，平均，LOG_SCALE_U
  LOG_F_U_ALPHA,    // int16   α 軸電壓命令，LOG_SCALE_U
  LOG_F_U_BETA,     // int16   β 軸電壓命令，LOG_SCALE_U
  LOG_F_ADC1,       // uint16  CH1 原始值
  LOG_F_ADC2,       // uint16  CH2 原始值
  LOG_F_SPEED_F,    // float   ang_speed，平均 (不會飽和)
  LOG_F_SPEED_CMD,  // float   速度環的目標：speed_Target 或位置環輸出
  LOG_F_POS,        // int32   pos_count
  LOG_F_POS_TGT,    // int32   pos_target_count
  LOG_F_ID_CMD,     // int16   id_Target，平均，LOG_SCALE_I
  LOG_F_COUNT
};
#define LOG_MASK_ALL     ((1UL << LOG_F_COUNT) - 1)
#define LOG_MASK_DEFAULT 0x7FUL   // 前 7 欄：14 byte/筆，log_capture.py 使用的格式
static const uint8_t log_field_size[LOG_F_COUNT] = {2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 4, 4, 4, 4, 2};

typedef enum {
  LOG_IDLE = 0,
  LOG_CAPTURING,
  LOG_READY,        // 已存滿，等主迴圈經 UART 送出
  LOG_HOLD,         // 已存滿，留在緩衝區等 SWD 讀走 (LOGC)，LOGX 後回到 LOG_IDLE
  LOG_STREAM,       // 環形緩衝區持續寫入 (LOGS)，LOGX 後回到 LOG_IDLE
} LogState;

#define LOG_MODE_UART   0
#define LOG_MODE_SWD    1
#define LOG_MODE_STREAM 2

uint8_t log_buf[LOG_BUF_BYTES] __attribute__((aligned(4)));
volatile LogState log_state = LOG_IDLE;
uint32_t log_mask = LOG_MASK_DEFAULT;
uint16_t log_rec_size = 14;            // log_mask 選到的欄位大小總和
uint32_t log_cap = LOG_BUF_BYTES / 14; // 緩衝區放得下幾筆
uint32_t log_idx;
uint8_t log_mode;
uint16_t log_div;
uint16_t log_div_cnt;
uint8_t log_quiet;             // 1 = 擷取期間暫停每 5 ms 的數據輸出
// 平均用的累加值 (整數，已乘上 LOG_SCALE_*)；div 最大 2000，不會溢位
int32_t log_sum_iq_cmd, log_sum_iq, log_sum_id, log_sum_speed, log_sum_pos_err, log_sum_uq, log_sum_ud, log_sum_id_cmd;
int32_t log_iq_cmd_i, log_speed_i;  // uq_Target × LOG_SCALE_I、ang_speed × LOG_SCALE_SPEED，時槽內更新
int32_t log_k_i, log_k_iph, log_k_u;  // 定點值換成擷取欄位的係數 (Q16)
float log_speed_cmd;           // 速度環用的目標
uint32_t log_isr_max_us;       // 擷取期間 FOC 中斷的最長執行時間
uint32_t log_isr_sum_us;
uint32_t log_isr_cnt;

// ---- SWD 介面：除錯器 (motor_ui) 直接讀寫這個結構。欄位順序與大小同後代，需與 motor_ui/link.py 的 SWD_IF 一致 ----
#define SWD_IF_MAGIC   0x31445753  // "SWD1"
#define SWD_IF_VERSION 1
#define SWD_REPLY_N    256
typedef struct {
  uint32_t magic;
  uint16_t version;
  uint16_t size;                // sizeof(SwdIf)
  volatile uint32_t cmd_seq;
  volatile uint32_t cmd_ack;
  char cmd_line[CMD_LINE_MAX];
  volatile uint32_t reply_total;
  char reply[SWD_REPLY_N];
  uint32_t log_buf_addr;
  uint32_t log_buf_bytes;
  uint32_t log_mask;
  uint32_t log_cap;
  volatile uint32_t log_total;  // 這次擷取 / 串流累計寫入的筆數，由 FOC 中斷更新
  uint16_t log_rec_size;
  uint16_t log_div;
  uint8_t log_state;
  uint8_t ctrl_mode;
  uint8_t out_state;
  uint8_t cal_state;            // 這一代沒有自校正，固定 0
  uint32_t foc_hz;
  float speed_target;
  int32_t pos_target_count;
  int32_t pos_count;
  float adc_bias1;
  float adc_bias2;
  uint32_t isr_max_us;
  uint32_t uptime_ms;           // HAL_GetTick()，變小代表 MCU 重置過
} SwdIf;
_Static_assert(sizeof(SwdIf) == 368, "SwdIf 的大小變了，motor_ui/link.py 要同步");

SwdIf swd_if;

// ---- 電氣零點：每次開機以 SetAng(0) 對齊 ----
#define ALIGN_HOLD_MS      500     // SetAng(0) 後等轉子停穩的時間
#define ALIGN_SAMPLES      32      // 對齊後平均幾筆角度當零點
#define ALIGN_MAX_SPEED    1.0f    // ALIGN 指令只在轉速低於這個值 (圈/秒) 時接受

// SET / GET 指令共用的迴路順序 (同後代)
#define CFG_PID_N 4
PIDController * const cfg_pid[CFG_PID_N] = {&PID__current_Id, &PID__current_Iq, &PID__velocity, &PID__position};
const char * const cfg_pid_name[CFG_PID_N] = {"ID", "IQ", "VEL", "POS"};

// ---- CAN：指令處理沿用舊程式，只收 StdId == can_id 的訊框 ----
enum DeviceMode deviceMode = DeviceMode_Stop;  // CAN 指令會改，但沒有其他地方使用 (同舊程式)
CAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8] = {};
uint32_t TxMailbox; // 用來儲存發送的郵箱號
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];

// CAN ID (11-bit 標準 ID) 與狀態回報週期，SET CANID / SET CANRPT 立即生效並寫進 EEPROM 0x0020，開機時讀回。
// 回報訊框用同一個 ID (配合平衡車)，每 can_rpt_ms 由 SysTick (Can_Tick) 送出 8 byte (LE)：
//   byte0~3 速度 float (圈/秒)、byte4~5 單圈角度 int16 (0.01 度，多圈位置折回 -180 ~ +180)、byte6~7 Iq int16 (mA，限幅)
#define CAN_ID_DEFAULT   0x110   // 平衡車的左輪；右輪 0x111 用 SET CANID 改
#define CAN_ID_MAX       0x7FF
// 回報 ID = 命令 ID + 0x10 (左 0x110 → 0x120、右 0x111 → 0x121)：同一個 ID 只能由一個節點送，避免和平衡車的命令撞在一起
#define CAN_RPT_ID_OFFSET 0x10
#define CAN_CMD_ID_MAX   (CAN_ID_MAX - CAN_RPT_ID_OFFSET)  // 命令 ID 上限，回報 ID 才不會超過 11 bit
#define CAN_RPT_DEFAULT  5       // ms，0 = 不回報
#define CAN_RPT_MAX      1000
#define CAN_EE_MAGIC     0xCA1E  // 0xCA1D 是回報格式改版前的設定，讀到時改用預設值
typedef struct {
  uint16_t magic;
  uint16_t id;
  uint16_t rpt_ms;
  uint16_t check;   // ~(id ^ rpt_ms)
} CanCfg;
_Static_assert(sizeof(CanCfg) == 8, "CanCfg 的大小變了，doc/設計文件.html 第 11 節要同步");
volatile uint16_t can_id = CAN_ID_DEFAULT;
volatile uint16_t can_rpt_ms = CAN_RPT_DEFAULT;
static uint16_t can_rpt_cnt;
static volatile uint8_t can_started;   // HAL_CAN_Start 成功之後才回報

void Cmd_Process(void);
void Log_Process(void);
static void Fault_Process(void);
static void Swd_Init(void);
static void Swd_Update(void);
static void Cmd_Reply(const char *msg);
static void Cmd_Exec(char *cmd_line);
static uint8_t Align_Do(void);
static void As5600_Report(void);
static void CAN_Filter_Config(void);
static void Can_Process(void);
static void Ctrl_Sync(void);

// 以 double 換算：圈數多時 float 的解析度不夠
static inline double Pos_CountToDeg(int32_t count)
{
  return (double)count * (360.0 / ENC_CPR);
}

// 多圈位置折回 -180 ~ +180 度並回傳。只改圈數，圈內的角度不變。呼叫時要關中斷
static inline int32_t Pos_Fold(void)
{
  int32_t pos = (int32_t)((uint32_t)pos_count & (ENC_CPR - 1));
  if (pos >= ENC_CPR / 2) {
    pos -= ENC_CPR;
  }
  pos_count = pos;
  return pos;
}

// 輸出關閉：PWM_ACTIVE 拉 Low (三相高阻) 與三相比較值 0
static inline void Pwm_Off(void)
{
  HAL_GPIO_WritePin(PWM_ACTIVE_GPIO_Port, PWM_ACTIVE_Pin, GPIO_PIN_RESET);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// 只收標準 ID == can_id 的訊框 (RTR 不看，同舊程式)。執行中也可以重設
static void CAN_Filter_Config(void)
{
  CAN_FilterTypeDef sFilterConfig;
  sFilterConfig.FilterBank = 0;      //筛选器编号, CAN1是0-13, CAN2是14-27
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK; //采用掩码模式
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; //设置筛选器的尺度, 采用32位
  // 32 bit 格式：STID 在 bit 31~21、IDE 在 bit 2、RTR 在 bit 1
  sFilterConfig.FilterIdHigh = (uint32_t)can_id << 5;
  sFilterConfig.FilterIdLow = 0x0000;     // IDE = 0 (標準 ID)
  sFilterConfig.FilterMaskIdHigh = CAN_ID_MAX << 5;
  sFilterConfig.FilterMaskIdLow = CAN_ID_EXT;  // 比對 IDE，擴充 ID 的訊框不收
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0; //设置经过筛选后数据存储到哪个接收FIFO
  sFilterConfig.FilterActivation = ENABLE;   //是否使能本筛选器
  sFilterConfig.SlaveStartFilterBank = 14;   //指定为CAN1分配多少个滤波器组
  if(HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

// 開機時從 EEPROM 讀回 CAN ID 與回報週期；讀不到、格式不符或超出範圍就用預設值。回傳 1 = 用了 EEPROM 的值
static uint8_t Can_Load(void)
{
  CanCfg c;
  if (M24C64_Read(EE_ADDR_CAN, (uint8_t *)&c, sizeof(c)) != HAL_OK || c.magic != CAN_EE_MAGIC
      || c.check != (uint16_t)~(c.id ^ c.rpt_ms) || c.id < 1 || c.id > CAN_CMD_ID_MAX || c.rpt_ms > CAN_RPT_MAX) {
    return 0;
  }
  can_id = c.id;
  can_rpt_ms = c.rpt_ms;
  return 1;
}

static HAL_StatusTypeDef Can_Save(void)
{
  CanCfg c = {CAN_EE_MAGIC, can_id, can_rpt_ms, (uint16_t)~(can_id ^ can_rpt_ms)};
  return M24C64_Write(EE_ADDR_CAN, (const uint8_t *)&c, sizeof(c));
}

static inline int16_t Sat_I16(float v)
{
  if (!(v > -32768.f)) return -32768;  // 也擋掉 NaN
  if (v > 32767.f) return 32767;
  return (int16_t)v;
}

// SysTick (1 ms，最低優先權) 呼叫：CAN 力矩命令逾時歸零；每 can_rpt_ms 送一筆狀態回報。
// 三個發送郵箱都滿 (沒有其他節點 ACK) 時這一筆跳過
void Can_Tick(void)
{
  __disable_irq();  // CAN 接收中斷 (優先權較高) 可能剛好送來新命令
  if (can_trq_live && HAL_GetTick() - can_trq_tick > TRQ_CAN_TIMEOUT_MS) {
    trq_iq = 0.f;
    can_trq_live = 0;
  }
  __enable_irq();
  uint16_t rpt = can_rpt_ms;
  if (!can_started || rpt == 0 || ++can_rpt_cnt < rpt) {
    return;
  }
  can_rpt_cnt = 0;
  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0) {
    return;
  }
  float spd = ang_speed;
  // 單圈角度：count 的低 16 bit 當 int16 (-32768 ~ 32767)，× 36000 / 65536 = × 1125 / 2048 (0.01 度)
  int16_t ang = (int16_t)(((int32_t)(int16_t)(uint16_t)pos_count * 1125) >> 11);
  int16_t iq = Sat_I16((float)iq_fix * (I_UNIT_A * 1000.f));
  uint8_t d[8];
  memcpy(&d[0], &spd, 4);
  memcpy(&d[4], &ang, 2);
  memcpy(&d[6], &iq, 2);
  TxHeader.StdId = can_id + CAN_RPT_ID_OFFSET;
  HAL_CAN_AddTxMessage(&hcan, &TxHeader, d, &TxMailbox);
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
  Swd_Init();

  // TIM1 中心對齊：一個 PWM 週期 = 2 × ARR 個計數。TIM1 在 APB2，APB2 不分頻時計時器時脈 = PCLK2
  uint32_t timer_clk = HAL_RCC_GetPCLK2Freq();
  if ((RCC->CFGR & RCC_CFGR_PPRE2) != RCC_CFGR_PPRE2_DIV1) {
    timer_clk *= 2;
  }
  pwm_freq = timer_clk / (htim1.Init.Prescaler + 1) / (2 * htim1.Init.Period);
  foc_hz = pwm_freq / FOC_DIV;
  foc_dt = 1.f / (float)foc_hz;
  foc_inv_dt = (float)foc_hz;
  v_fix_per_volt = (float)htim1.Init.Period * 0.1f * (1 << V_SHIFT);
  log_k_i = (int32_t)(I_UNIT_A * LOG_SCALE_I * 65536.f);
  log_k_iph = (int32_t)(I_UNIT_A * LOG_SCALE_IPH * 65536.f);
  log_k_u = (int32_t)(LOG_SCALE_U / v_fix_per_volt * 65536.f);

  init_micros_timer(); // 初始化微秒計時器 (TIM2)
  // DWT 週期計數器：量 FOC 中斷的執行時間
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  // 電流環：ITEST 電壓步階實測 R 3.29 Ω、L 約 0.87 mH (以 10 V 基準計，2026-10-14)。P = L·ωc、I = R·ωc，
  // ωc 取 2π × 750 Hz (作者 2026-10-14 決定)：d 軸 0 → 0.3 A 進入 ±5% 0.45 ms、過衝 < 1%。
  // 1 kHz (P 5.5 / I 20700，後代的頻寬) 是 0.30 ms，但電壓命令的雜訊比 500 Hz 多約 70%。限幅同後代 5.0
  PIDController_init(&PID__current_Id, 4.1, 15500, 0, 0, 5.0);
  PIDController_init(&PID__current_Iq, 4.1, 15500, 0, 0, 5.0);
  // 速度環 P = 0.2、I = 25、VELTF 0.5 ms (2026-10-14 實測)：0.1 圈/秒 時位置對等速直線的偏差 0.10 度 rms
  // (約 1 個 AS5600 LSB；原本的 0.1 / 4 是 0.59 度，每圈 14、24 次的起伏推不動，低速一卡一卡)；
  // 1 → 5 圈/秒 的步階過衝 46%、90 ms 進入 ±5%。I = 53 (後代的值) 會振盪。代價是 1 ~ 5 圈/秒 時
  // Iq 命令的雜訊約 ±200 mA (AS5600 量化 × P)
  // 位置環 P = 0.15 (後代 0.2)：配上面的速度環時 0.2 會在目標附近 ±1 度來回晃；0.15 時 90 度的 MOVE
  // 92 ms 進入 ±1 度、166 ms 進入 ±0.1 度，過衝 1 LSB
  PIDController_init(&PID__velocity, 0.2, 25., 0., 0, 1.);
  PIDController_init(&PID__position, 0.15, 0., 0.000, 0, 10);
  LowPassFilter_Init(&filter_Speed, VEL_TF_DEFAULT);

#if EXF_MAP_ENABLE
  if (ReadPreF(exf_map) != HAL_OK) {
    memset(exf_map, 0, sizeof(exf_map));
    Cmd_Reply("[EXF] ERR EEPROM");
  }
#endif

  // AS5600：讀一次角度 (同時把位址指標指到 ANGLE)，之後由 FOC 中斷在背景連續讀
  as5600.hi2c = &hi2c2;
  as5600.scl_port = GPIOB;
  as5600.scl_pin = GPIO_PIN_10;
  as5600.sda_port = GPIOB;
  as5600.sda_pin = GPIO_PIN_11;
  if (AS5600_Init(&as5600, (uint32_t)micros()) != AS5600_OK) {
    Cmd_Reply("[AS5600] ERR I2C");
  }
  As5600_Report();

  // ADC1 injected：TIM1 CH4 (OC4REF) 上升沿經 TRGO 觸發，轉換完由 ADC1_2_IRQHandler 呼叫 Foc_AdcIsr()
  if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK) { Error_Handler(); }
  if (HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK) { Error_Handler(); }

  /**
   * PWM處理
   */
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);    // U
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);    // V
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);    // W
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);  // 沒有輸出腳 (PA11 是 CAN_RX)，只用 OC4REF 觸發 ADC

  /**
   * DRV8317HREER 設定
   */
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
  HAL_Delay(DRV_WAKE_SETTLE_MS);
  // 電流偏置取多筆平均：單筆有幾個 LSB 的雜訊，會變成電氣角 1 倍的轉矩漣波 (同後代)
  uint32_t bias_sum_CH1 = 0;
  uint32_t bias_sum_CH2 = 0;
  for (int i = 0; i < ADC_BIAS_SAMPLES; i++) {
    HAL_Delay(1);
    __disable_irq();
    uint16_t s1 = adc_raw1, s2 = adc_raw2;  // 兩個通道來自同一次轉換
    __enable_irq();
    bias_sum_CH1 += s1;
    bias_sum_CH2 += s2;
  }
  adc_bios_CH1 = (float)bias_sum_CH1 / ADC_BIAS_SAMPLES;
  adc_bios_CH2 = (float)bias_sum_CH2 / ADC_BIAS_SAMPLES;
  Ctrl_Sync();
  HAL_GPIO_WritePin(PWM_ACTIVE_GPIO_Port, PWM_ACTIVE_Pin, GPIO_PIN_SET);

  // 電氣零點：每次開機以 SetAng(0) 對齊
  Cmd_Reply(Align_Do() ? "[ZERO] ALIGN" : "[ZERO] ERR ANG");

  char can_msg[48];
  uint8_t can_ee = Can_Load();
  snprintf(can_msg, sizeof(can_msg), "[CAN] ID=%u RPT=%u %s", can_id, can_rpt_ms, can_ee ? "EEPROM" : "DEFAULT");
  Cmd_Reply(can_msg);
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
  TxHeader.StdId = can_id + CAN_RPT_ID_OFFSET; // 回報 ID = 命令 ID + 0x10
  TxHeader.RTR = CAN_RTR_DATA; // 數據幀
  TxHeader.IDE = CAN_ID_STD; // 標準ID
  TxHeader.DLC = 8; // 數據長度，最多8個位元組
  TxHeader.TransmitGlobalTime = DISABLE;
  can_started = 1;
  deviceMode = DeviceMode_SpeedMode;

  ang_stall_tick = HAL_GetTick();
  FocDo = 1;

  HAL_UART_Receive_IT(&huart1, &cmd_rx_byte, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    Cmd_Process();
    Can_Process();
    Log_Process();
    Swd_Update();
    Fault_Process();
    Ctrl_Sync();

    // LOGQ 擷取期間暫停數據輸出
    if (!(log_quiet && log_state != LOG_IDLE)) {
      __disable_irq();
      float angle = (float)pos_enc_prev * POS_DEG_PER_COUNT;
      uint16_t a1 = adc_raw1, a2 = adc_raw2;
      int32_t c1 = cur1_fix, c2 = cur2_fix, iq = iq_fix;
      float spd = ang_speed;
      int32_t tgt = pos_target_count, pos = pos_count;
      __enable_irq();
      // 欄位同後代：角度、ADC CH1/CH2、三相電流、速度、Iq、位置目標、目前多圈位置 (度)
      printf("%.2f,\t%u,\t%u,\t%.2f,\t%.2f,\t%.2f,\t%.2f,\t%.4f,\t%.2f,\t%.2f\n",
        angle, a1, a2, c1 * I_UNIT_A, c2 * I_UNIT_A, -(c1 + c2) * I_UNIT_A, spd, iq * I_UNIT_A,
        Pos_CountToDeg(tgt), Pos_CountToDeg(pos));
    }
    HAL_Delay(5);
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

  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_9;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedNbrOfConversion = 2;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_TRGO;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_8;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
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
  hcan.Init.AutoBusOff = ENABLE;  // bus-off 後由硬體自動恢復 (關閉時要重新上電才會回到匯流排)
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
  hi2c2.Init.ClockSpeed = 400000;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;
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
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 1695;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 1, 0);
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
// printf 經 USART1 DMA 送出。上一筆還沒送完時這一筆直接丟掉 (數據行是抽樣輸出)；
// 先複製到自己的緩衝區，printf 回來改寫它的緩衝區時才不會改到傳送中的資料
int _write(int file, char *ptr, int len)
{
  if (huart1.gState == HAL_UART_STATE_READY) {
    int n = (len < (int)sizeof(uart_tx_buf)) ? len : (int)sizeof(uart_tx_buf);
    memcpy(uart_tx_buf, ptr, n);
    HAL_UART_Transmit_DMA(&huart1, (uint8_t*)uart_tx_buf, n);
  }
  return len;
}

static void Swd_Init(void)
{
  swd_if.version = SWD_IF_VERSION;
  swd_if.size = sizeof(SwdIf);
  swd_if.log_buf_addr = (uint32_t)log_buf;
  swd_if.log_buf_bytes = LOG_BUF_BYTES;
  swd_if.magic = SWD_IF_MAGIC;
}

// 主迴圈每次更新給 SWD 讀的狀態
static void Swd_Update(void)
{
  swd_if.log_mask = log_mask;
  swd_if.log_cap = log_cap;
  swd_if.log_rec_size = log_rec_size;
  swd_if.log_div = log_div;
  swd_if.log_state = log_state;
  swd_if.ctrl_mode = ctrl_mode;
  swd_if.out_state = out_state;
  swd_if.cal_state = 0;
  swd_if.foc_hz = foc_hz;
  swd_if.speed_target = speed_Target;
  swd_if.pos_target_count = pos_target_count;
  swd_if.pos_count = pos_count;
  swd_if.adc_bias1 = adc_bios_CH1;
  swd_if.adc_bias2 = adc_bios_CH2;
  swd_if.isr_max_us = log_isr_max_us;
  swd_if.uptime_ms = HAL_GetTick();
}

static void Swd_ReplyPut(const char *s, uint32_t len)
{
  uint32_t total = swd_if.reply_total;
  for (uint32_t i = 0; i < len; i++) {
    swd_if.reply[(total + i) % SWD_REPLY_N] = s[i];
  }
  swd_if.reply_total = total + len;
}

// 指令回應：等前一筆 printf 的 DMA 傳送結束後，以阻塞方式送出一行；同時寫一份給 SWD
static void Cmd_Reply(const char *msg)
{
  Swd_ReplyPut(msg, strlen(msg));
  Swd_ReplyPut("\n", 1);
  uint32_t t0 = HAL_GetTick();
  while (huart1.gState != HAL_UART_STATE_READY && HAL_GetTick() - t0 < 50) {}
  HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
  HAL_UART_Transmit(&huart1, (uint8_t*)"\n", 1, 100);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    char c = (char)cmd_rx_byte;
    if (c == '\n' || c == '\r') {
      // 上一行還沒被主迴圈處理完時，新的一行直接丟棄
      if (cmd_rx_len > 0 && !cmd_rx_overflow && !cmd_line_ready) {
        memcpy(cmd_line, cmd_rx_buf, cmd_rx_len);
        cmd_line[cmd_rx_len] = '\0';
        cmd_line_ready = 1;
      }
      cmd_rx_len = 0;
      cmd_rx_overflow = 0;
    } else if (cmd_rx_len < CMD_LINE_MAX - 1) {
      cmd_rx_buf[cmd_rx_len++] = c;
    } else {
      cmd_rx_overflow = 1;  // 超長的行整行丟棄
    }
    HAL_UART_Receive_IT(&huart1, &cmd_rx_byte, 1);
  }
}

// Overrun 等錯誤會讓 HAL 停止接收，這裡重新啟動
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    cmd_rx_len = 0;
    cmd_rx_overflow = 0;
    HAL_UART_Receive_IT(&huart1, &cmd_rx_byte, 1);
  }
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C2) {
    AS5600_RxDone(&as5600, (uint32_t)micros());
  }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C2) {
    AS5600_RxError(&as5600);
  }
}

// TIM2 溢位：micros() 的高位
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2) {
    overflow_handler();
  }
}

// 讀出 AS5600 的狀態與設定暫存器並印出 (確認磁鐵距離與晶片設定用)
static void As5600_Report(void)
{
  uint8_t st, agc, conf[2], zpos[2], mpos[2];
  AS5600_Pause(&as5600);
  uint8_t ok = AS5600_ReadReg(&as5600, AS5600_REG_STATUS, &st, 1) == AS5600_OK
            && AS5600_ReadReg(&as5600, AS5600_REG_AGC, &agc, 1) == AS5600_OK
            && AS5600_ReadReg(&as5600, AS5600_REG_CONF, conf, 2) == AS5600_OK
            && AS5600_ReadReg(&as5600, AS5600_REG_ZPOS, zpos, 2) == AS5600_OK
            && AS5600_ReadReg(&as5600, AS5600_REG_MPOS, mpos, 2) == AS5600_OK;
  AS5600_Resume(&as5600, (uint32_t)micros());
  char msg[80];
  if (ok) {
    snprintf(msg, sizeof(msg), "[AS5600] STATUS=0x%02X AGC=%u CONF=0x%02X%02X ZPOS=%u MPOS=%u",
      st, agc, conf[0], conf[1], ((zpos[0] & 0x0F) << 8) | zpos[1], ((mpos[0] & 0x0F) << 8) | mpos[1]);
  } else {
    snprintf(msg, sizeof(msg), "[AS5600] ERR I2C");
  }
  Cmd_Reply(msg);
}

// SetAng(0) 把轉子拉到電氣零點，平均 ALIGN_SAMPLES 筆角度當零點，之後 FOC 中斷重新起算位置。
// 呼叫前 FocDo 必須為 0。讀不到角度時回傳 0
static uint8_t Align_Do(void)
{
  SetAng(0);
  HAL_Delay(ALIGN_HOLD_MS);
  uint32_t seq = as5600.seq;
  uint16_t first = as5600.raw;
  int32_t sum = 0;
  int n = 0;
  uint32_t t0 = HAL_GetTick();
  while (n < ALIGN_SAMPLES && HAL_GetTick() - t0 < 100) {
    if (as5600.seq != seq) {
      seq = as5600.seq;
      // 以第一筆為基準取差值 (-2048 ~ 2047)，零點在 0 / 4095 交界時也平均得對
      int32_t d = (int32_t)((as5600.raw - first + AS5600_CPR / 2) & (AS5600_CPR - 1)) - AS5600_CPR / 2;
      sum += d;
      n++;
    }
  }
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);    // U
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);    // V
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);    // W
  if (n == 0) {
    return 0;
  }
  int32_t avg = (sum >= 0) ? (sum + n / 2) / n : -((-sum + n / 2) / n);
  zero_raw = (uint16_t)((first + avg) & (AS5600_CPR - 1));
  foc_reseed = 1;
  return 1;
}

static void Align_Start(void)
{
  // 輸出關閉時 SetAng(0) 拉不動轉子，對出來的零點是錯的
  if (log_state != LOG_IDLE || out_state != OUT_RUN || itest_mode != ITEST_OFF) {
    Cmd_Reply("[ALIGN] BUSY");
    return;
  }
  if (fabsf(ang_speed) > ALIGN_MAX_SPEED) {
    Cmd_Reply("[ALIGN] ERR SPEED");
    return;
  }
  FocDo = 0;
  uint8_t ok = Align_Do();
  for (int i = 0; i < CFG_PID_N; i++) {
    PIDController_reset(cfg_pid[i]);
  }
  iq_pi_on = id_pi_on = 0;  // 電流 PI (定點) 也從 0 開始
  FocDo = 1;
  Cmd_Reply(ok ? "[ALIGN] OK" : "[ALIGN] ERR ANG");
}

// 解析一個整數 (十進位或 0x 開頭的十六進位)，成功時回傳 1 並把 *s 移到數字之後
static uint8_t Cmd_ParseUint(char **s, uint32_t *out)
{
  char *end;
  unsigned long v = strtoul(*s, &end, 0);
  if (end == *s) {
    return 0;
  }
  *s = end;
  *out = v;
  return 1;
}

// 掃描 I2C1 (EEPROM 那一條) 上所有 7-bit 位址，列出有回 ACK 的裝置。I2C2 正在連續讀 AS5600，不掃
static void I2c_Scan(void)
{
  char msg[24 + 16 * 5] = "[I2C] SCAN:";
  int n = strlen(msg);
  int found = 0;
  for (uint16_t addr = 0x08; addr <= 0x77; addr++) {
    if (HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 1, 2) == HAL_OK && found < 16) {
      n += snprintf(msg + n, sizeof(msg) - n, " 0x%02X", addr);
      found++;
    }
  }
  Cmd_Reply(found ? msg : "[I2C] SCAN NONE");
}

static float *Pid_Field(PIDController *pid, const char *field)
{
  if (strcmp(field, "P") == 0) return &pid->P;
  if (strcmp(field, "I") == 0) return &pid->I;
  if (strcmp(field, "D") == 0) return &pid->D;
  if (strcmp(field, "RAMP") == 0) return &pid->output_ramp;
  if (strcmp(field, "LIMIT") == 0) return &pid->limit;
  return NULL;
}

// name 以迴路名稱 (例如 "VEL") 開頭時回傳迴路編號，並把 *rest 指向名稱之後 (結尾或 '.')；否則回傳 -1
static int Pid_Index(const char *name, const char **rest)
{
  for (int i = 0; i < CFG_PID_N; i++) {
    size_t n = strlen(cfg_pid_name[i]);
    if (strncmp(name, cfg_pid_name[i], n) == 0 && (name[n] == '\0' || name[n] == '.')) {
      *rest = name + n;
      return i;
    }
  }
  return -1;
}

// "SPEED"、"BIAS1/2"、"MIT.KP/KD" 或 "<迴路>.<欄位>" 對應的變數，找不到時回傳 NULL
static float *Param_Find(const char *name)
{
  if (strcmp(name, "SPEED") == 0) {
    return &speed_Target;
  }
  // 直接改電流偏置，單位是 ADC LSB (同後代)
  if (strcmp(name, "BIAS1") == 0) {
    return &adc_bios_CH1;
  }
  if (strcmp(name, "BIAS2") == 0) {
    return &adc_bios_CH2;
  }
  if (strcmp(name, "MIT.KP") == 0) return &mit_kp;
  if (strcmp(name, "MIT.KD") == 0) return &mit_kd;
  // 電流環測試 (ITEST) 的準位；範圍在 Ctrl_Sync() 依測試的種類限制
  if (strcmp(name, "IT.QA") == 0) return &it_qa;
  if (strcmp(name, "IT.QB") == 0) return &it_qb;
  if (strcmp(name, "IT.DA") == 0) return &it_da;
  if (strcmp(name, "IT.DB") == 0) return &it_db;
  const char *rest;
  int i = Pid_Index(name, &rest);
  if (i < 0 || *rest != '.') {
    return NULL;
  }
  return Pid_Field(cfg_pid[i], rest + 1);
}

// 進入位置模式：目標設為目前位置，馬達停在原地
static void Pos_Enter(void)
{
  PIDController_reset(&PID__position);
  pos_speed_cmd = 0.f;
  pos_div_cnt = 0;
  pos_target_count = pos_count;
  ctrl_mode = CTRL_MODE_POS;
}

// POSZERO：多圈位置折回 -180 ~ +180 度，轉子不動。位置目標設成折回後的位置，位置模式下還沒走完的移動會被取消
static void Pos_CmdZero(void)
{
  __disable_irq();
  int32_t pos = Pos_Fold();
  pos_target_count = pos;
  if (ctrl_mode == CTRL_MODE_POS) {
    // 誤差從移動途中的值跳到 0，不重置的話微分項會打一下
    PIDController_reset(&PID__position);
    pos_speed_cmd = 0.f;
    pos_div_cnt = 0;
  }
  __enable_irq();
  char msg[48];
  snprintf(msg, sizeof(msg), "[PARAM] POSNOW=%.2f", Pos_CountToDeg(pos));
  Cmd_Reply(msg);
}

// 切換控制模式 (0 速度、1 位置、5 力矩)。電流環測試期間只能留在速度模式，回傳 0。進入力矩模式時不改 IQREF
static uint8_t Mode_Set(uint8_t mode)
{
  if (mode == ctrl_mode) {
    return 1;
  }
  // 電流環測試只在速度模式下進行
  if (itest_mode != ITEST_OFF) {
    return 0;
  }
  // MIT 與力矩模式沒有在跑速度環，積分還停在進入之前
  if (ctrl_mode == CTRL_MODE_TRQ || (ctrl_mode == CTRL_MODE_POS && pos_type == POS_TYPE_MIT)) {
    PIDController_reset(&PID__velocity);
  }
  if (mode == CTRL_MODE_POS) {
    Pos_Enter();
  } else if (mode == CTRL_MODE_TRQ) {
    ctrl_mode = CTRL_MODE_TRQ;
  } else {
    speed_Target = 0.f;  // 切回速度模式時停在原地，要轉動再下 SET SPEED
    ctrl_mode = CTRL_MODE_VEL;
  }
  return 1;
}

// MODE / ANGLE / MOVE / POSDIV / POSTYPE：不是單純寫入一個 float 的參數。name 是這幾個之一時回傳 1 (已回應)
static uint8_t Pos_ParamSet(const char *name, float value)
{
  char msg[48];
  if (strcmp(name, "MODE") == 0) {
    // 後代的 MODE 2 ~ 4 (棘輪 / 限位 / 無摩擦) 這一代還沒有；MODE 5 (力矩) 這一代才有
    if (value != CTRL_MODE_VEL && value != CTRL_MODE_POS && value != CTRL_MODE_TRQ) {
      Cmd_Reply("[PARAM] ERR PARAM");
      return 1;
    }
    if (value != ctrl_mode) {
      if (!Mode_Set((uint8_t)value)) {
        Cmd_Reply("[PARAM] BUSY");
        return 1;
      }
      if (value == CTRL_MODE_TRQ) {
        trq_iq = 0.f;  // 以 SET MODE 5 進入時從 0 開始
        can_trq_live = 0;
      }
    }
    snprintf(msg, sizeof(msg), "[PARAM] MODE=%u", ctrl_mode);
    Cmd_Reply(msg);
    return 1;
  }
  if (strcmp(name, "IQREF") == 0) {
    // 不在力矩模式時自動切過去 (同 ANGLE)。文字指令設的值不會逾時歸零
    if (!Mode_Set(CTRL_MODE_TRQ)) {
      Cmd_Reply("[PARAM] BUSY");
      return 1;
    }
    __disable_irq();
    trq_iq = value;
    can_trq_live = 0;
    __enable_irq();
    snprintf(msg, sizeof(msg), "[PARAM] IQREF=%.6g", value);
    Cmd_Reply(msg);
    return 1;
  }
  if (strcmp(name, "POSTYPE") == 0) {
    if (value != POS_TYPE_CASCADE && value != POS_TYPE_MIT) {
      Cmd_Reply("[PARAM] ERR PARAM");
      return 1;
    }
    if (value != pos_type) {
      __disable_irq();
      // 兩種做法的內部狀態不通用，從目前的誤差重新開始
      PIDController_reset(&PID__position);
      PIDController_reset(&PID__velocity);
      pos_speed_cmd = 0.f;
      pos_div_cnt = 0;
      pos_type = (uint8_t)value;
      __enable_irq();
    }
    snprintf(msg, sizeof(msg), "[PARAM] POSTYPE=%u", pos_type);
    Cmd_Reply(msg);
    return 1;
  }
  uint8_t relative = (strcmp(name, "MOVE") == 0);
  if (relative || strcmp(name, "ANGLE") == 0) {
    // MOVE 以目前的目標為基準；速度模式下沒有目標，以目前位置為基準
    double target = (double)value * (ENC_CPR / 360.0);
    if (relative) {
      target += (ctrl_mode == CTRL_MODE_POS) ? pos_target_count : pos_count;
    }
    if (fabs(target) > POS_TARGET_MAX) {
      Cmd_Reply("[PARAM] ERR PARAM");
      return 1;
    }
    // 速度模式下收到位置目標時自動切到位置模式
    if (ctrl_mode != CTRL_MODE_POS) {
      Pos_Enter();
    }
    pos_target_count = (int32_t)lround(target);
    snprintf(msg, sizeof(msg), "[PARAM] ANGLE=%.2f", Pos_CountToDeg(pos_target_count));
    Cmd_Reply(msg);
    return 1;
  }
  if (strcmp(name, "POSDIV") == 0) {
    if (value < 1 || value > POS_DIV_MAX || value != floorf(value)) {
      Cmd_Reply("[PARAM] ERR PARAM");
      return 1;
    }
    pos_div = (uint16_t)value;
    snprintf(msg, sizeof(msg), "[PARAM] POSDIV=%u", pos_div);
    Cmd_Reply(msg);
    return 1;
  }
  return 0;
}

// 浮點數 → Q16 定點，超出 int32 時限幅
static int32_t Fix_Q16(float v)
{
  float q = v * 65536.f;
  if (!(q < 2.0e9f)) return 2000000000;  // 也擋掉 NaN
  if (q < -2.0e9f) return -2000000000;
  return (int32_t)q;
}

// 主迴圈每次呼叫：把 SET 改過的浮點參數換成 FOC 中斷用的係數 (中斷內不做除法)，最慢 5 ms 後生效。
//   電流 PI：IQ / ID 的 P、I、LIMIT (D、RAMP 這一代的電流環不使用)
//   電流偏置 BIAS1 / BIAS2、電流回授低通 IPHTF / IDQTF、速度低通 VELTF、目標權重 (VEL.P / VEL.I)、位置環週期 (POSDIV)
static void Ctrl_Sync(void)
{
  // 電流 1 單位 = I_UNIT_A 安培，電壓 1 單位 = 1 / v_fix_per_volt (10 V 基準的單位)
  float kv = v_fix_per_volt * I_UNIT_A;
  float half_ts = 0.5f * foc_dt;
  int32_t iq_kp = Fix_Q16(PID__current_Iq.P * kv), iq_ki = Fix_Q16(PID__current_Iq.I * half_ts * kv);
  int32_t id_kp = Fix_Q16(PID__current_Id.P * kv), id_ki = Fix_Q16(PID__current_Id.I * half_ts * kv);
  int32_t iq_lim = Fix_Q16(PID__current_Iq.limit * v_fix_per_volt) >> 16;
  int32_t id_lim = Fix_Q16(PID__current_Id.limit * v_fix_per_volt) >> 16;
  int32_t b1 = (int32_t)lroundf(adc_bios_CH1 * (1 << CUR_SHIFT));
  int32_t b2 = (int32_t)lroundf(adc_bios_CH2 * (1 << CUR_SHIFT));
  int32_t k1 = Fix_Q16(foc_dt / (iph_tf + foc_dt));
  int32_t k2 = Fix_Q16(foc_dt / (idq_tf + foc_dt));
  float vel_dt = foc_dt * VEL_DIV;
  float tf = filter_Speed.Tf;
  float a = tf / (tf + vel_dt);
  float vk = (PID__velocity.I > 0.f) ? vel_dt / (PID__velocity.P / PID__velocity.I + vel_dt) : 1.f;
  // 位置環實際的週期：pos_div 以 VEL_DIV 為單位進位
  float pts = foc_dt * (float)(((pos_div + VEL_DIV - 1) / VEL_DIV) * VEL_DIV);
  float pinv = 1.f / pts;
  const float it_lv[4] = {it_qa, it_qb, it_da, it_db};
  int32_t it_f[2][4];
  for (int i = 0; i < 4; i++) {
    it_f[0][i] = (int32_t)(constrain(it_lv[i], -ITEST_I_MAX, ITEST_I_MAX) * (1.f / I_UNIT_A));
    it_f[1][i] = (int32_t)(constrain(it_lv[i], -ITEST_U_MAX, ITEST_U_MAX) * v_fix_per_volt);
  }
  __disable_irq();
  memcpy((void *)it_fix, it_f, sizeof(it_f));
  itest_i_trip_fix = (int32_t)(ITEST_I_TRIP * (1.f / I_UNIT_A));
  itest_spd_trip_q16 = (int32_t)(ITEST_SPEED_TRIP * ((float)ENC_CPR * 65536.f / 1e6f));
  pi_iq.kp = iq_kp;
  pi_iq.ki = iq_ki;
  pi_iq.limit = iq_lim;
  pi_id.kp = id_kp;
  pi_id.ki = id_ki;
  pi_id.limit = id_lim;
  bias1_fix = b1;
  bias2_fix = b2;
  iph_k = k1;
  idq_k = k2;
  spd_alpha = a;
  vel_lp_k = vk;
  pos_ts = pts;
  pos_inv_ts = pinv;
  __enable_irq();
}

static void Param_Set(char *args)
{
  char *sep = strchr(args, ' ');
  if (sep == NULL) {
    Cmd_Reply("[PARAM] ERR PARAM");
    return;
  }
  *sep = '\0';
  char *end;
  float value = strtof(sep + 1, &end);
  if (end == sep + 1 || *end != '\0' || !isfinite(value)) {
    Cmd_Reply("[PARAM] ERR PARAM");
    return;
  }
  if (Pos_ParamSet(args, value)) {
    return;
  }
  char msg[48];
  if (strcmp(args, "IDLOOP") == 0) {
    if (value != 0 && value != 1) {
      Cmd_Reply("[PARAM] ERR PARAM");
      return;
    }
    id_loop = (uint8_t)value;
    Cmd_Reply(id_loop ? "[PARAM] IDLOOP=1" : "[PARAM] IDLOOP=0");
    return;
  }
  if (strcmp(args, "VELAW") == 0 || strcmp(args, "VELB") == 0 || strcmp(args, "VELTF") == 0) {
    if (args[3] == 'A' && (value == 0 || value == 1)) {
      vel_aw = (uint8_t)value;
    } else if (args[3] == 'B' && value >= 0.f && value <= 1.f) {
      vel_b = value;
    } else if (args[3] == 'T' && value >= 0.f && value <= VEL_TF_MAX * 1e3f) {
      filter_Speed.Tf = value * 1e-3f;
    } else {
      Cmd_Reply("[PARAM] ERR PARAM");
      return;
    }
    snprintf(msg, sizeof(msg), "[PARAM] %s=%.6g", args, value);
    Cmd_Reply(msg);
    return;
  }
  if (strcmp(args, "IPHLPF") == 0 || strcmp(args, "IDQLPF") == 0 || strcmp(args, "IPHTF") == 0 || strcmp(args, "IDQTF") == 0) {
    uint8_t dq = (args[1] == 'D');
    if (args[3] == 'L' && (value == 0 || value == 1)) {
      if (dq) {
        idq_lpf = (uint8_t)value;
      } else {
        iph_lpf = (uint8_t)value;
      }
    } else if (args[3] == 'T' && value >= 0.f && value <= CUR_TF_MAX * 1e3f) {
      if (dq) {
        idq_tf = value * 1e-3f;
      } else {
        iph_tf = value * 1e-3f;
      }
    } else {
      Cmd_Reply("[PARAM] ERR PARAM");
      return;
    }
    snprintf(msg, sizeof(msg), "[PARAM] %s=%.6g", args, value);
    Cmd_Reply(msg);
    return;
  }
  uint8_t is_rpt = (strcmp(args, "CANRPT") == 0);
  if (is_rpt || strcmp(args, "CANID") == 0) {
    // 立即生效並寫進 EEPROM；寫入失敗時 RAM 內的值仍然生效，另外回 [EE] ERR WRITE
    if (value != floorf(value) || (is_rpt ? (value < 0 || value > CAN_RPT_MAX) : (value < 1 || value > CAN_CMD_ID_MAX))) {
      Cmd_Reply("[PARAM] ERR PARAM");
      return;
    }
    if (is_rpt) {
      can_rpt_ms = (uint16_t)value;
    } else {
      can_id = (uint16_t)value;
      CAN_Filter_Config();
    }
    snprintf(msg, sizeof(msg), "[PARAM] %s=%u", args, (unsigned)(is_rpt ? can_rpt_ms : can_id));
    Cmd_Reply(msg);
    if (Can_Save() != HAL_OK) {
      Cmd_Reply("[EE] ERR WRITE");
    }
    return;
  }
  float *var = Param_Find(args);
  if (var == NULL) {
    Cmd_Reply("[PARAM] ERR PARAM");
    return;
  }
  // 位置模式下速度由位置環決定
  if (var == &speed_Target && ctrl_mode != CTRL_MODE_VEL) {
    Cmd_Reply("[PARAM] BUSY");
    return;
  }
  if ((var == &mit_kp || var == &mit_kd) && value < 0.f) {
    Cmd_Reply("[PARAM] ERR PARAM");
    return;
  }
  *var = value;
  snprintf(msg, sizeof(msg), "[PARAM] %s=%.6g", args, value);
  Cmd_Reply(msg);
}

// GET <name>：name 只給迴路名稱時一次列出五個欄位
static void Param_Get(const char *name)
{
  char msg[120];
  const char *rest;
  int i = Pid_Index(name, &rest);
  if (i >= 0 && *rest == '\0') {
    PIDController *pid = cfg_pid[i];
    snprintf(msg, sizeof(msg), "[PARAM] %s P=%.6g I=%.6g D=%.6g RAMP=%.6g LIMIT=%.6g",
      name, pid->P, pid->I, pid->D, pid->output_ramp, pid->limit);
    Cmd_Reply(msg);
    return;
  }
  if (strcmp(name, "MODE") == 0) {
    snprintf(msg, sizeof(msg), "[PARAM] MODE=%u", ctrl_mode);
  } else if (strcmp(name, "POSTYPE") == 0) {
    snprintf(msg, sizeof(msg), "[PARAM] POSTYPE=%u", pos_type);
  } else if (strcmp(name, "MIT") == 0) {
    snprintf(msg, sizeof(msg), "[PARAM] MIT KP=%.6g KD=%.6g", mit_kp, mit_kd);
  } else if (strcmp(name, "ANGLE") == 0) {
    snprintf(msg, sizeof(msg), "[PARAM] ANGLE=%.2f", Pos_CountToDeg(pos_target_count));
  } else if (strcmp(name, "POSNOW") == 0) {
    snprintf(msg, sizeof(msg), "[PARAM] POSNOW=%.2f", Pos_CountToDeg(pos_count));
  } else if (strcmp(name, "POSDIV") == 0) {
    snprintf(msg, sizeof(msg), "[PARAM] POSDIV=%u", pos_div);
  } else if (strcmp(name, "IDLOOP") == 0) {
    snprintf(msg, sizeof(msg), "[PARAM] IDLOOP=%u", id_loop);
  } else if (strcmp(name, "VELAW") == 0) {
    snprintf(msg, sizeof(msg), "[PARAM] VELAW=%u", vel_aw);
  } else if (strcmp(name, "VELB") == 0) {
    snprintf(msg, sizeof(msg), "[PARAM] VELB=%.6g", vel_b);
  } else if (strcmp(name, "VELTF") == 0) {
    snprintf(msg, sizeof(msg), "[PARAM] VELTF=%.6g", filter_Speed.Tf * 1e3f);
  } else if (strcmp(name, "IPHLPF") == 0) {
    snprintf(msg, sizeof(msg), "[PARAM] IPHLPF=%u", iph_lpf);
  } else if (strcmp(name, "IDQLPF") == 0) {
    snprintf(msg, sizeof(msg), "[PARAM] IDQLPF=%u", idq_lpf);
  } else if (strcmp(name, "IPHTF") == 0) {
    snprintf(msg, sizeof(msg), "[PARAM] IPHTF=%.6g", iph_tf * 1e3f);
  } else if (strcmp(name, "IDQTF") == 0) {
    snprintf(msg, sizeof(msg), "[PARAM] IDQTF=%.6g", idq_tf * 1e3f);
  } else if (strcmp(name, "ITEST") == 0) {
    snprintf(msg, sizeof(msg), "[PARAM] ITEST=%u END=%u", itest_mode, itest_end);
  } else if (strcmp(name, "IQREF") == 0) {
    snprintf(msg, sizeof(msg), "[PARAM] IQREF=%.6g", trq_iq);
  } else if (strcmp(name, "CANID") == 0) {
    snprintf(msg, sizeof(msg), "[PARAM] CANID=%u", can_id);
  } else if (strcmp(name, "CANRPT") == 0) {
    snprintf(msg, sizeof(msg), "[PARAM] CANRPT=%u", can_rpt_ms);
  } else {
    msg[0] = '\0';
  }
  if (msg[0] != '\0') {
    Cmd_Reply(msg);
    return;
  }
  float *var = Param_Find(name);
  if (var == NULL) {
    Cmd_Reply("[PARAM] ERR PARAM");
    return;
  }
  snprintf(msg, sizeof(msg), "[PARAM] %s=%.6g", name, *var);
  Cmd_Reply(msg);
}

// 關閉輸出 (同後代)：COAST = 三相高阻，BRAKE = 下橋全開
static void Out_Stop(uint8_t state)
{
  FocDo = 0;
  if (itest_mode != ITEST_OFF) {
    itest_mode = ITEST_OFF;
    itest_end = ITEST_END_STOP;
    id_ref_fix = 0;
    iq_ref_fix = 0;
  }
  if (state == OUT_COAST) {
    HAL_GPIO_WritePin(PWM_ACTIVE_GPIO_Port, PWM_ACTIVE_Pin, GPIO_PIN_RESET);
  }
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);    // U
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);    // V
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);    // W
  if (state == OUT_BRAKE) {
    HAL_GPIO_WritePin(PWM_ACTIVE_GPIO_Port, PWM_ACTIVE_Pin, GPIO_PIN_SET);
  }
  out_state = state;
  Cmd_Reply(state == OUT_COAST ? "[OUT] COAST" : "[OUT] BRAKE");
}

// 恢復輸出：從靜止開始。速度目標歸零，位置模式的目標設為目前位置
static void Out_Run(void)
{
  if (out_state != OUT_RUN) {
    speed_Target = 0.f;
    if (!can_trq_live) {
      trq_iq = 0.f;  // 力矩也從 0 開始；CAN 力矩命令觸發的 RUN 則沿用剛收到的值
    }
    for (int i = 0; i < CFG_PID_N; i++) {
      PIDController_reset(cfg_pid[i]);
    }
    pos_speed_cmd = 0.f;
    pos_div_cnt = 0;
    pos_target_count = pos_count;
    HAL_GPIO_WritePin(PWM_ACTIVE_GPIO_Port, PWM_ACTIVE_Pin, GPIO_PIN_SET);
    out_state = OUT_RUN;
    FocDo = 1;
  }
  Cmd_Reply("[OUT] RUN");
}

// ITEST <mode> [half] [ms]：mode 0 = 結束，1 = 電流步階 (ITEST_CUR)，2 = 電壓步階 (ITEST_VOLT)；
// half 是半週期 (FOC 中斷次數)，ms 是總長度。準位先用 SET IT.QA / IT.QB / IT.DA / IT.DB 設好 (同後代)
static void Itest_Cmd(char *args)
{
  uint32_t mode, half = 0, ms = ITEST_MS_DEFAULT;
  uint8_t ok = Cmd_ParseUint(&args, &mode);
  if (ok && mode != ITEST_OFF) {
    ok = Cmd_ParseUint(&args, &half);
    if (ok && *args != '\0') {
      ok = Cmd_ParseUint(&args, &ms);
    }
  }
  if (!ok || *args != '\0' || mode > ITEST_VOLT) {
    Cmd_Reply("[ITEST] ERR PARAM");
    return;
  }
  if (mode == ITEST_OFF) {
    // 由 FOC 中斷在下一次執行時結束，速度環與電流環的積分在那裡重置
    __disable_irq();
    if (itest_mode != ITEST_OFF) {
      itest_left = 0;
      itest_end = ITEST_END_STOP;
    }
    __enable_irq();
    Cmd_Reply("[ITEST] STOP");
    return;
  }
  if (half < 1 || half > ITEST_HALF_MAX || ms < 1 || ms > ITEST_MS_MAX) {
    Cmd_Reply("[ITEST] ERR PARAM");
    return;
  }
  float max = (mode == ITEST_CUR) ? ITEST_I_MAX : ITEST_U_MAX;
  if (fabsf(it_qa) > max || fabsf(it_qb) > max || fabsf(it_da) > max || fabsf(it_db) > max) {
    Cmd_Reply("[ITEST] ERR LEVEL");
    return;
  }
  if (itest_mode != ITEST_OFF || ctrl_mode != CTRL_MODE_VEL || out_state != OUT_RUN || !FocDo) {
    Cmd_Reply("[ITEST] BUSY");
    return;
  }
  Ctrl_Sync();         // 剛 SET 的準位馬上換算
  speed_Target = 0.f;  // 結束後由速度環停在原地
  itest_half = half;
  itest_left = ms * foc_hz / 1000;
  itest_cnt = 0;
  itest_phase = 0;
  itest_end = 0;
  itest_mode = (uint8_t)mode;
  Cmd_Reply("[ITEST] START");
}

static uint16_t Log_RecSize(uint32_t mask)
{
  uint16_t size = 0;
  for (int i = 0; i < LOG_F_COUNT; i++) {
    if (mask & (1UL << i)) {
      size += log_field_size[i];
    }
  }
  return size;
}

// LOGSEL <mask>：選擇之後的擷取 / 串流要存哪些欄位
static void Log_Select(char *args)
{
  uint32_t mask;
  if (!Cmd_ParseUint(&args, &mask) || *args != '\0' || mask == 0 || (mask & ~LOG_MASK_ALL)) {
    Cmd_Reply("[LOG] ERR PARAM");
    return;
  }
  if (log_state != LOG_IDLE) {
    Cmd_Reply("[LOG] BUSY");
    return;
  }
  log_mask = mask;
  log_rec_size = Log_RecSize(mask);
  log_cap = LOG_BUF_BYTES / log_rec_size;
  char msg[64];
  snprintf(msg, sizeof(msg), "[LOG] SEL mask=0x%lX rec=%u n=%lu foc_hz=%lu", log_mask, log_rec_size, log_cap, foc_hz);
  Cmd_Reply(msg);
}

static void Log_Start(int div, uint8_t quiet, uint8_t mode)
{
  if (log_state != LOG_IDLE) {
    Cmd_Reply("[LOG] BUSY");
    return;
  }
  if (div < 1 || div > LOG_DIV_MAX) {
    Cmd_Reply("[LOG] ERR PARAM");
    return;
  }
  log_div = (uint16_t)div;
  log_quiet = quiet;
  log_mode = mode;
  log_idx = 0;
  log_div_cnt = 0;
  log_sum_iq_cmd = log_sum_iq = log_sum_id = log_sum_speed = log_sum_pos_err = log_sum_uq = log_sum_ud = 0.f;
  log_sum_id_cmd = 0.f;
  log_isr_max_us = 0;
  log_isr_sum_us = 0;
  log_isr_cnt = 0;
  swd_if.log_total = 0;
  Swd_Update();  // 主機看到 [LOG] START 時，swd_if 內的格式已經是這一次的
  Cmd_Reply("[LOG] START");
  log_state = (mode == LOG_MODE_STREAM) ? LOG_STREAM : LOG_CAPTURING;
}

// LOGX：停止串流、放掉 LOGC 留在緩衝區的資料，或中止進行中的擷取
static void Log_Stop(void)
{
  log_state = LOG_IDLE;
  Cmd_Reply("[LOG] STOP");
}

// 在 FOC 中斷結尾呼叫。欄位與比例同後代；這一代電流與電壓是定點數，累加與換算都用整數，浮點欄位只在存一筆時才換算
static inline int16_t Log_Q16(int32_t v, int32_t k)
{
  int32_t r = (int32_t)(((int64_t)v * k) >> 16);
  if (r > 32767) return 32767;
  if (r < -32768) return -32768;
  return (int16_t)r;
}

static inline int32_t Log_Sat16(int32_t v)
{
  if (v > 32767) return 32767;
  if (v < -32768) return -32768;
  return v;
}

static inline void Log_Sample(void)
{
  if (log_state != LOG_CAPTURING && log_state != LOG_STREAM) {
    return;
  }
  log_sum_iq_cmd += log_iq_cmd_i;
  log_sum_iq += iq_fix;
  log_sum_id += id_fix;
  log_sum_speed += log_speed_i;
  log_sum_pos_err += Log_Sat16(pos_err_count);
  log_sum_uq += vq_fix;
  log_sum_ud += vd_fix;
  log_sum_id_cmd += id_ref_fix;

  uint32_t isr_us = (uint32_t)(micros() - new_ang_time);
  if (isr_us > log_isr_max_us) {
    log_isr_max_us = isr_us;
  }
  log_isr_sum_us += isr_us;
  log_isr_cnt++;

  if (++log_div_cnt >= log_div) {
    int32_t n = log_div;
    uint32_t m = log_mask;
    uint8_t *p = log_buf + log_idx * log_rec_size;
    // 欄位大小都是 2 的倍數，但 4 byte 的欄位不一定落在 4 的倍數上，所以用 memcpy
#define LOG_PUT(field, type, value) \
    if (m & (1UL << (field))) { type v_ = (type)(value); memcpy(p, &v_, sizeof(v_)); p += sizeof(v_); }
    LOG_PUT(LOG_F_ENC, uint16_t, pos_enc_prev)
    LOG_PUT(LOG_F_T_US, uint16_t, new_ang_time)
    LOG_PUT(LOG_F_IQ_CMD, int16_t, Log_Sat16(log_sum_iq_cmd / n))
    LOG_PUT(LOG_F_IQ, int16_t, Log_Q16(log_sum_iq / n, log_k_i))
    LOG_PUT(LOG_F_ID, int16_t, Log_Q16(log_sum_id / n, log_k_i))
    LOG_PUT(LOG_F_SPEED, int16_t, Log_Sat16(log_sum_speed / n))
    LOG_PUT(LOG_F_POS_ERR, int16_t, Log_Sat16(log_sum_pos_err / n))
    LOG_PUT(LOG_F_I_CH1, int16_t, Log_Q16(cur1_fix, log_k_iph))
    LOG_PUT(LOG_F_I_CH2, int16_t, Log_Q16(cur2_fix, log_k_iph))
    LOG_PUT(LOG_F_I_CH3, int16_t, Log_Q16(-cur1_fix - cur2_fix, log_k_iph))
    LOG_PUT(LOG_F_UQ, int16_t, Log_Q16(log_sum_uq / n, log_k_u))
    LOG_PUT(LOG_F_UD, int16_t, Log_Q16(log_sum_ud / n, log_k_u))
    LOG_PUT(LOG_F_U_ALPHA, int16_t, Log_Q16(va_fix, log_k_u))
    LOG_PUT(LOG_F_U_BETA, int16_t, Log_Q16(vb_fix, log_k_u))
    LOG_PUT(LOG_F_ADC1, uint16_t, adc_foc1)
    LOG_PUT(LOG_F_ADC2, uint16_t, adc_foc2)
    LOG_PUT(LOG_F_SPEED_F, float, (float)log_sum_speed / ((float)n * LOG_SCALE_SPEED))
    LOG_PUT(LOG_F_SPEED_CMD, float, log_speed_cmd)
    LOG_PUT(LOG_F_POS, int32_t, pos_count)
    LOG_PUT(LOG_F_POS_TGT, int32_t, pos_target_count)
    LOG_PUT(LOG_F_ID_CMD, int16_t, Log_Q16(log_sum_id_cmd / n, log_k_i))
#undef LOG_PUT
    log_div_cnt = 0;
    log_sum_iq_cmd = log_sum_iq = log_sum_id = log_sum_speed = log_sum_pos_err = log_sum_uq = log_sum_ud = 0;
    log_sum_id_cmd = 0;
    if (++log_idx >= log_cap) {
      log_idx = 0;
      if (log_state == LOG_CAPTURING) {
        log_state = (log_mode == LOG_MODE_UART) ? LOG_READY : LOG_HOLD;
      }
    }
    swd_if.log_total++;  // 這一筆寫完之後才加，SWD 端只讀編號小於 log_total 的紀錄
  }
}

// LOG / LOGQ 存滿後送出：一行文字標頭 + log_cap 筆二進位資料 + 一行結尾 (含 byte 總和)。標頭格式同後代，另加 foc_hz
void Log_Process(void)
{
  if (log_state != LOG_READY) {
    return;
  }
  char msg[240];
  snprintf(msg, sizeof(msg),
    "[LOG] BEGIN n=%lu rec=%u div=%u quiet=%u speed_target=%.4f isr_max_us=%lu isr_avg_us=%.2f bias1=%.2f bias2=%.2f mode=%u pos_div=%u mask=0x%lX foc_hz=%lu",
    log_cap, log_rec_size, log_div, log_quiet, speed_Target,
    log_isr_max_us, (float)log_isr_sum_us / (float)log_isr_cnt,
    adc_bios_CH1, adc_bios_CH2, ctrl_mode, pos_div, log_mask, foc_hz);
  Cmd_Reply(msg);

  const uint8_t *p = (const uint8_t*)log_buf;
  uint32_t total = log_cap * log_rec_size;
  uint32_t sum = 0;
  for (uint32_t i = 0; i < total; i++) {
    sum += p[i];
  }
  for (uint32_t off = 0; off < total; off += 1024) {
    uint32_t len = (total - off < 1024) ? (total - off) : 1024;
    HAL_UART_Transmit(&huart1, (uint8_t*)(p + off), (uint16_t)len, 1000);
  }

  snprintf(msg, sizeof(msg), "\n[LOG] END sum=%lu", sum);
  Cmd_Reply(msg);
  log_state = LOG_IDLE;
}

void Cmd_Process(void)
{
  if (swd_cmd != SWD_CMD_NONE) {
    switch (swd_cmd) {
    case SWD_CMD_ALIGN:      Align_Start();   break;
    default:                 Cmd_Reply("[CMD] UNKNOWN"); break;
    }
    swd_cmd = SWD_CMD_NONE;
  }

  // SWD 指令信箱：內容與 UART 的一行指令相同
  if (swd_if.cmd_seq != swd_if.cmd_ack) {
    uint32_t seq = swd_if.cmd_seq;
    char line[CMD_LINE_MAX];
    memcpy(line, swd_if.cmd_line, CMD_LINE_MAX);
    line[CMD_LINE_MAX - 1] = '\0';
    Cmd_Exec(line);
    swd_if.cmd_ack = seq;
  }

  if (cmd_line_ready) {
    Cmd_Exec(cmd_line);
    cmd_line_ready = 0;
  }
}

// 指令與回應同後代 (doc/serial指令.html)；後代有、這一代還沒有的指令回 [CMD] UNKNOWN
static void Cmd_Exec(char *cmd_line)
{
  // 不分大小寫
  for (char *p = cmd_line; *p; p++) {
    if (*p >= 'a' && *p <= 'z') {
      *p -= 'a' - 'A';
    }
  }

  if (strcmp(cmd_line, "STOP") == 0) {
    Out_Stop(OUT_COAST);
  } else if (strcmp(cmd_line, "BRAKE") == 0) {
    Out_Stop(OUT_BRAKE);
  } else if (strcmp(cmd_line, "RUN") == 0) {
    Out_Run();
  } else if (strncmp(cmd_line, "LOGQ ", 5) == 0) {
    Log_Start(atoi(cmd_line + 5), 1, LOG_MODE_UART);
  } else if (strncmp(cmd_line, "LOG ", 4) == 0) {
    Log_Start(atoi(cmd_line + 4), 0, LOG_MODE_UART);
  } else if (strncmp(cmd_line, "LOGC ", 5) == 0) {
    Log_Start(atoi(cmd_line + 5), 0, LOG_MODE_SWD);
  } else if (strncmp(cmd_line, "LOGS ", 5) == 0) {
    Log_Start(atoi(cmd_line + 5), 0, LOG_MODE_STREAM);
  } else if (strncmp(cmd_line, "LOGSEL ", 7) == 0) {
    Log_Select(cmd_line + 7);
  } else if (strcmp(cmd_line, "LOGX") == 0) {
    Log_Stop();
  } else if (strcmp(cmd_line, "I2C_SCAN") == 0) {
    I2c_Scan();
  } else if (strcmp(cmd_line, "ALIGN") == 0) {
    Align_Start();
  } else if (strncmp(cmd_line, "ITEST ", 6) == 0) {
    Itest_Cmd(cmd_line + 6);
  } else if (strcmp(cmd_line, "POSZERO") == 0) {
    Pos_CmdZero();
  } else if (strncmp(cmd_line, "SET ", 4) == 0) {
    Param_Set(cmd_line + 4);
  } else if (strncmp(cmd_line, "GET ", 4) == 0) {
    Param_Get(cmd_line + 4);
  } else {
    Cmd_Reply("[CMD] UNKNOWN");
  }
}

// 主迴圈：回報 FOC 中斷關掉輸出的原因、AS5600 卡住時重新初始化 I2C2
static void Fault_Process(void)
{
  if (ang_fault) {
    ang_fault = 0;
    Cmd_Reply("[ANG] ERR TIMEOUT");
    Cmd_Reply("[OUT] COAST");
  }
  uint32_t seq = as5600.seq;
  uint32_t now = HAL_GetTick();
  if (seq != ang_stall_seq) {
    ang_stall_seq = seq;
    ang_stall_tick = now;
  } else if (now - ang_stall_tick > ANG_STALL_MS) {
    ang_stall_tick = now;
    AS5600_Recover(&as5600);
    Cmd_Reply(AS5600_Resume(&as5600, (uint32_t)micros()) == AS5600_OK ? "[ANG] I2C RESET" : "[ANG] ERR I2C");
  }
}

// FOC：TIM1 每個 PWM 週期在計數頂點附近 (下橋導通) 觸發一次 ADC1 injected 轉換，
// ADC1_2_IRQHandler 在 JEOC 時呼叫這裡 (不經過 HAL_ADC_IRQHandler：F1 的 HAL 在這種設定下轉完一次就關掉 JEOC 中斷)。
// 控制架構與後代相同 (速度 PI → Iq PI，位置 P → 速度 PI)。F103 沒有 FPU，浮點運算一次要 50 ~ 100 個週期，所以：
//   - 電流環 (Clarke、Park、Iq/Id PI、反 Park、SVPWM) 以定點數每次都算 (20 kHz)
//   - 速度估測、速度環、位置環 / MIT 以浮點、每 VEL_DIV 次中斷算一輪 (5 kHz)，分散在四個時槽
static void Foc_Step(uint16_t a1, uint16_t a2);

void Foc_AdcIsr(void)
{
  uint32_t c0 = DWT->CYCCNT;
  uint16_t a1 = (uint16_t)ADC1->JDR1;  // CH1 (U, PB1)
  uint16_t a2 = (uint16_t)ADC1->JDR2;  // CH2 (V, PB0)
  adc_raw1 = a1;
  adc_raw2 = a2;
  AS5600_Kick(&as5600);   // 背景讀下一筆角度

  if (++foc_div_cnt < FOC_DIV) {
    return;
  }
  foc_div_cnt = 0;
  Foc_Step(a1, a2);
  uint32_t cyc = DWT->CYCCNT - c0;
  foc_isr_cyc = cyc;
  if (cyc > foc_isr_cyc_max) {
    foc_isr_cyc_max = cyc;
  }
}

// ---- 速度環 / 位置環 (浮點)：每 VEL_DIV (4) 次 FOC 中斷為一輪，計算分散在四個時槽，每次中斷不超過 50 µs ----
//   時槽 0：速度估測            時槽 1：速度目標 (串級時是位置環輸出) 與目標權重
//   時槽 2：速度 PI 或 MIT，更新 Iq 命令   時槽 3：位置 P (串級，每 pos_div 次 FOC 中斷一次)
static float vel_cmd_now;   // 時槽 1 算出、時槽 2 使用的速度目標

static void Foc_Slot0(void)
{
  // 速度 = 這一輪累計的角度變化 ÷ 時間，再過一階低通 (固定以速度環的週期更新)
  if (spd_dt_sum > 0) {
    spd_meas = (float)spd_d_sum * (1e6f / ENC_CPR) / (float)spd_dt_sum;
    spd_d_sum = 0;
    spd_dt_sum = 0;
  }
  float alpha = spd_alpha;
  ang_speed = alpha * ang_speed + (1.0f - alpha) * spd_meas;
  speed_q16 = (int32_t)(ang_speed * ((float)ENC_CPR * 65536.f / 1e6f));
  log_speed_i = (int32_t)(ang_speed * LOG_SCALE_SPEED);
}

static void Foc_Slot1(uint8_t run)
{
  uint8_t pos = (ctrl_mode == CTRL_MODE_POS);
  if (!run || (pos && pos_type == POS_TYPE_MIT) || ctrl_mode == CTRL_MODE_TRQ) {
    vel_cmd_lp = vel_cmd_now = 0.f;
    log_speed_cmd = 0.f;
    return;
  }
  float speed_cmd = pos ? pos_speed_cmd : speed_Target;
  log_speed_cmd = speed_cmd;
  // 目標權重 (速度模式)：等效於比例項只看 vel_b 倍的目標，但寫成目標的前置濾波，穩態時積分不用多扛一份
  float vel_b_now = vel_b;
  if (!pos && vel_b_now < 1.f && PID__velocity.I > 0.f) {
    vel_cmd_lp += vel_lp_k * (speed_cmd - vel_cmd_lp);
    speed_cmd = vel_b_now * speed_cmd + (1.f - vel_b_now) * vel_cmd_lp;
  } else {
    vel_cmd_lp = speed_cmd;
  }
  vel_cmd_now = speed_cmd;
}

static void Foc_Slot2(uint8_t run)
{
  if (!run) {
    uq_Target = 0.f;
  } else if (itest_mode != ITEST_OFF) {
    return;  // 電流環測試取代速度環，Iq 命令由 Itest_Process() 給
  } else if (ctrl_mode == CTRL_MODE_TRQ) {
    // 力矩模式：IQREF 直接當 Iq 命令，限幅同速度環的輸出 (VEL.LIMIT)
    float limit = PID__velocity.limit;
    uq_Target = constrain(trq_iq, -limit, limit);
  } else if (ctrl_mode == CTRL_MODE_POS && pos_type == POS_TYPE_MIT) {
    // MIT：彈簧 (MIT.KP，A/度) + 阻尼 (MIT.KD，A/(圈/秒)) 直接算 Iq，限幅同速度環的輸出 (VEL.LIMIT)
    float limit = PID__velocity.limit;
    uq_Target = constrain(mit_kp * (float)pos_err_count * POS_DEG_PER_COUNT - mit_kd * ang_speed, -limit, limit);
  } else {
    uq_Target = PIDController_runTs(&PID__velocity, vel_cmd_now - ang_speed, foc_dt * VEL_DIV,
                                    foc_inv_dt * (1.f / VEL_DIV), vel_aw);
#if EXF_MAP_ENABLE
    if (ctrl_mode == CTRL_MODE_VEL && fabsf(speed_Target) <= 0.5f) {
      uq_Target += exf_map[((uint32_t)as5600.raw * PRE_F_LEN) / AS5600_CPR];
    }
#endif
  }
  iq_ref_fix = (int32_t)(uq_Target * (1.f / I_UNIT_A));
  log_iq_cmd_i = (int32_t)(uq_Target * LOG_SCALE_I);
}

static void Foc_Slot3(uint8_t run)
{
  if (!run || ctrl_mode != CTRL_MODE_POS || pos_type != POS_TYPE_CASCADE) {
    pos_div_cnt = 0;
    return;
  }
  // 位置環每 pos_div 次 FOC 中斷算一次 (以 VEL_DIV 為單位進位)
  pos_div_cnt += VEL_DIV;
  if (pos_div_cnt >= pos_div) {
    pos_div_cnt = 0;
    float pos_limit = PID__position.limit;
    pos_speed_cmd = constrain(PIDController_runTs(&PID__position, (float)pos_err_count * POS_DEG_PER_COUNT, pos_ts, pos_inv_ts, 0),
                              -pos_limit, pos_limit);
  }
}

// 電流環測試的準位，在 FOC 中斷內取代速度環 (同後代，定點數)。回傳這一次中斷的測試種類；
// 結束時 itest_mode 變回 ITEST_OFF，速度環從靜止接手
static inline uint8_t Itest_Process(void)
{
  uint8_t mode = itest_mode;
  if (mode == ITEST_OFF) {
    return ITEST_OFF;
  }
  uint8_t end = 0;
  int32_t trip = itest_i_trip_fix;
  int32_t spd_trip = itest_spd_trip_q16;
  if (id_fix > trip || id_fix < -trip || iq_fix > trip || iq_fix < -trip) {
    end = ITEST_END_CURRENT;
  } else if (speed_q16 > spd_trip || speed_q16 < -spd_trip) {
    end = ITEST_END_SPEED;
  } else if (itest_left == 0) {
    end = itest_end ? itest_end : ITEST_END_TIME;
  }
  if (end) {
    itest_end = end;
    itest_mode = ITEST_OFF;
    itest_q = itest_d = 0;
    uq_Target = 0.f;
    iq_ref_fix = id_ref_fix = 0;
    log_iq_cmd_i = 0;
    PIDController_reset(&PID__velocity);
    PiFix_Reset(&pi_iq);
    PiFix_Reset(&pi_id);
    return ITEST_OFF;
  }
  itest_left--;
  const volatile int32_t *lv = it_fix[mode == ITEST_CUR ? 0 : 1];
  itest_q = itest_phase ? lv[1] : lv[0];
  itest_d = itest_phase ? lv[3] : lv[2];
  if (++itest_cnt >= itest_half) {
    itest_cnt = 0;
    itest_phase ^= 1;
  }
  if (mode == ITEST_CUR) {
    iq_ref_fix = itest_q;
    id_ref_fix = itest_d;
  } else {
    iq_ref_fix = id_ref_fix = 0;
  }
  log_iq_cmd_i = (int32_t)(((int64_t)iq_ref_fix * log_k_i) >> 16);
  return mode;
}

// (a × b) >> 15，a 是定點數、b 是 Q15
static inline int32_t Mul_Q15(int32_t a, int32_t b)
{
  return (int32_t)(((int64_t)a * b) >> 15);
}

// 一階低通 (定點)：y 是電流單位 × 4096，k 是 Q16
static inline int32_t Lpf_Fix(int32_t *y, int32_t x, int32_t k)
{
  *y += (int32_t)(((int64_t)k * ((x << 12) - *y)) >> 16);
  return *y >> 12;
}

static void Foc_Step(uint16_t a1, uint16_t a2)
{
  uint8_t run = (FocDo != 0);
  // FocDo = 0 而且不是 STOP / BRAKE (開機、對齊中)：什麼都不做。
  // STOP / BRAKE 之後繼續量測角度、速度與電流但不輸出，多圈位置也不會中斷
  if (!run && out_state == OUT_RUN) {
    return;
  }
  new_ang_time = micros();
  uint32_t now32 = (uint32_t)new_ang_time;

  // ---- 角度：AS5600 有新的一筆時更新多圈位置，並累計給速度估測 ----
  uint32_t seq, t_s;
  uint16_t raw;
  do {
    seq = as5600.seq;
    raw = as5600.raw;
    t_s = as5600.t_us;
  } while (seq != as5600.seq);  // 讀的途中被 I2C 中斷改掉了就重讀
  if (seq != ang_seq_prev) {
    ang_seq_prev = seq;
#if ENC_DIR > 0
    uint16_t enc_now = (uint16_t)(((raw - zero_raw) & (AS5600_CPR - 1)) << ENC_SHIFT);
#else
    uint16_t enc_now = (uint16_t)(((zero_raw - raw) & (AS5600_CPR - 1)) << ENC_SHIFT);
#endif
    if (foc_reseed) {
      // 開機或 ALIGN 剛換過零點：多圈位置從單圈角度重新開始，位置目標設為目前位置
      foc_reseed = 0;
      pos_count = enc_now;
      pos_target_count = pos_count;
      pos_speed_cmd = 0.f;
      ang_speed = spd_meas = 0.f;
      speed_q16 = 0;
      spd_d_sum = 0;
      spd_dt_sum = 0;
    } else {
      // 一圈剛好 65536 count，相鄰兩筆的差以 int16 解讀就是有號的位移
      int16_t d = (int16_t)(enc_now - pos_enc_prev);
      pos_count += d;
      spd_d_sum += d;
      spd_dt_sum += t_s - ang_t_prev;
    }
    pos_enc_prev = enc_now;
    ang_t_prev = t_s;
  }
  // 有號數：I2C 中斷 (優先權較高) 可能在這個中斷開始之後才存入角度，時間比 now32 晚
  int32_t age_us = (int32_t)(now32 - ang_t_prev);
  if (age_us < 0) {
    age_us = 0;
  } else if (age_us > ANG_TIMEOUT_US) {
    if (run) {
      // 角度太久沒更新：輸出關閉 (滑行)，要 RUN 才恢復
      FocDo = 0;
      Pwm_Off();
      out_state = OUT_COAST;
      ang_fault = 1;
      run = 0;
    }
    age_us = ANG_EXTRAP_MAX_US;
  } else if (age_us > ANG_EXTRAP_MAX_US) {
    age_us = ANG_EXTRAP_MAX_US;
  }
  // AS5600 一筆要讀 70 µs 以上，電角度以目前速度外插到現在 (count)
  int32_t enc_ext = (int32_t)pos_enc_prev + ((speed_q16 * age_us) >> 16);
  // 電角度 (count) = 機械角 × 極對數，取 16 bit 就是 0 ~ 2π。sin / cos 是 Q15
  uint16_t elec = (uint16_t)(enc_ext * POLE_PAIRS);
  uint32_t li = elec >> 8;
  int32_t lf = elec & 0xFF;
  int32_t s0 = sin_lut[li];
  int32_t sn = s0 + (((sin_lut[li + 1] - s0) * lf) >> 8);
  uint32_t lj = (li + SIN_LUT_N / 4) & (SIN_LUT_N - 1);
  int32_t c0 = sin_lut[lj];
  int32_t cs = c0 + (((sin_lut[lj + 1] - c0) * lf) >> 8);

  // 位置誤差：以無號數相減，pos_count 回繞時誤差仍然正確；速度模式時為 0
  pos_err_count = (run && ctrl_mode == CTRL_MODE_POS) ? (int32_t)((uint32_t)pos_target_count - (uint32_t)pos_count) : 0;

  // ---- 速度環 / 位置環 (浮點)，分散在不同的中斷 ----
  uint8_t slot = foc_slot;
  foc_slot = (slot + 1 < VEL_DIV) ? slot + 1 : 0;
  switch (slot) {
  case 0: Foc_Slot0(); break;
  case 1: Foc_Slot1(run); break;
  case 2: Foc_Slot2(run); break;
  default: Foc_Slot3(run); break;
  }

  // ---- 電流 (定點)：1 V/A，1 單位 = ADC 1 LSB / 16 ----
  adc_foc1 = a1;
  adc_foc2 = a2;
  int32_t i1 = ((int32_t)a1 << CUR_SHIFT) - bias1_fix;
  int32_t i2 = ((int32_t)a2 << CUR_SHIFT) - bias2_fix;
  if (iph_lpf) {
    i1 = Lpf_Fix(&iph_lp1, i1, iph_k);
    i2 = Lpf_Fix(&iph_lp2, i2, iph_k);
  } else {
    iph_lp1 = i1 << 12;
    iph_lp2 = i2 << 12;
  }
  cur1_fix = i1;
  cur2_fix = i2;

  // Clarke：α = CH1、β = (2·CH2 + CH1) / √3 (1/√3 = 18919 / 32768)；Park
  int32_t ia = i1;
  int32_t ib = Mul_Q15(2 * i2 + i1, 18919);
#if PHASE_SWAP
  ib = -ib;
#endif
  int32_t id = (int32_t)(((int64_t)ia * cs + (int64_t)ib * sn) >> 15);
  int32_t iq = (int32_t)(((int64_t)ib * cs - (int64_t)ia * sn) >> 15);
  if (idq_lpf) {
    id = Lpf_Fix(&idq_lp_d, id, idq_k);
    iq = Lpf_Fix(&idq_lp_q, iq, idq_k);
  } else {
    idq_lp_d = id << 12;
    idq_lp_q = iq << 12;
  }
  id_fix = id;
  iq_fix = iq;

  if (!run) {
    if (itest_mode != ITEST_OFF) {
      // 角度逾時關閉了輸出：測試結束
      itest_mode = ITEST_OFF;
      itest_end = ITEST_END_STOP;
      iq_ref_fix = id_ref_fix = 0;
    }
    vd_fix = vq_fix = va_fix = vb_fix = 0;
    iq_pi_on = id_pi_on = 0;
    Log_Sample();
    return;
  }
  uint8_t itest = Itest_Process();
  int32_t vq, vd;
  if (itest == ITEST_VOLT) {
    // 開迴路：不經過 PI
    vq = itest_q;
    vd = itest_d;
    iq_pi_on = id_pi_on = 0;
  } else {
    uint8_t id_on = (id_loop || itest == ITEST_CUR);
    // 上一次沒有執行該 PI (剛 RUN、剛打開 Id 環)：從 0 開始積分
    if (!iq_pi_on) {
      PiFix_Reset(&pi_iq);
    }
    if (id_on && !id_pi_on) {
      PiFix_Reset(&pi_id);
    }
    iq_pi_on = 1;
    id_pi_on = id_on;
    vq = PiFix_Run(&pi_iq, iq_ref_fix - iq);
    vd = id_on ? PiFix_Run(&pi_id, id_ref_fix - id) : 0;
  }
  vq_fix = vq;
  vd_fix = vd;
  // 反 Park
  int32_t va = (int32_t)(((int64_t)vd * cs - (int64_t)vq * sn) >> 15);
  int32_t vb = (int32_t)(((int64_t)vd * sn + (int64_t)vq * cs) >> 15);
  va_fix = va;
  vb_fix = vb;
  // 電壓單位是比較值 / 16；β 乘 √3/2 (28378 / 32768) 給 SVPWM
#if PHASE_SWAP
  Svpwm_Counts(va >> V_SHIFT, Mul_Q15(-vb, 28378) >> V_SHIFT);
#else
  Svpwm_Counts(va >> V_SHIFT, Mul_Q15(vb, 28378) >> V_SHIFT);
#endif
  Log_Sample();
}

// CAN：沿用舊程式的處理方式 (doc/設計文件.html 第 10 節)
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
        memcpy(&speed_Target, &RxData[2], sizeof(float));
        deviceMode = DeviceMode_SpeedMode;
      }
    }
    // 平衡車的力矩命令：byte2~5 float = Iq (A)。不在力矩模式或輸出關閉時，由主迴圈切到 MODE 5 並 RUN
    else if (RxData[0] == SET_IQ && RxData[1] == Set_TargetVale && RxHeader.DLC >= 6) {
      float iq;
      memcpy(&iq, &RxData[2], sizeof(float));
      if (isfinite(iq)) {
        trq_iq = iq;
        can_trq_tick = HAL_GetTick();
        can_trq_live = 1;
        if (ctrl_mode != CTRL_MODE_TRQ || out_state != OUT_RUN) {
          can_trq_req = 1;
        }
      }
    }
}

// 主迴圈呼叫：CAN 力矩命令要求的模式切換與 RUN (中斷內不做)
static void Can_Process(void)
{
  if (!can_trq_req) {
    return;
  }
  can_trq_req = 0;
  if (ctrl_mode != CTRL_MODE_TRQ) {
    if (!Mode_Set(CTRL_MODE_TRQ)) {
      return;  // 電流環測試中，不切換
    }
    Cmd_Reply("[PARAM] MODE=5");
  }
  if (out_state != OUT_RUN) {
    Out_Run();
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
  // 先關掉輸出：PWM_ACTIVE 拉 Low (三相高阻)、三相比較值 0、關閉 TIM1 主輸出。不呼叫 HAL，時脈還沒開時寫入也無害
  PWM_ACTIVE_GPIO_Port->BSRR = (uint32_t)PWM_ACTIVE_Pin << 16;
  TIM1->CCR1 = 0;
  TIM1->CCR2 = 0;
  TIM1->CCR3 = 0;
  TIM1->BDTR &= ~TIM_BDTR_MOE;
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
