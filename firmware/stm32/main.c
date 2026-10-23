/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <math.h>
#include "estop.h"
#include "elevator_rf.h"
#include "nrf24.h"
#include "FreeRTOS.h"
#include "task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  int16_t acc_x;
  int16_t acc_y;
  int16_t acc_z;

  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;

  uint16_t seq;
} ImuRawData_t;

/* 0x210 as received from CAN; dlc 0 marks a remote frame. */
typedef struct
{
  uint8_t dlc;
  uint8_t data[8];
} ElevCmdMsg_t;

/* 0x310/0x311 waiting for CAN_Task; stamp enforces the 20 ms freshness rule. */
typedef struct
{
  uint16_t id;
  uint8_t data[8];
  uint32_t stamp_ms;
} ElevCanTx_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CAN_ID_JETSON_DRIVE_CMD     0x201U
#define CAN_ID_ESTOP_STATUS          0x202U
#define ESTOP_CAN_PERIOD_MS          100U
#define ODRV_CMD_ESTOP               0x002U
#define JETSON_CMD_TIMEOUT_MS       300U

/* Elevator RF relay (ELEVATOR_RF_PROTOCOL.md, ELEVATOR_CAN_PROTOCOL.md) */
#define RF_FLAG_IRQ                 0x01U
#define RF_FLAG_CMD                 0x02U
#define RF_TICK_MS                  10U
#define RF_TX_TIMEOUT_MS            20U    /* ARC 10 x ARD 500 us ~ 8.5 ms */
#define RF_CHECK_PERIOD_MS          1000U
#define ELEV_CAN_FRESH_MS           20U
#define ELEV_SCAN_FIRST_CH          70U
#define ELEV_SCAN_CHANNELS          14U    /* 70..83 */
#define ELEV_SCAN_SAMPLES           100U
#define ELEV_SCAN_CHUNK             10U    /* ~2 ms busy per chunk, then yield */

/* Independent watchdog: LSI 32 kHz / 64 x 1000 = ~2 s (LSI tolerance 17-47 kHz -> 1.4-3.8 s).
 * defaultTask kicks it only while CAN_Task, IMU_Task and RF_Task keep beating. */
#define WDG_IWDG_PR_DIV64           4U
#define WDG_IWDG_RELOAD             999U
#define WDG_TASK_STALL_MS           1000U  /* > longest legitimate block (4 x 100 ms CAN TX wait) */
#define WDG_TASKS                   3U
#define SYS_STATS_PERIOD_MS         1000U

/*
 * Differential-drive calibration parameters
 * - Node 0: right wheel, positive command = forward direction
 * - Node 1: left wheel, physical direction is reversed, so direction = -1
 * - Wheel outer diameter: 172 mm -> 0.172 m
 * - Track width: 529 mm -> 0.529 m
 *
 * Speed limiting is performed by Jetson control_mux before sending (v, w).
 * ODrive Input_Vel unit is rev/s (= turn/s).
 * Final command:
 *   right_rev_s = (v + w * track_width / 2) / (pi * wheel_diameter)
 *   left_rev_s  = (v - w * track_width / 2) / (pi * wheel_diameter)
 */
#define DRIVE_PI                         3.1415926f
#define DRIVE_WHEEL_DIAMETER_DEFAULT_M   0.172f
#define DRIVE_TRACK_WIDTH_DEFAULT_M       0.529f

#define DRIVE_NODE0_DIRECTION_DEFAULT     1.0f
#define DRIVE_NODE1_DIRECTION_DEFAULT    -1.0f


#define TEST_STOP_REV_S                   0.0f

#define CAN_ID_IMU_ACC_RAW          0x301U
#define CAN_ID_IMU_GYRO_RAW         0x302U

#define LSM6DSV16X_WHO_AM_I_REG     0x0FU
#define LSM6DSV16X_WHO_AM_I_VAL     0x70U

#define LSM6DSV16X_CTRL1            0x10U
#define LSM6DSV16X_CTRL2            0x11U
#define LSM6DSV16X_CTRL3            0x12U

#define LSM6DSV16X_OUTX_L_G         0x22U
#define LSM6DSV16X_OUTX_L_A         0x28U

#define IMU_ODR_30HZ_HP             0x04U
#define IMU_SAMPLE_PERIOD_MS        100U
#define IMU_WHOAMI_CHECK_PERIOD_MS  1000U

#define ODRV_NODE_ID_LEFT           0U
#define ODRV_NODE_ID_RIGHT          1U

#define ODRV_CMD_HEARTBEAT          0x001U
#define ODRV_CMD_SET_AXIS_STATE       0x007U
#define ODRV_CMD_GET_ENCODER_ESTIMATES 0x009U
#define ODRV_CMD_SET_CONTROLLER_MODE  0x00BU
#define ODRV_CMD_SET_INPUT_VEL        0x00DU
#define ODRV_CMD_GET_BUS_VOLTAGE_CURRENT 0x017U
#define ODRV_CMD_CLEAR_ERRORS         0x018U

#define ODRV_CAN_ID(node_id, cmd_id)  ((((uint32_t)(node_id)) << 5) | ((uint32_t)(cmd_id)))
#define ODRV_NODE_FROM_CAN_ID(std_id) ((uint8_t)(((std_id) >> 5) & 0x3FU))
#define ODRV_CMD_FROM_CAN_ID(std_id)  ((uint32_t)((std_id) & 0x1FU))

#define ODRV0_ID_HEARTBEAT           ODRV_CAN_ID(ODRV_NODE_ID_LEFT,  ODRV_CMD_HEARTBEAT)
#define ODRV0_ID_ENCODER_ESTIMATES   ODRV_CAN_ID(ODRV_NODE_ID_LEFT,  ODRV_CMD_GET_ENCODER_ESTIMATES)
#define ODRV0_ID_BUS_VOLTAGE_CURRENT ODRV_CAN_ID(ODRV_NODE_ID_LEFT,  ODRV_CMD_GET_BUS_VOLTAGE_CURRENT)

#define ODRV1_ID_HEARTBEAT           ODRV_CAN_ID(ODRV_NODE_ID_RIGHT, ODRV_CMD_HEARTBEAT)
#define ODRV1_ID_ENCODER_ESTIMATES   ODRV_CAN_ID(ODRV_NODE_ID_RIGHT, ODRV_CMD_GET_ENCODER_ESTIMATES)
#define ODRV1_ID_BUS_VOLTAGE_CURRENT ODRV_CAN_ID(ODRV_NODE_ID_RIGHT, ODRV_CMD_GET_BUS_VOLTAGE_CURRENT)

#define ODRV_HEARTBEAT_TIMEOUT_MS   500U

#define ODRV_CONTROL_MODE_VELOCITY    2U
#define ODRV_INPUT_MODE_VEL_RAMP      2U

#define ODRV_AXIS_STATE_IDLE          1U
#define ODRV_AXIS_STATE_CLOSED_LOOP   8U
#define ODRV_AXIS_ERROR_WATCHDOG_TIMER_EXPIRED 0x01000000U

#define ODRV_VEL_CMD_PERIOD_MS        50U
#define ODRV_FEEDBACK_REQ_PERIOD_MS   100U
#define ODRV_ENABLE_FEEDBACK_REQUESTS 0U

#define CAN_CRITICAL_TX_TIMEOUT_MS    100U
#define CAN_BUSOFF_RECOVERY_PERIOD_MS 200U
#define ODRV_FAULT_RETRY_DELAY_MS     1000U

#define CAN_ESR_BOFF_FLAG             0x00000004U
#define CAN_NOTIFICATION_MASK         (CAN_IT_RX_FIFO0_MSG_PENDING | \
                                       CAN_IT_ERROR_WARNING | \
                                       CAN_IT_ERROR_PASSIVE | \
                                       CAN_IT_BUSOFF | \
                                       CAN_IT_LAST_ERROR_CODE | \
                                       CAN_IT_ERROR)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CAN_Task */
osThreadId_t CAN_TaskHandle;
const osThreadAttr_t CAN_Task_attributes = {
  .name = "CAN_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for IMU_Task */
osThreadId_t IMU_TaskHandle;
const osThreadAttr_t IMU_Task_attributes = {
  .name = "IMU_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for imuRawQueue */
osMessageQueueId_t imuRawQueueHandle;
const osMessageQueueAttr_t imuRawQueue_attributes = {
  .name = "imuRawQueue"
};
/* USER CODE BEGIN PV */
/* IMU status: kept minimal for Live Expressions and fault checking */
volatile uint8_t imu_whoami = 0;
volatile uint8_t imu_spi_status = 0;
volatile uint8_t imu_init_done = 0;
volatile uint32_t imu_raw_read_count = 0;
volatile uint32_t can_imu_tx_error_count = 0;

/* ODrive node 0 status and control variables */
volatile uint32_t odrv0_hb_count = 0;
volatile uint32_t odrv0_last_rx_tick = 0;
volatile uint32_t odrv0_axis_error = 0;
volatile uint8_t  odrv0_axis_state = 0;
volatile uint8_t  odrv0_hb_timeout = 1;

volatile float odrv0_target_vel = 0.0f;
volatile float odrv0_torque_ff = 0.0f;
volatile float odrv0_pos_estimate = 0.0f;
volatile float odrv0_vel_estimate = 0.0f;
volatile float odrv0_bus_voltage = 0.0f;
volatile float odrv0_bus_current = 0.0f;

/* ODrive node 1 status and control variables */
volatile uint32_t odrv1_hb_count = 0;
volatile uint32_t odrv1_last_rx_tick = 0;
volatile uint32_t odrv1_axis_error = 0;
volatile uint8_t  odrv1_axis_state = 0;
volatile uint8_t  odrv1_hb_timeout = 1;

volatile float odrv1_target_vel = 0.0f;
volatile float odrv1_torque_ff = 0.0f;
volatile float odrv1_pos_estimate = 0.0f;
volatile float odrv1_vel_estimate = 0.0f;
volatile float odrv1_bus_voltage = 0.0f;
volatile float odrv1_bus_current = 0.0f;

/* Shared ODrive control state */
volatile uint8_t  odrv_control_step = 0;
volatile uint32_t odrv_step_tick = 0;
volatile uint32_t odrv_last_vel_cmd_tick = 0;

volatile uint8_t  odrv_both_ready = 0;
volatile uint32_t odrv_tx_error_count = 0;
volatile uint32_t odrv_req_error_count = 0;
volatile uint32_t odrv_safe_stop_count = 0;
volatile uint32_t odrv0_vel_ok_count = 0;
volatile uint32_t odrv0_vel_fail_count = 0;
volatile uint32_t odrv1_vel_ok_count = 0;
volatile uint32_t odrv1_vel_fail_count = 0;

/* CAN diagnostics and recovery variables for Live Expressions */
volatile uint32_t can_esr = 0;
volatile uint32_t can_tsr = 0;
volatile uint32_t can_msr = 0;
volatile uint32_t can_rf0r = 0;
volatile uint32_t can_hal_error = 0;
volatile uint8_t  can_tec = 0;
volatile uint8_t  can_rec = 0;
volatile uint8_t  can_busoff = 0;
volatile uint8_t  can_busoff_latched = 0;
volatile uint32_t can_busoff_count = 0;
volatile uint32_t can_recover_count = 0;
volatile uint32_t can_recover_error_count = 0;
volatile uint32_t can_tx_timeout_count = 0;
volatile uint32_t can_tx_add_error_count = 0;
volatile uint32_t can_last_error_tick = 0;
volatile uint32_t can_last_recover_tick = 0;

/* Jetson drive command status */
volatile uint32_t jetson_drive_rx_count = 0;
volatile uint32_t jetson_drive_last_tick = 0;
volatile float jetson_cmd_v_mps = 0.0f;
volatile float jetson_cmd_w_radps = 0.0f;

volatile uint8_t jetson_cmd_timeout = 1;
volatile uint32_t jetson_cmd_timeout_count = 0;

/*
 * Geometry and motor direction for Live Expressions.
 * Geometry must match the upper control_mux wheel limiter.
 * Motor directions are signs (+1 or -1), not speed compensation gains.
 */
volatile float drive_wheel_diameter_m = DRIVE_WHEEL_DIAMETER_DEFAULT_M;
volatile float drive_track_width_m = DRIVE_TRACK_WIDTH_DEFAULT_M;

volatile float drive_node0_direction = DRIVE_NODE0_DIRECTION_DEFAULT;
volatile float drive_node1_direction = DRIVE_NODE1_DIRECTION_DEFAULT;


/* Inverse kinematics debug values for Live Expressions */
volatile float ik_right_wheel_mps = 0.0f;
volatile float ik_left_wheel_mps = 0.0f;
volatile float ik_right_rev_s_raw = 0.0f;
volatile float ik_left_rev_s_raw = 0.0f;
volatile float ik_odrv0_cmd_rev_s = 0.0f;
volatile float ik_odrv1_cmd_rev_s = 0.0f;
/* Diagnostics only: this firmware intentionally suspends CAN and IMU tasks. */
volatile uint32_t estop_input_test_mode = 0U;
volatile uint32_t estop_can_inhibit = 0U;
volatile uint32_t estop_can_stop_pending = 0U;
volatile uint32_t estop_can_trip_count = 0U;
volatile uint32_t estop_can_trip_ms = 0U;
volatile uint32_t estop_can_stop_queued = 0U;
volatile uint32_t estop_can_status_queued = 0U;
volatile uint32_t estop_can_tx_ok = 0U;
volatile uint32_t estop_can_tx_failed = 0U;
volatile uint32_t estop_can_tx_timeout = 0U;
volatile uint32_t estop_can_status_flags = 0U;
volatile uint32_t estop_can_idle_observed = 0U;
volatile uint32_t estop_drive_enabled = 0U;
volatile uint32_t estop_drive_rejected = 0U;
volatile uint32_t estop_can_task_suspended = 0U;
volatile uint32_t estop_imu_task_suspended = 0U;
/* 1 while E-stop frames are in flight or a burst is due: other senders must wait. */
volatile uint32_t estop_can_mailbox_hold = 0U;

/* Elevator RF relay: RF_Task owns SPI2 and the nRF24. */
osThreadId_t RF_TaskHandle;
const osThreadAttr_t RF_Task_attributes = {
  .name = "RF_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
osMessageQueueId_t elevCmdQueueHandle;
osMessageQueueId_t elevCanTxQueueHandle;
volatile uint32_t elev_radio_ok = 0U;
volatile uint32_t elev_radio_init_count = 0U;
volatile uint32_t elev_radio_lost_count = 0U;
volatile uint32_t elev_rx_overflow_count = 0U;
volatile uint32_t elev_tx_timeout_count = 0U;
volatile uint32_t elev_cmd_queue_full = 0U;
volatile uint32_t elev_can_tx_queued = 0U;
volatile uint32_t elev_can_tx_dropped = 0U;
volatile uint32_t elev_can_tx_sent = 0U;
volatile uint32_t elev_can_tx_failed = 0U;
volatile uint32_t elev_can_tx_stale = 0U;
volatile uint32_t elev_lat_last_us = 0U;
volatile uint32_t elev_lat_max_us = 0U;
volatile uint32_t elev_heap_free = 0U;
volatile uint32_t elev_rf_task_loops = 0U;
/* Debugger injection (bench tests without CAN): fill elev_debug_cmd, then set the request. */
volatile uint32_t elev_debug_cmd_request = 0U;
volatile uint32_t elev_debug_cmd_count = 0U;
volatile uint32_t elev_debug_scan_request = 0U;
volatile uint32_t elev_scan_done = 0U;
volatile uint8_t elev_debug_cmd[8] __attribute__((aligned(4)));
volatile uint32_t elev_scan_hits[ELEV_SCAN_CHANNELS];
static volatile uint32_t rf_irq_cycles;

/* Watchdog and resource diagnostics (read with --prefix sys_). */
volatile uint32_t sys_reset_csr = 0U;        /* RCC->CSR at boot; bit29 IWDGRSTF = last reset by watchdog */
volatile uint32_t sys_iwdg_reset = 0U;
volatile uint32_t sys_beat_can = 0U;
volatile uint32_t sys_beat_imu = 0U;
volatile uint32_t sys_beat_rf = 0U;
volatile uint32_t sys_wdg_stall_mask = 0U;   /* bit0 CAN, bit1 IMU, bit2 RF: kicking stopped */
volatile uint32_t sys_wdg_kicks = 0U;
volatile uint32_t sys_stack_free_default = 0U; /* minimum ever free stack, words */
volatile uint32_t sys_stack_free_can = 0U;
volatile uint32_t sys_stack_free_imu = 0U;
volatile uint32_t sys_stack_free_rf = 0U;
volatile uint32_t sys_heap_free_min = 0U;
/* Bench test of the reset path only: writing 0xDEAD via SWD stops the kicks (bit31 in the mask). */
volatile uint32_t sys_debug_wdg_test = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
void StartDefaultTask(void *argument);
void Start_CAN_Task(void *argument);
void Start_IMU_Task(void *argument);

/* USER CODE BEGIN PFP */
static void CAN_Filter_Config(void);
static uint8_t CAN_Send_Imu3Axis(uint32_t std_id, int16_t x, int16_t y, int16_t z, uint16_t seq);

static uint8_t ODrive_SendFrame(uint8_t node_id, uint32_t cmd_id, const uint8_t *data, uint8_t dlc);
static void ODrive_PutU32(uint8_t *buf, uint32_t value);
static void ODrive_PutFloat(uint8_t *buf, float value);

static uint8_t ODrive_Send_Clear_Errors(uint8_t node_id);
static uint8_t ODrive_Send_Controller_Mode(uint8_t node_id, uint32_t control_mode, uint32_t input_mode);
static uint8_t ODrive_Send_Axis_State(uint8_t node_id, uint32_t axis_state);
static uint8_t ODrive_Send_Input_Vel(uint8_t node_id, float input_vel, float torque_ff);

static uint32_t ODrive_GetU32(const uint8_t *buf);
static float ODrive_GetFloat(const uint8_t *buf);
static uint8_t ODrive_RequestFrame(uint8_t node_id, uint32_t cmd_id);
static uint8_t CAN_WaitTxComplete(uint32_t tx_mailbox, uint32_t timeout_ms);
static void CAN_UpdateDebugRegisters(void);
static void CAN_RecordTxFailure(void);
static void CAN_ResetOdriveRuntimeState(void);
static uint8_t CAN_RecoverIfBusOff(uint32_t now_tick);

static void ODrive_UpdateTimeouts(uint32_t now_tick);
static uint8_t ODrive_BothHeartbeatReady(void);
static uint8_t ODrive_BothNoFault(void);
static uint8_t ODrive_BothClosedLoop(void);
static uint8_t ODrive_RetryableFault(void);
static float Drive_GetWheelCircumference(void);
static void ODrive_UpdateTargetFromBodyVelocity(float v_mps, float w_radps);
static void ODrive_SetBothTargetVelocity(float target_rev_s);
static void ODrive_SendBothZero(void);
static void ODrive_SafeStopBoth(void);
static uint32_t Estop_ReadLevel(void);
static void Estop_Poll(void);
static void Estop_Capture(void);
static uint32_t Estop_Blocked(void);
static uint32_t Estop_CAN_Service(uint32_t now);
static uint32_t Elevator_CAN_Service(uint32_t now);
void Start_RF_Task(void *argument);
static void Wdg_Start(void);
static uint32_t Wdg_Supervise(uint32_t now);
static void Sys_UpdateResourceStats(void);
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
  /* Keep the reset cause, then arm the watchdog before any init that could hang. */
  sys_reset_csr = RCC->CSR;
  sys_iwdg_reset = ((sys_reset_csr & RCC_CSR_IWDGRSTF) != 0U) ? 1U : 0U;
  RCC->CSR |= RCC_CSR_RMVF;
  Wdg_Start();
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
  MX_USART2_UART_Init();
  MX_CAN1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of imuRawQueue */
  imuRawQueueHandle = osMessageQueueNew (16, sizeof(ImuRawData_t), &imuRawQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  elevCmdQueueHandle = osMessageQueueNew(4, sizeof(ElevCmdMsg_t), NULL);
  elevCanTxQueueHandle = osMessageQueueNew(16, sizeof(ElevCanTx_t), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of CAN_Task */
  CAN_TaskHandle = osThreadNew(Start_CAN_Task, NULL, &CAN_Task_attributes);

  /* creation of IMU_Task */
  IMU_TaskHandle = osThreadNew(Start_IMU_Task, NULL, &IMU_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* CAN integration active; preserve the existing IMU producer. */
  RF_TaskHandle = osThreadNew(Start_RF_Task, NULL, &RF_Task_attributes);
  if ((defaultTaskHandle == NULL) || (CAN_TaskHandle == NULL) ||
      (IMU_TaskHandle == NULL) || (imuRawQueueHandle == NULL) ||
      (RF_TaskHandle == NULL) || (elevCmdQueueHandle == NULL) ||
      (elevCanTxQueueHandle == NULL)) { Error_Handler(); }
  estop_input_test_mode = 0U;
  estop_can_task_suspended = 0U;
  estop_imu_task_suspended = 0U;
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  uint32_t estop_saved_primask = __get_PRIMASK();
  __disable_irq();
  
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, NRF_CSN_Pin|SPI1_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : NRF_IRQ_Pin */
  GPIO_InitStruct.Pin = NRF_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(NRF_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : NRF_CE_Pin */
  GPIO_InitStruct.Pin = NRF_CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NRF_CE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : NRF_CSN_Pin SPI1_CS_Pin */
  GPIO_InitStruct.Pin = NRF_CSN_Pin|SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ESTOP_Pin */
  GPIO_InitStruct.Pin = ESTOP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ESTOP_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  Estop_CoreBoot(Estop_ReadLevel(), HAL_GetTick());
  Estop_Capture();
  /* Preserve pending transitions that occurred during GPIO initialization. */
  __set_PRIMASK(estop_saved_primask);
  
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

static void CAN_Filter_Config(void)
{
  CAN_FilterTypeDef can_filter;

  /*
   * Filter bank 0:
   *  - 0x201: Jetson drive command
   *  - 0x001: ODrive node 0 heartbeat
   *  - 0x009: ODrive node 0 encoder estimates
   *  - 0x017: ODrive node 0 bus voltage/current
   */
  memset(&can_filter, 0, sizeof(can_filter));

  can_filter.FilterBank = 0;
  can_filter.FilterMode = CAN_FILTERMODE_IDLIST;
  can_filter.FilterScale = CAN_FILTERSCALE_16BIT;

  can_filter.FilterIdHigh = (uint16_t)(CAN_ID_JETSON_DRIVE_CMD << 5);
  can_filter.FilterIdLow = (uint16_t)(ODRV0_ID_HEARTBEAT << 5);
  can_filter.FilterMaskIdHigh = (uint16_t)(ODRV0_ID_ENCODER_ESTIMATES << 5);
  can_filter.FilterMaskIdLow = (uint16_t)(ODRV0_ID_BUS_VOLTAGE_CURRENT << 5);

  can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  can_filter.FilterActivation = ENABLE;
  can_filter.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan1, &can_filter) != HAL_OK)
  {
    Error_Handler();
  }

  /*
   * Filter bank 1:
   *  - 0x021: ODrive node 1 heartbeat
   *  - 0x029: ODrive node 1 encoder estimates
   *  - 0x037: ODrive node 1 bus voltage/current
   *  - 0x210: Jetson elevator button command
   */
  memset(&can_filter, 0, sizeof(can_filter));

  can_filter.FilterBank = 1;
  can_filter.FilterMode = CAN_FILTERMODE_IDLIST;
  can_filter.FilterScale = CAN_FILTERSCALE_16BIT;

  can_filter.FilterIdHigh = (uint16_t)(ODRV1_ID_HEARTBEAT << 5);
  can_filter.FilterIdLow = (uint16_t)(ODRV1_ID_ENCODER_ESTIMATES << 5);
  can_filter.FilterMaskIdHigh = (uint16_t)(ODRV1_ID_BUS_VOLTAGE_CURRENT << 5);
  can_filter.FilterMaskIdLow = (uint16_t)(ELEV_CAN_ID_COMMAND << 5);

  can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  can_filter.FilterActivation = ENABLE;
  can_filter.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan1, &can_filter) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];
  uint8_t node_id;
  uint32_t cmd_id;

  if (hcan->Instance != CAN1)
  {
    return;
  }

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U)
  {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
    {
      continue;
    }

    if (rx_header.IDE != CAN_ID_STD)
    {
      continue;
    }

    if ((rx_header.StdId == CAN_ID_JETSON_DRIVE_CMD) &&
        (rx_header.RTR == CAN_RTR_DATA) &&
        (rx_header.DLC == 8U))
    {
      float v = ODrive_GetFloat(&rx_data[0]);
      float w = ODrive_GetFloat(&rx_data[4]);
      jetson_drive_rx_count++;
      if (Estop_Blocked() || !isfinite(v) || !isfinite(w) || !odrv_both_ready)
      {
        estop_drive_rejected++;
        continue;
      }
      /* A fresh zero command after initialization/reset is mandatory. */
      if (!estop_drive_enabled)
      {
        if ((v != 0.0f) || (w != 0.0f)) { estop_drive_rejected++; continue; }
        estop_drive_enabled = 1U;
      }
      jetson_drive_last_tick = HAL_GetTick();

      jetson_cmd_v_mps = ODrive_GetFloat(&rx_data[0]);
      jetson_cmd_w_radps = ODrive_GetFloat(&rx_data[4]);

      /*
       * Jetson sends body velocity command:
       *   byte 0~3: float32 v_mps
       *   byte 4~7: float32 w_radps
       * STM32 converts (v, w) to right/left wheel rev/s by differential-drive inverse kinematics.
       */
      ODrive_UpdateTargetFromBodyVelocity(jetson_cmd_v_mps, jetson_cmd_w_radps);

      continue;
    }

    if (rx_header.StdId == ELEV_CAN_ID_COMMAND)
    {
      /* Validation (BAD_FRAME/NO_ROUTE) runs in RF_Task, outside the ISR. */
      ElevCmdMsg_t msg;
      msg.dlc = (rx_header.RTR == CAN_RTR_DATA) ? (uint8_t)rx_header.DLC : 0U;
      memcpy(msg.data, rx_data, sizeof(msg.data));
      if (osMessageQueuePut(elevCmdQueueHandle, &msg, 0U, 0U) == osOK)
      {
        (void)osThreadFlagsSet(RF_TaskHandle, RF_FLAG_CMD);
      }
      else
      {
        elev_cmd_queue_full++;
      }
      continue;
    }

    if (rx_header.RTR != CAN_RTR_DATA)
    {
      continue;
    }

    node_id = ODRV_NODE_FROM_CAN_ID(rx_header.StdId);
    cmd_id = ODRV_CMD_FROM_CAN_ID(rx_header.StdId);

    if ((node_id != ODRV_NODE_ID_LEFT) && (node_id != ODRV_NODE_ID_RIGHT))
    {
      continue;
    }

    if ((cmd_id == ODRV_CMD_HEARTBEAT) && (rx_header.DLC >= 7U))
    {
      if (node_id == ODRV_NODE_ID_LEFT)
      {
        odrv0_hb_count++;
        odrv0_last_rx_tick = HAL_GetTick();
        odrv0_axis_error = ODrive_GetU32(&rx_data[0]);
        odrv0_axis_state = rx_data[4];
      }
      else
      {
        odrv1_hb_count++;
        odrv1_last_rx_tick = HAL_GetTick();
        odrv1_axis_error = ODrive_GetU32(&rx_data[0]);
        odrv1_axis_state = rx_data[4];
      }
    }
    else if ((cmd_id == ODRV_CMD_GET_ENCODER_ESTIMATES) && (rx_header.DLC == 8U))
    {
      if (node_id == ODRV_NODE_ID_LEFT)
      {
        odrv0_pos_estimate = ODrive_GetFloat(&rx_data[0]);
        odrv0_vel_estimate = ODrive_GetFloat(&rx_data[4]);
      }
      else
      {
        odrv1_pos_estimate = ODrive_GetFloat(&rx_data[0]);
        odrv1_vel_estimate = ODrive_GetFloat(&rx_data[4]);
      }
    }
    else if ((cmd_id == ODRV_CMD_GET_BUS_VOLTAGE_CURRENT) && (rx_header.DLC == 8U))
    {
      if (node_id == ODRV_NODE_ID_LEFT)
      {
        odrv0_bus_voltage = ODrive_GetFloat(&rx_data[0]);
        odrv0_bus_current = ODrive_GetFloat(&rx_data[4]);
      }
      else
      {
        odrv1_bus_voltage = ODrive_GetFloat(&rx_data[0]);
        odrv1_bus_current = ODrive_GetFloat(&rx_data[4]);
      }
    }
    else
    {
      /* Filter should prevent unused frames from arriving here. */
    }
  }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan->Instance != CAN1)
  {
    return;
  }

  can_hal_error = HAL_CAN_GetError(hcan);
  CAN_RecordTxFailure();
}

static void IMU_CS_LOW(void)
{
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
}

static void IMU_CS_HIGH(void)
{
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
}

static uint8_t IMU_ReadReg(uint8_t reg)
{
  HAL_StatusTypeDef status;
  uint8_t tx;
  uint8_t rx = 0;

  tx = reg | 0x80U;

  IMU_CS_LOW();
  status = HAL_SPI_Transmit(&hspi1, &tx, 1, 100);

  if (status == HAL_OK)
  {
    status = HAL_SPI_Receive(&hspi1, &rx, 1, 100);
  }

  IMU_CS_HIGH();

  imu_spi_status = (status == HAL_OK) ? 1U : 2U;
  return rx;
}

static void IMU_WriteReg(uint8_t reg, uint8_t data)
{
  HAL_StatusTypeDef status;
  uint8_t tx[2];

  tx[0] = reg & 0x7FU;
  tx[1] = data;

  IMU_CS_LOW();
  status = HAL_SPI_Transmit(&hspi1, tx, 2, 100);
  IMU_CS_HIGH();

  imu_spi_status = (status == HAL_OK) ? 1U : 2U;
}

static void IMU_ReadRegs(uint8_t start_reg, uint8_t *data, uint8_t len)
{
  HAL_StatusTypeDef status;
  uint8_t tx;

  tx = start_reg | 0x80U;

  IMU_CS_LOW();
  status = HAL_SPI_Transmit(&hspi1, &tx, 1, 100);

  if (status == HAL_OK)
  {
    status = HAL_SPI_Receive(&hspi1, data, len, 100);
  }

  IMU_CS_HIGH();

  imu_spi_status = (status == HAL_OK) ? 1U : 2U;
}

static void IMU_Init_Simple(void)
{
  /* CTRL3: BDU = 1, IF_INC = 1 */
  IMU_WriteReg(LSM6DSV16X_CTRL3, 0x44U);

  /* CTRL1/CTRL2: Accelerometer/Gyroscope ODR setting */
  IMU_WriteReg(LSM6DSV16X_CTRL1, IMU_ODR_30HZ_HP);
  IMU_WriteReg(LSM6DSV16X_CTRL2, IMU_ODR_30HZ_HP);

  osDelay(50);
  imu_init_done = 1U;
}

static void IMU_ReadRawData(ImuRawData_t *sample)
{
  uint8_t acc_buf[6];
  uint8_t gyro_buf[6];

  IMU_ReadRegs(LSM6DSV16X_OUTX_L_A, acc_buf, 6);
  IMU_ReadRegs(LSM6DSV16X_OUTX_L_G, gyro_buf, 6);

  sample->acc_x = (int16_t)(((uint16_t)acc_buf[1] << 8) | acc_buf[0]);
  sample->acc_y = (int16_t)(((uint16_t)acc_buf[3] << 8) | acc_buf[2]);
  sample->acc_z = (int16_t)(((uint16_t)acc_buf[5] << 8) | acc_buf[4]);

  sample->gyro_x = (int16_t)(((uint16_t)gyro_buf[1] << 8) | gyro_buf[0]);
  sample->gyro_y = (int16_t)(((uint16_t)gyro_buf[3] << 8) | gyro_buf[2]);
  sample->gyro_z = (int16_t)(((uint16_t)gyro_buf[5] << 8) | gyro_buf[4]);

  imu_raw_read_count++;
  sample->seq = (uint16_t)imu_raw_read_count;
}

static uint8_t CAN_Send_Imu3Axis(uint32_t std_id, int16_t x, int16_t y, int16_t z, uint16_t seq)
{
  CAN_TxHeaderTypeDef tx_header;
  uint8_t tx_data[8];
  uint32_t tx_mailbox;

  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U)
  {
    can_imu_tx_error_count++;
    can_tx_add_error_count++;
    CAN_RecordTxFailure();
    return 0U;
  }

  tx_header.StdId = std_id;
  tx_header.ExtId = 0x00;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = 8;
  tx_header.TransmitGlobalTime = DISABLE;

  tx_data[0] = (uint8_t)((uint16_t)x & 0xFFU);
  tx_data[1] = (uint8_t)(((uint16_t)x >> 8) & 0xFFU);
  tx_data[2] = (uint8_t)((uint16_t)y & 0xFFU);
  tx_data[3] = (uint8_t)(((uint16_t)y >> 8) & 0xFFU);
  tx_data[4] = (uint8_t)((uint16_t)z & 0xFFU);
  tx_data[5] = (uint8_t)(((uint16_t)z >> 8) & 0xFFU);
  tx_data[6] = (uint8_t)(seq & 0xFFU);
  tx_data[7] = (uint8_t)((seq >> 8) & 0xFFU);

  if (HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox) != HAL_OK)
  {
    can_imu_tx_error_count++;
    can_tx_add_error_count++;
    CAN_RecordTxFailure();
    return 0U;
  }

  return 1U;
}

static void ODrive_PutU32(uint8_t *buf, uint32_t value)
{
  buf[0] = (uint8_t)(value & 0xFFU);
  buf[1] = (uint8_t)((value >> 8) & 0xFFU);
  buf[2] = (uint8_t)((value >> 16) & 0xFFU);
  buf[3] = (uint8_t)((value >> 24) & 0xFFU);
}

static void ODrive_PutFloat(uint8_t *buf, float value)
{
  uint32_t raw;

  memcpy(&raw, &value, sizeof(raw));
  ODrive_PutU32(buf, raw);
}

static uint32_t ODrive_GetU32(const uint8_t *buf)
{
  return ((uint32_t)buf[0]) |
         ((uint32_t)buf[1] << 8) |
         ((uint32_t)buf[2] << 16) |
         ((uint32_t)buf[3] << 24);
}

static float ODrive_GetFloat(const uint8_t *buf)
{
  uint32_t raw;
  float value;

  raw = ODrive_GetU32(buf);
  memcpy(&value, &raw, sizeof(value));

  return value;
}

static void CAN_UpdateDebugRegisters(void)
{
  can_esr = hcan1.Instance->ESR;
  can_tsr = hcan1.Instance->TSR;
  can_msr = hcan1.Instance->MSR;
  can_rf0r = hcan1.Instance->RF0R;
  can_hal_error = HAL_CAN_GetError(&hcan1);

  can_tec = (uint8_t)((can_esr >> 16) & 0xFFU);
  can_rec = (uint8_t)((can_esr >> 24) & 0xFFU);
  can_busoff = ((can_esr & CAN_ESR_BOFF_FLAG) != 0U) ? 1U : 0U;
}

static void CAN_RecordTxFailure(void)
{
  can_last_error_tick = HAL_GetTick();
  CAN_UpdateDebugRegisters();
}

static void CAN_ResetOdriveRuntimeState(void)
{
  odrv0_hb_count = 0U;
  odrv1_hb_count = 0U;
  odrv0_last_rx_tick = 0U;
  odrv1_last_rx_tick = 0U;
  odrv0_hb_timeout = 1U;
  odrv1_hb_timeout = 1U;

  odrv_both_ready = 0U;
  estop_drive_enabled = 0U;
  odrv_last_vel_cmd_tick = 0U;
  ODrive_SetBothTargetVelocity(TEST_STOP_REV_S);
}

static uint8_t CAN_RecoverIfBusOff(uint32_t now_tick)
{
  HAL_StatusTypeDef start_status;
  HAL_StatusTypeDef notif_status;

  CAN_UpdateDebugRegisters();

  if (can_busoff == 0U)
  {
    can_busoff_latched = 0U;
    return 0U;
  }

  if (can_busoff_latched == 0U)
  {
    can_busoff_latched = 1U;
    can_busoff_count++;

    CAN_ResetOdriveRuntimeState();
    odrv_control_step = 0U;
    odrv_step_tick = now_tick;
  }

  if ((now_tick - can_last_recover_tick) < CAN_BUSOFF_RECOVERY_PERIOD_MS)
  {
    return 1U;
  }

  can_last_recover_tick = now_tick;

  (void)HAL_CAN_DeactivateNotification(&hcan1, CAN_NOTIFICATION_MASK);
  (void)HAL_CAN_Stop(&hcan1);

  start_status = HAL_CAN_Start(&hcan1);
  notif_status = HAL_CAN_ActivateNotification(&hcan1, CAN_NOTIFICATION_MASK);

  if ((start_status == HAL_OK) && (notif_status == HAL_OK))
  {
    can_recover_count++;
  }
  else
  {
    can_recover_error_count++;
  }

  CAN_UpdateDebugRegisters();

  if (can_busoff == 0U)
  {
    can_busoff_latched = 0U;
  }

  return 1U;
}

static uint8_t CAN_WaitTxComplete(uint32_t tx_mailbox, uint32_t timeout_ms)
{
  uint32_t start_tick;

  start_tick = osKernelGetTickCount();

  while (HAL_CAN_IsTxMessagePending(&hcan1, tx_mailbox))
  {
    if (Estop_Blocked())
    {
      (void)HAL_CAN_AbortTxRequest(&hcan1, tx_mailbox);
      return 0U;
    }
    if ((osKernelGetTickCount() - start_tick) > timeout_ms)
    {
      HAL_CAN_AbortTxRequest(&hcan1, tx_mailbox);
      can_tx_timeout_count++;
      CAN_RecordTxFailure();
      return 0U;
    }

    osDelay(1);
  }

  return ((hcan1.Instance->TSR &
          ((tx_mailbox == CAN_TX_MAILBOX0) ? CAN_TSR_TXOK0 :
           (tx_mailbox == CAN_TX_MAILBOX1) ? CAN_TSR_TXOK1 : CAN_TSR_TXOK2)) != 0U)
         && !Estop_Blocked();
}

static uint8_t ODrive_RequestFrame(uint8_t node_id, uint32_t cmd_id)
{
  CAN_TxHeaderTypeDef tx_header;
  uint8_t tx_data[1] = {0};
  uint32_t tx_mailbox;
  uint32_t irq;
  HAL_StatusTypeDef added;

  tx_header.StdId = ODRV_CAN_ID(node_id, cmd_id);
  tx_header.ExtId = 0x00;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_REMOTE;
  tx_header.DLC = 0;
  tx_header.TransmitGlobalTime = DISABLE;

  irq = __get_PRIMASK();
  __disable_irq();
  if (Estop_Blocked()) { __set_PRIMASK(irq); return 0U; }
  added = HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox);
  __set_PRIMASK(irq);
  if (added != HAL_OK)
  {
    can_tx_add_error_count++;
    odrv_req_error_count++;
    CAN_RecordTxFailure();
    return 0U;
  }

  if (CAN_WaitTxComplete(tx_mailbox, CAN_CRITICAL_TX_TIMEOUT_MS) == 0U)
  {
    odrv_req_error_count++;
    return 0U;
  }

  return 1U;
}

static uint8_t ODrive_SendFrame(uint8_t node_id, uint32_t cmd_id, const uint8_t *data, uint8_t dlc)
{
  CAN_TxHeaderTypeDef tx_header;
  uint8_t tx_data[8] = {0};
  uint32_t tx_mailbox;
  uint32_t irq;
  HAL_StatusTypeDef added;
  uint8_t i;

  if (dlc > 8U)
  {
    odrv_tx_error_count++;
    return 0U;
  }

  for (i = 0U; i < dlc; i++)
  {
    tx_data[i] = data[i];
  }

  tx_header.StdId = ODRV_CAN_ID(node_id, cmd_id);
  tx_header.ExtId = 0x00;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = dlc;
  tx_header.TransmitGlobalTime = DISABLE;

  irq = __get_PRIMASK();
  __disable_irq();
  if (Estop_Blocked()) { __set_PRIMASK(irq); return 0U; }
  added = HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox);
  __set_PRIMASK(irq);
  if (added != HAL_OK)
  {
    can_tx_add_error_count++;
    odrv_tx_error_count++;
    CAN_RecordTxFailure();
    return 0U;
  }

  if (CAN_WaitTxComplete(tx_mailbox, CAN_CRITICAL_TX_TIMEOUT_MS) == 0U)
  {
    odrv_tx_error_count++;
    return 0U;
  }

  return 1U;
}

static uint8_t ODrive_Send_Clear_Errors(uint8_t node_id)
{
  uint8_t data[1] = {0};

  return ODrive_SendFrame(node_id, ODRV_CMD_CLEAR_ERRORS, data, 1U);
}

static uint8_t ODrive_Send_Controller_Mode(uint8_t node_id, uint32_t control_mode, uint32_t input_mode)
{
  uint8_t data[8] = {0};

  ODrive_PutU32(&data[0], control_mode);
  ODrive_PutU32(&data[4], input_mode);

  return ODrive_SendFrame(node_id, ODRV_CMD_SET_CONTROLLER_MODE, data, 8U);
}

static uint8_t ODrive_Send_Axis_State(uint8_t node_id, uint32_t axis_state)
{
  uint8_t data[4] = {0};

  ODrive_PutU32(&data[0], axis_state);

  return ODrive_SendFrame(node_id, ODRV_CMD_SET_AXIS_STATE, data, 4U);
}

static uint8_t ODrive_Send_Input_Vel(uint8_t node_id, float input_vel, float torque_ff)
{
  uint8_t data[8] = {0};

  ODrive_PutFloat(&data[0], input_vel);
  ODrive_PutFloat(&data[4], torque_ff);

  return ODrive_SendFrame(node_id, ODRV_CMD_SET_INPUT_VEL, data, 8U);
}

static void ODrive_UpdateTimeouts(uint32_t now_tick)
{
  if ((odrv0_hb_count == 0U) ||
      ((now_tick - odrv0_last_rx_tick) > ODRV_HEARTBEAT_TIMEOUT_MS))
  {
    odrv0_hb_timeout = 1U;
  }
  else
  {
    odrv0_hb_timeout = 0U;
  }

  if ((odrv1_hb_count == 0U) ||
      ((now_tick - odrv1_last_rx_tick) > ODRV_HEARTBEAT_TIMEOUT_MS))
  {
    odrv1_hb_timeout = 1U;
  }
  else
  {
    odrv1_hb_timeout = 0U;
  }
}

static uint8_t ODrive_BothHeartbeatReady(void)
{
  if ((odrv0_hb_count >= 5U) &&
      (odrv1_hb_count >= 5U) &&
      (odrv0_hb_timeout == 0U) &&
      (odrv1_hb_timeout == 0U))
  {
    return 1U;
  }

  return 0U;
}

static uint8_t ODrive_BothNoFault(void)
{
  if ((odrv0_axis_error == 0U) &&
      (odrv1_axis_error == 0U) &&
      (odrv0_hb_timeout == 0U) &&
      (odrv1_hb_timeout == 0U))
  {
    return 1U;
  }

  return 0U;
}

static uint8_t ODrive_BothClosedLoop(void)
{
  if ((odrv0_axis_state == ODRV_AXIS_STATE_CLOSED_LOOP) &&
      (odrv1_axis_state == ODRV_AXIS_STATE_CLOSED_LOOP))
  {
    return 1U;
  }

  return 0U;
}

static uint8_t ODrive_RetryableFault(void)
{
  uint32_t retryable_mask;

  retryable_mask = ODRV_AXIS_ERROR_WATCHDOG_TIMER_EXPIRED;

  if (((odrv0_axis_error & ~retryable_mask) == 0U) &&
      ((odrv1_axis_error & ~retryable_mask) == 0U))
  {
    return 1U;
  }

  return 0U;
}

static float Drive_GetWheelCircumference(void)
{
  float diameter_m;

  diameter_m = drive_wheel_diameter_m;

  if (diameter_m < 0.010f)
  {
    diameter_m = DRIVE_WHEEL_DIAMETER_DEFAULT_M;
  }

  return DRIVE_PI * diameter_m;
}

static void ODrive_UpdateTargetFromBodyVelocity(float v_mps, float w_radps)
{
  float wheel_circumference_m;
  float track_width_m;
  float right_rev_s;
  float left_rev_s;

  wheel_circumference_m = Drive_GetWheelCircumference();
  track_width_m = drive_track_width_m;

  if (track_width_m < 0.010f)
  {
    track_width_m = DRIVE_TRACK_WIDTH_DEFAULT_M;
  }

  /*
   * Jetson control_mux owns body and wheel speed limits. Apply only
   * differential-drive inverse kinematics and physical motor direction here.
   * Do not independently clip either wheel: that would change the requested yaw.
   * +v = forward; +w = counter-clockwise yaw.
   * Node 0 = right wheel; node 1 = left wheel.
   */
  ik_right_wheel_mps = v_mps + (w_radps * track_width_m * 0.5f);
  ik_left_wheel_mps  = v_mps - (w_radps * track_width_m * 0.5f);

  right_rev_s = ik_right_wheel_mps / wheel_circumference_m;
  left_rev_s  = ik_left_wheel_mps  / wheel_circumference_m;

  ik_right_rev_s_raw = right_rev_s;
  ik_left_rev_s_raw = left_rev_s;

  odrv0_target_vel = drive_node0_direction * right_rev_s;
  odrv1_target_vel = drive_node1_direction * left_rev_s;

  ik_odrv0_cmd_rev_s = odrv0_target_vel;
  ik_odrv1_cmd_rev_s = odrv1_target_vel;
}

static void ODrive_SetBothTargetVelocity(float target_rev_s)
{
  odrv0_target_vel = target_rev_s;
  odrv1_target_vel = target_rev_s;

  ik_right_wheel_mps = 0.0f;
  ik_left_wheel_mps = 0.0f;
  ik_right_rev_s_raw = target_rev_s;
  ik_left_rev_s_raw = target_rev_s;
  ik_odrv0_cmd_rev_s = target_rev_s;
  ik_odrv1_cmd_rev_s = target_rev_s;
}

static void ODrive_SendBothZero(void)
{
  ODrive_SetBothTargetVelocity(TEST_STOP_REV_S);

  (void)ODrive_Send_Input_Vel(ODRV_NODE_ID_LEFT,  0.0f, 0.0f);
  (void)ODrive_Send_Input_Vel(ODRV_NODE_ID_RIGHT, 0.0f, 0.0f);
}

static void ODrive_SafeStopBoth(void)
{
  ODrive_SendBothZero();

  (void)ODrive_Send_Axis_State(ODRV_NODE_ID_LEFT,  ODRV_AXIS_STATE_IDLE);
  (void)ODrive_Send_Axis_State(ODRV_NODE_ID_RIGHT, ODRV_AXIS_STATE_IDLE);

  odrv_safe_stop_count++;
  odrv_both_ready = 0U;
}

static uint32_t Estop_ReadLevel(void)
{
  return (HAL_GPIO_ReadPin(ESTOP_GPIO_Port, ESTOP_Pin) == GPIO_PIN_SET) ? 1U : 0U;
}

static void Estop_Poll(void)
{
  uint32_t saved_primask = __get_PRIMASK();
  uint32_t level;
  uint32_t now;
  uint32_t pending;
  __disable_irq();
  level = Estop_ReadLevel();
  now = HAL_GetTick();
  pending = (__HAL_GPIO_EXTI_GET_IT(ESTOP_Pin) != RESET) ? 1U : 0U;
  Estop_CorePoll(level, now, pending | estop_can_stop_pending);
  Estop_CoreGuard(Estop_ReadLevel());
  Estop_Capture();
  __set_PRIMASK(saved_primask);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == ESTOP_Pin)
  {
    uint32_t level = Estop_ReadLevel();
    uint32_t now = HAL_GetTick();
    Estop_CoreEdge(level, now);
    Estop_Capture();
  }
  else if (GPIO_Pin == NRF_IRQ_Pin)
  {
    rf_irq_cycles = DWT->CYCCNT;
    if (RF_TaskHandle != NULL) { (void)osThreadFlagsSet(RF_TaskHandle, RF_FLAG_IRQ); }
  }
  /* PC13/B1 (EXTI15_10) is not the E-stop input. */
}


/* ISR publishes a sticky event; only CAN_Task calls the CAN HAL. */
static void Estop_Capture(void)
{
  if (estop_latched && !estop_can_inhibit)
  {
    estop_can_inhibit = 1U;
    estop_can_stop_pending = 1U;
    estop_can_trip_count++;
    estop_can_trip_ms = HAL_GetTick();
    estop_drive_enabled = 0U;
  }
}

static uint32_t Estop_Blocked(void)
{
  uint32_t irq = __get_PRIMASK(), blocked;
  __disable_irq();
  Estop_CoreGuard(Estop_ReadLevel());
  Estop_Capture();
  blocked = estop_latched | estop_can_inhibit;
  __set_PRIMASK(irq);
  return blocked;
}

/* A burst owns all three mailboxes until completion/abort. TX interrupts stay
 * disabled: TSR.TXOK is collected before any mailbox is reused. No busy wait. */
static uint32_t Estop_CAN_Service(uint32_t now)
{
  static uint32_t active, tracked, sent_at, phase, last_round;
  static uint32_t last_status, sequence, last_flags = UINT32_MAX;
  uint32_t irq, flags, mb, i, tsr, aborting = 0U, released = 0U;
  uint8_t data[8] = {0};
  CAN_TxHeaderTypeDef h = {0};
  const uint32_t all = CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2;
  estop_can_mailbox_hold = 0U;
  (void)Estop_Blocked();
  irq = __get_PRIMASK();
  __disable_irq();
  if (estop_can_stop_pending)
  {
    (void)HAL_CAN_AbortTxRequest(&hcan1, all);
    tracked = 0U;
    active = 1U;
    phase = 0U;
    last_round = now - ESTOP_CAN_PERIOD_MS;
    last_flags = UINT32_MAX;
    estop_can_stop_pending = 0U;
    odrv_both_ready = 0U;
    estop_drive_enabled = 0U;
    ODrive_SetBothTargetVelocity(0.0f);
    jetson_cmd_v_mps = jetson_cmd_w_radps = 0.0f;
    jetson_drive_last_tick = 0U;
    aborting = 1U;
  }
  else if (active && !estop_latched)
  {
    /* Only a debounced release (ESTOP_AUTO_RELEASE) or explicit reset reaches here. */
    (void)HAL_CAN_AbortTxRequest(&hcan1, all);
    tracked = 0U;
    active = 0U;
    estop_can_inhibit = 0U;
    CAN_ResetOdriveRuntimeState();
    jetson_drive_last_tick = 0U;
    jetson_cmd_v_mps = jetson_cmd_w_radps = 0.0f;
    odrv_control_step = 0U;
    odrv_step_tick = now;
    last_flags = UINT32_MAX;
    aborting = 1U;
    released = 1U;
  }
  __set_PRIMASK(irq);
  /* IMU frames are held back while stopped; drop samples that went stale. */
  if (released) { (void)osMessageQueueReset(imuRawQueueHandle); }
  if (aborting) { return 1U; }

  if (tracked)
  {
    if (HAL_CAN_IsTxMessagePending(&hcan1, tracked))
    {
      if ((uint32_t)(now - sent_at) >= CAN_CRITICAL_TX_TIMEOUT_MS)
      {
        (void)HAL_CAN_AbortTxRequest(&hcan1, tracked);
        estop_can_tx_timeout++;
        sent_at = now;
      }
      estop_can_mailbox_hold = 1U; /* own frames in flight: TXOK must stay ours */
      return 1U;
    }
    tsr = hcan1.Instance->TSR;
    for (i = 0U; i < 3U; i++)
    {
      if (!(tracked & (1UL << i))) { continue; }
      if (tsr & (CAN_TSR_TXOK0 << (8U * i))) { estop_can_tx_ok++; }
      else { estop_can_tx_failed++; }
    }
    tracked = 0U;
  }
  ODrive_UpdateTimeouts(now);
  CAN_UpdateDebugRegisters();
  estop_can_idle_observed =
      (!odrv0_hb_timeout && !odrv1_hb_timeout &&
       odrv0_axis_state == ODRV_AXIS_STATE_IDLE && odrv1_axis_state == ODRV_AXIS_STATE_IDLE &&
       (!active || ((int32_t)(odrv0_last_rx_tick - estop_can_trip_ms) > 0 &&
                    (int32_t)(odrv1_last_rx_tick - estop_can_trip_ms) > 0))) ? 1U : 0U;
  flags = (estop_raw ? 1U : 0U) | (estop_debounced ? 2U : 0U) |
          (estop_latched ? 4U : 0U) | (estop_can_inhibit ? 8U : 0U) |
          (estop_can_idle_observed ? 16U : 0U) | (can_busoff ? 32U : 0U) |
          ((!odrv0_hb_timeout && !odrv1_hb_timeout) ? 64U : 0U) |
          (estop_drive_enabled ? 128U : 0U);
  estop_can_status_flags = flags;
  if (!active && flags == last_flags &&
      (uint32_t)(now - last_status) < ESTOP_CAN_PERIOD_MS) { return 0U; }
  if (active && phase == 0U &&
      (uint32_t)(now - last_round) < ESTOP_CAN_PERIOD_MS &&
      flags == last_flags) { return 1U; }

  /* Prior IMU/elevator sends must finish before reserving the three-mailbox burst.
   * While stopping they are aborted at once; otherwise after 100 ms, so a
   * disconnected bus cannot leave stale frames occupying slots forever. */
  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 3U)
  {
    estop_can_mailbox_hold = 1U;
    if (active || ((uint32_t)(now - last_status) >= CAN_CRITICAL_TX_TIMEOUT_MS))
    { (void)HAL_CAN_AbortTxRequest(&hcan1, all); }
    return 1U;
  }
  h.IDE = CAN_ID_STD;
  h.RTR = CAN_RTR_DATA;
  irq = __get_PRIMASK();
  __disable_irq();
  /* Recheck after building flags: a new EXTI must be handled first. */
  if (estop_can_stop_pending) { __set_PRIMASK(irq); return 1U; }
  if (active)
  {
    for (i = 0U; i < 2U; i++)
    {
      h.StdId = ODRV_CAN_ID(i, phase == 0U ? ODRV_CMD_ESTOP : ODRV_CMD_SET_AXIS_STATE);
      h.DLC = phase == 0U ? 0U : 4U;
      data[0] = 1U; /* little endian IDLE, ignored for DLC=0 */
      if (HAL_CAN_AddTxMessage(&hcan1, &h, data, &mb) == HAL_OK)
      { tracked |= mb; estop_can_stop_queued++; }
      else { estop_can_tx_failed++; }
    }
    if (phase == 0U) { last_round = now; phase = 1U; }
    else { phase = 0U; }
  }
  h.StdId = CAN_ID_ESTOP_STATUS;
  h.DLC = 8U;
  data[0] = 1U;
  data[1] = (uint8_t)flags;
  data[2] = (uint8_t)estop_can_trip_count;
  data[3] = (uint8_t)(estop_can_trip_count >> 8);
  data[4] = (uint8_t)sequence;
  data[5] = (uint8_t)(sequence >> 8);
  data[6] = (uint8_t)estop_reset_accepted;
  data[7] = (uint8_t)estop_reset_rejected;
  if (HAL_CAN_AddTxMessage(&hcan1, &h, data, &mb) == HAL_OK)
  {
    tracked |= mb;
    sequence++;
    estop_can_status_queued++;
    last_flags = flags;
    last_status = now;
  }
  else { estop_can_tx_failed++; }
  sent_at = now;
  estop_can_mailbox_hold = (tracked != 0U) ? 1U : 0U;
  __set_PRIMASK(irq);
  return active || tracked;
}

/* Sends at most one queued 0x310/0x311 per call, never ahead of an E-stop burst. */
static uint32_t Elevator_CAN_Service(uint32_t now)
{
  ElevCanTx_t m;
  CAN_TxHeaderTypeDef h = {0};
  uint32_t mb;
  (void)now;
  if (estop_can_stop_pending || estop_can_mailbox_hold) { return 0U; }
  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U) { return 0U; }
  while (osMessageQueueGet(elevCanTxQueueHandle, &m, NULL, 0U) == osOK)
  {
    /* Delayed reports are dropped, not sent late (Jetson discards > 0.5 s anyway). */
    if ((int32_t)(HAL_GetTick() - m.stamp_ms) > (int32_t)ELEV_CAN_FRESH_MS)
    {
      elev_can_tx_stale++;
      continue;
    }
    h.StdId = m.id;
    h.IDE = CAN_ID_STD;
    h.RTR = CAN_RTR_DATA;
    h.DLC = 8U;
    h.TransmitGlobalTime = DISABLE;
    if (HAL_CAN_AddTxMessage(&hcan1, &h, m.data, &mb) == HAL_OK) { elev_can_tx_sent++; }
    else { elev_can_tx_failed++; }
    return 1U;
  }
  return 0U;
}

/* elevator_rf.c adapter: called only from RF_Task. */
void Elev_PortEmitCan(uint32_t std_id, const uint8_t data[ELEV_CAN_LEN])
{
  ElevCanTx_t m;
  m.id = (uint16_t)std_id;
  memcpy(m.data, data, sizeof(m.data));
  m.stamp_ms = HAL_GetTick();
  if (osMessageQueuePut(elevCanTxQueueHandle, &m, 0U, 0U) == osOK) { elev_can_tx_queued++; }
  else { elev_can_tx_dropped++; }
}

/* nrf24.c adapter: SPI2 + PB12 CSN + PB2 CE, called only from RF_Task. */
void Nrf24_PortCsn(uint32_t high)
{
  HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Nrf24_PortCe(uint32_t high)
{
  HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

uint32_t Nrf24_PortTransfer(const uint8_t *tx, uint8_t *rx, uint32_t len)
{
  return HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)tx, rx, (uint16_t)len, 5U) == HAL_OK;
}

/* Long waits yield to the RTOS; short ones spin on the DWT cycle counter. */
void Nrf24_PortDelayUs(uint32_t us)
{
  uint32_t start;
  uint32_t cycles;
  if (us >= 1000U)
  {
    osDelay((us + 999U) / 1000U + 1U);
    return;
  }
  start = DWT->CYCCNT;
  cycles = us * (SystemCoreClock / 1000000U);
  while ((DWT->CYCCNT - start) < cycles) { }
}

static void RF_RecordLatency(void)
{
  uint32_t us = (DWT->CYCCNT - rf_irq_cycles) / (SystemCoreClock / 1000000U);
  elev_lat_last_us = us;
  if (us > elev_lat_max_us) { elev_lat_max_us = us; }
}

static void RF_ScanChannels(void)
{
  uint32_t hits;
  for (uint32_t c = 0U; c < ELEV_SCAN_CHANNELS; c++)
  {
    uint32_t total = 0U;
    for (uint32_t done = 0U; done < ELEV_SCAN_SAMPLES; done += ELEV_SCAN_CHUNK)
    {
      Nrf24_Scan((uint8_t)(ELEV_SCAN_FIRST_CH + c), 1U, ELEV_SCAN_CHUNK, &hits);
      total += hits;
      sys_beat_rf++;
      osDelay(1); /* keep E-stop polling and CAN_Task running */
    }
    elev_scan_hits[c] = total;
  }
}

/* RF_Task: nRF24 IRQ -> read FIFO -> core -> COMMAND within 3 ms (RF doc 7.5). */
void Start_RF_Task(void *argument)
{
  ElevCmdMsg_t cmd;
  uint8_t frames[NRF24_RX_MAX][NRF24_PAYLOAD_LEN];
  uint8_t tx[ELEV_RF_LEN];
  uint8_t dst = 0U;
  uint32_t now;
  uint32_t n;
  uint32_t rpd;
  uint32_t ovf;
  uint32_t tx_active = 0U;
  uint32_t tx_started = 0U;
  uint32_t last_check;
  (void)argument;

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  now = HAL_GetTick();
  Elev_CoreInit(now);
  elev_radio_ok = Nrf24_Init(ELEV_AMR_NODE, ELEV_BROADCAST_NODE);
  elev_radio_init_count++;
  Elev_SetRadioOk(elev_radio_ok);
  last_check = now;

  for (;;)
  {
    (void)osThreadFlagsWait(RF_FLAG_IRQ | RF_FLAG_CMD, osFlagsWaitAny, RF_TICK_MS);
    now = HAL_GetTick();
    elev_rf_task_loops++;
    sys_beat_rf++;

    if (elev_radio_ok)
    {
      if (tx_active)
      {
        uint32_t outcome = Nrf24_PollTx();
        if (outcome != NRF24_TX_BUSY)
        {
          tx_active = 0U;
          Elev_OnTxResult(outcome == NRF24_TX_OK, now);
        }
        else if ((uint32_t)(now - tx_started) > RF_TX_TIMEOUT_MS)
        {
          Nrf24_AbortTx();
          tx_active = 0U;
          elev_tx_timeout_count++;
          Elev_NoteTxTimeout();
          Elev_OnTxResult(0U, now);
        }
      }
      /* IRQ is level-low until cleared; polling the pin also covers a missed edge. */
      if (!tx_active && (HAL_GPIO_ReadPin(NRF_IRQ_GPIO_Port, NRF_IRQ_Pin) == GPIO_PIN_RESET))
      {
        n = Nrf24_ReadRx(frames, NRF24_RX_MAX, &rpd, &ovf);
        if (ovf) { elev_rx_overflow_count++; Elev_NoteRxOverflow(); }
        for (uint32_t i = 0U; i < n; i++)
        {
          if (Elev_OnRfFrame(frames[i], NRF24_PAYLOAD_LEN, rpd, now, tx, &dst) && !tx_active)
          {
            Nrf24_StartTx(dst, tx);
            RF_RecordLatency();
            tx_active = 1U;
            tx_started = now;
          }
        }
      }
    }

    while (osMessageQueueGet(elevCmdQueueHandle, &cmd, NULL, 0U) == osOK)
    {
      Elev_OnCanCommand(cmd.data, cmd.dlc, now);
    }
    if (elev_debug_cmd_request != 0U)
    {
      uint8_t d[8];
      for (uint32_t i = 0U; i < sizeof(d); i++) { d[i] = elev_debug_cmd[i]; }
      elev_debug_cmd_request = 0U;
      elev_debug_cmd_count++;
      Elev_OnCanCommand(d, sizeof(d), now);
    }
    if ((elev_debug_scan_request != 0U) && elev_radio_ok && !tx_active)
    {
      elev_debug_scan_request = 0U;
      RF_ScanChannels();
      elev_scan_done++;
      now = HAL_GetTick();
    }

    /* Re-verify the radio every second; a supply dip resets the module's registers. */
    if (((uint32_t)(now - last_check) >= RF_CHECK_PERIOD_MS) && !tx_active)
    {
      last_check = now;
      if (elev_radio_ok && !Nrf24_CheckConfig())
      {
        elev_radio_ok = 0U;
        elev_radio_lost_count++;
      }
      if (!elev_radio_ok)
      {
        elev_radio_ok = Nrf24_Init(ELEV_AMR_NODE, ELEV_BROADCAST_NODE);
        elev_radio_init_count++;
      }
      Elev_SetRadioOk(elev_radio_ok);
      elev_heap_free = xPortGetFreeHeapSize();
    }
    Elev_Tick(now);
  }
}

/* IWDG cannot be stopped once started. It freezes while a debugger halts the core. */
static void Wdg_Start(void)
{
  DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;
  IWDG->KR = 0xCCCCU;   /* start (LSI turns on) */
  IWDG->KR = 0x5555U;   /* unlock PR/RLR */
  IWDG->PR = WDG_IWDG_PR_DIV64;
  IWDG->RLR = WDG_IWDG_RELOAD;
  while (IWDG->SR != 0U) { }
  IWDG->KR = 0xAAAAU;
}

/* Called from defaultTask (2 ms). Kicks only while every monitored task beat within
 * WDG_TASK_STALL_MS; a hung defaultTask, scheduler or ISR stops the kicks as well. */
static uint32_t Wdg_Supervise(uint32_t now)
{
  static uint32_t seen[WDG_TASKS];
  static uint32_t changed_ms[WDG_TASKS];
  static uint32_t started;
  const uint32_t beat[WDG_TASKS] = {sys_beat_can, sys_beat_imu, sys_beat_rf};
  uint32_t stalled = 0U;

  for (uint32_t i = 0U; i < WDG_TASKS; i++)
  {
    if (!started || (beat[i] != seen[i]))
    {
      seen[i] = beat[i];
      changed_ms[i] = now;
    }
    else if ((uint32_t)(now - changed_ms[i]) > WDG_TASK_STALL_MS)
    {
      stalled |= 1UL << i;
    }
  }
  started = 1U;
  if (sys_debug_wdg_test == 0xDEADU) { stalled |= 0x80000000UL; }
  sys_wdg_stall_mask = stalled;
  if (stalled == 0U)
  {
    IWDG->KR = 0xAAAAU;
    sys_wdg_kicks++;
  }
  return stalled;
}

/* Minimum-ever free stack per task (words) and heap, for sizing checks. */
static void Sys_UpdateResourceStats(void)
{
  sys_stack_free_default = uxTaskGetStackHighWaterMark(NULL);
  sys_stack_free_can = uxTaskGetStackHighWaterMark((TaskHandle_t)CAN_TaskHandle);
  sys_stack_free_imu = uxTaskGetStackHighWaterMark((TaskHandle_t)IMU_TaskHandle);
  sys_stack_free_rf = uxTaskGetStackHighWaterMark((TaskHandle_t)RF_TaskHandle);
  sys_heap_free_min = xPortGetMinimumEverFreeHeapSize();
}

/* USER CODE END 4 */


/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  (void)argument;
  uint32_t period_ticks = (osKernelGetTickFreq() * ESTOP_POLL_MS + 999U) / 1000U;
  if (period_ticks == 0U) { period_ticks = 1U; }
  uint32_t stats_tick = HAL_GetTick() - SYS_STATS_PERIOD_MS;
  for (;;)
  {
    Estop_Poll();
    (void)Wdg_Supervise(HAL_GetTick());
    if ((uint32_t)(HAL_GetTick() - stats_tick) >= SYS_STATS_PERIOD_MS)
    {
      stats_tick = HAL_GetTick();
      Sys_UpdateResourceStats();
    }
    osDelay(period_ticks);
  }
  
/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Start_CAN_Task */
/**
* @brief Function implementing the CAN_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_CAN_Task */

void Start_CAN_Task(void *argument)
{
  /* USER CODE BEGIN Start_CAN_Task */

  ImuRawData_t imu_sample;
  uint32_t odrv_last_feedback_req_tick = 0U;
  uint32_t now_tick;
  uint8_t odrv0_vel_tx_ok;
  uint8_t odrv1_vel_tx_ok;

  CAN_Filter_Config();

  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_CAN_ActivateNotification(&hcan1, CAN_NOTIFICATION_MASK) != HAL_OK)
  {
    Error_Handler();
  }

  for (;;)
  {
    now_tick = HAL_GetTick();
    sys_beat_can++;

    uint32_t safety_busy = Estop_CAN_Service(now_tick);
    if (CAN_RecoverIfBusOff(now_tick) != 0U)
    {
      osDelay(10);
      continue;
    }

    /* Elevator relay keeps reporting during an E-stop latch (CAN doc 6). */
    (void)Elevator_CAN_Service(now_tick);

    if (safety_busy || Estop_Blocked()) { osDelay(1); continue; }

    if (osMessageQueueGet(imuRawQueueHandle, &imu_sample, NULL, 1U) == osOK)
    {
      (void)CAN_Send_Imu3Axis(
          CAN_ID_IMU_ACC_RAW,
          imu_sample.acc_x,
          imu_sample.acc_y,
          imu_sample.acc_z,
          imu_sample.seq
      );

      (void)CAN_Send_Imu3Axis(
          CAN_ID_IMU_GYRO_RAW,
          imu_sample.gyro_x,
          imu_sample.gyro_y,
          imu_sample.gyro_z,
          imu_sample.seq
      );
    }

    now_tick = HAL_GetTick();
    CAN_UpdateDebugRegisters();

    ODrive_UpdateTimeouts(now_tick);

    /*
     * ODrive feedback request
     * - Runs only after both ODrive nodes are in normal control step.
     * - Requests node 0 and node 1 encoder estimates and bus voltage/current.
     */
    if ((ODRV_ENABLE_FEEDBACK_REQUESTS != 0U) &&
        (odrv_control_step >= 5U) &&
        (ODrive_BothNoFault() != 0U) &&
        ((now_tick - odrv_last_feedback_req_tick) >= ODRV_FEEDBACK_REQ_PERIOD_MS))
    {
      (void)ODrive_RequestFrame(ODRV_NODE_ID_LEFT,  ODRV_CMD_GET_ENCODER_ESTIMATES);
      (void)ODrive_RequestFrame(ODRV_NODE_ID_LEFT,  ODRV_CMD_GET_BUS_VOLTAGE_CURRENT);
      (void)ODrive_RequestFrame(ODRV_NODE_ID_RIGHT, ODRV_CMD_GET_ENCODER_ESTIMATES);
      (void)ODrive_RequestFrame(ODRV_NODE_ID_RIGHT, ODRV_CMD_GET_BUS_VOLTAGE_CURRENT);

      odrv_last_feedback_req_tick = now_tick;
    }

    /*
     * Jetson command timeout safety.
     * If Jetson command disappears, both wheel targets become zero.
     */
    if ((jetson_drive_last_tick == 0U) ||
        ((now_tick - jetson_drive_last_tick) > JETSON_CMD_TIMEOUT_MS))
    {
      if (jetson_cmd_timeout == 0U)
      {
        jetson_cmd_timeout_count++;
      }

      jetson_cmd_timeout = 1U;
      estop_drive_enabled = 0U;
      jetson_cmd_v_mps = 0.0f;
      jetson_cmd_w_radps = 0.0f;
      ODrive_SetBothTargetVelocity(TEST_STOP_REV_S);
    }
    else
    {
      jetson_cmd_timeout = 0U;
    }

    if (Estop_Blocked()) { continue; }
    switch (odrv_control_step)
    {
    case 0:
      /*
       * Wait until both ODrive S1 nodes are visible on CAN.
       */
      if (ODrive_BothHeartbeatReady() != 0U)
      {
        if (ODrive_BothClosedLoop() != 0U)
        {
          ODrive_SendBothZero();
        }

        odrv_control_step = 1U;
        odrv_step_tick = now_tick;
      }
      break;

    case 1:
      /*
       * Clear errors for both nodes.
       * Zero velocity is also sent to feed watchdogs before entering Closed Loop.
       */
      if ((ODrive_Send_Clear_Errors(ODRV_NODE_ID_LEFT) != 0U) &&
          (ODrive_Send_Clear_Errors(ODRV_NODE_ID_RIGHT) != 0U))
      {
        ODrive_SendBothZero();

        odrv_control_step = 2U;
        odrv_step_tick = now_tick;
      }
      else
      {
        odrv_control_step = 100U;
      }
      break;

    case 2:
      /*
       * Wait for Clear_Errors to be reflected in heartbeat.
       * If a watchdog error is still visible, repeat Clear_Errors + zero velocity.
       */
      if ((now_tick - odrv_step_tick) > 100U)
      {
        if ((odrv0_axis_error != 0U) || (odrv1_axis_error != 0U))
        {
          if ((ODrive_Send_Clear_Errors(ODRV_NODE_ID_LEFT) != 0U) &&
              (ODrive_Send_Clear_Errors(ODRV_NODE_ID_RIGHT) != 0U))
          {
            ODrive_SendBothZero();
            odrv_step_tick = now_tick;
          }
          else
          {
            odrv_control_step = 100U;
          }

          break;
        }

        if ((ODrive_Send_Controller_Mode(ODRV_NODE_ID_LEFT,
                                         ODRV_CONTROL_MODE_VELOCITY,
                                         ODRV_INPUT_MODE_VEL_RAMP) != 0U) &&
            (ODrive_Send_Controller_Mode(ODRV_NODE_ID_RIGHT,
                                         ODRV_CONTROL_MODE_VELOCITY,
                                         ODRV_INPUT_MODE_VEL_RAMP) != 0U))
        {
          ODrive_SendBothZero();

          odrv_control_step = 3U;
          odrv_step_tick = now_tick;
        }
        else
        {
          odrv_control_step = 100U;
        }
      }
      break;

    case 3:
      /*
       * Request Closed Loop Control for both nodes.
       */
      if ((now_tick - odrv_step_tick) > 50U)
      {
        if ((ODrive_Send_Axis_State(ODRV_NODE_ID_LEFT,
                                    ODRV_AXIS_STATE_CLOSED_LOOP) != 0U) &&
            (ODrive_Send_Axis_State(ODRV_NODE_ID_RIGHT,
                                    ODRV_AXIS_STATE_CLOSED_LOOP) != 0U))
        {
          odrv_control_step = 4U;
          odrv_step_tick = now_tick;
        }
        else
        {
          odrv_control_step = 100U;
        }
      }
      break;

    case 4:
      /*
       * Both ODrive nodes must be in Closed Loop without error.
       */
      if ((odrv0_axis_error != 0U) || (odrv1_axis_error != 0U))
      {
        odrv_control_step = 100U;
      }
      else if (ODrive_BothClosedLoop() != 0U)
      {
        odrv_control_step = 5U;
        odrv_step_tick = now_tick;
        odrv_last_vel_cmd_tick = 0U;
        odrv_both_ready = 1U;

        /*
         * Start in a safe zero-speed state.
         */
        ODrive_SendBothZero();
      }
      else if ((now_tick - odrv_step_tick) > 2000U)
      {
        odrv_control_step = 100U;
      }
      break;

    case 5:
      /*
       * Normal dual ODrive velocity command.
       * Targets are calculated from Jetson (v_mps, w_radps) by inverse kinematics.
       */
      if ((ODrive_BothNoFault() == 0U))
      {
        odrv_control_step = 100U;
      }
      else if ((now_tick - odrv_last_vel_cmd_tick) >= ODRV_VEL_CMD_PERIOD_MS)
      {
        odrv0_vel_tx_ok = ODrive_Send_Input_Vel(ODRV_NODE_ID_LEFT,
                                                odrv0_target_vel,
                                                odrv0_torque_ff);
        odrv1_vel_tx_ok = ODrive_Send_Input_Vel(ODRV_NODE_ID_RIGHT,
                                                odrv1_target_vel,
                                                odrv1_torque_ff);

        if (odrv0_vel_tx_ok != 0U)
        {
          odrv0_vel_ok_count++;
        }
        else
        {
          odrv0_vel_fail_count++;
        }

        if (odrv1_vel_tx_ok != 0U)
        {
          odrv1_vel_ok_count++;
        }
        else
        {
          odrv1_vel_fail_count++;
        }

        if ((odrv0_vel_tx_ok == 0U) || (odrv1_vel_tx_ok == 0U))
        {
          CAN_RecordTxFailure();
        }

        odrv_last_vel_cmd_tick = now_tick;
      }
      break;

    case 6:
      /*
       * Reserved for future controlled stop sequence.
       */
      if ((now_tick - odrv_step_tick) > 1000U)
      {
        ODrive_SafeStopBoth();
        odrv_control_step = 7U;
        odrv_step_tick = now_tick;
      }
      break;

    case 7:
      /*
       * Stopped / faulted state.
       * Watchdog-only faults are retried after CAN has recovered.
       */
      ODrive_SetBothTargetVelocity(TEST_STOP_REV_S);

      if (((now_tick - odrv_step_tick) >= ODRV_FAULT_RETRY_DELAY_MS) &&
          (ODrive_RetryableFault() != 0U))
      {
        CAN_ResetOdriveRuntimeState();
        odrv_control_step = 0U;
        odrv_step_tick = now_tick;
      }

      osDelay(10);
      break;

    default:
      /*
       * Fault path: stop both nodes.
       * If CAN is physically disconnected, ODrive watchdog remains the final safety layer.
       */
      ODrive_SafeStopBoth();
      odrv_control_step = 7U;
      odrv_step_tick = now_tick;
      break;
    }
  }
  /* USER CODE END Start_CAN_Task */
}


/* USER CODE BEGIN Header_Start_IMU_Task */
/**
* @brief Function implementing the IMU_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_IMU_Task */
void Start_IMU_Task(void *argument)
{
  /* USER CODE BEGIN Start_IMU_Task */
  ImuRawData_t imu_sample;
  uint32_t last_whoami_check_tick = 0U;
  uint32_t now_tick;

  osDelay(100);
  IMU_CS_HIGH();

  /*
   * Initial WHO_AM_I check
   */
  imu_whoami = IMU_ReadReg(LSM6DSV16X_WHO_AM_I_REG);

  if (imu_whoami == LSM6DSV16X_WHO_AM_I_VAL)
  {
    IMU_Init_Simple();
  }
  else
  {
    imu_init_done = 0U;
  }

  last_whoami_check_tick = HAL_GetTick();

  for (;;)
  {
    now_tick = HAL_GetTick();
    sys_beat_imu++;

    /*
     * WHO_AM_I is checked periodically, not every sample.
     * This reduces SPI overhead.
     */
    if ((imu_init_done == 0U) ||
        ((now_tick - last_whoami_check_tick) >= IMU_WHOAMI_CHECK_PERIOD_MS))
    {
      imu_whoami = IMU_ReadReg(LSM6DSV16X_WHO_AM_I_REG);
      last_whoami_check_tick = now_tick;

      if (imu_whoami == LSM6DSV16X_WHO_AM_I_VAL)
      {
        if (imu_init_done == 0U)
        {
          IMU_Init_Simple();
        }
      }
      else
      {
        imu_init_done = 0U;
      }
    }

    if (imu_init_done != 0U)
    {
      IMU_ReadRawData(&imu_sample);

      (void)osMessageQueuePut(imuRawQueueHandle, &imu_sample, 0U, 0U);
    }

    osDelay(IMU_SAMPLE_PERIOD_MS);
  }
  /* USER CODE END Start_IMU_Task */
}
/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
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
