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
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CAN_ID_JETSON_DRIVE_CMD     0x201U
#define JETSON_CMD_TIMEOUT_MS       300U

/*
 * Differential-drive calibration parameters
 * - Node 0: right wheel, positive command = forward direction
 * - Node 1: left wheel, physical direction is reversed, so direction = -1
 * - Wheel outer diameter: 172 mm -> 0.172 m
 * - Track width: 499 mm -> 0.499 m
 *
 * Speed limiting is performed by Jetson control_mux before sending (v, w).
 * ODrive Input_Vel unit is rev/s (= turn/s).
 * Final command:
 *   right_rev_s = (v + w * track_width / 2) / (pi * wheel_diameter)
 *   left_rev_s  = (v - w * track_width / 2) / (pi * wheel_diameter)
 */
#define DRIVE_PI                         3.1415926f
#define DRIVE_WHEEL_DIAMETER_DEFAULT_M   0.172f
#define DRIVE_TRACK_WIDTH_DEFAULT_M       0.499f

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

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
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
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
static void MX_SPI1_Init(void);
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
  MX_USART2_UART_Init();
  MX_CAN1_Init();
  MX_SPI1_Init();
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
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  /* defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes); */

  /* creation of CAN_Task */
  CAN_TaskHandle = osThreadNew(Start_CAN_Task, NULL, &CAN_Task_attributes);

  /* creation of IMU_Task */
  IMU_TaskHandle = osThreadNew(Start_IMU_Task, NULL, &IMU_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
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

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

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
   * Fourth slot is filled with 0x021 again to avoid accepting an unused ID.
   */
  memset(&can_filter, 0, sizeof(can_filter));

  can_filter.FilterBank = 1;
  can_filter.FilterMode = CAN_FILTERMODE_IDLIST;
  can_filter.FilterScale = CAN_FILTERSCALE_16BIT;

  can_filter.FilterIdHigh = (uint16_t)(ODRV1_ID_HEARTBEAT << 5);
  can_filter.FilterIdLow = (uint16_t)(ODRV1_ID_ENCODER_ESTIMATES << 5);
  can_filter.FilterMaskIdHigh = (uint16_t)(ODRV1_ID_BUS_VOLTAGE_CURRENT << 5);
  can_filter.FilterMaskIdLow = (uint16_t)(ODRV1_ID_HEARTBEAT << 5);

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
      jetson_drive_rx_count++;
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
    if ((osKernelGetTickCount() - start_tick) > timeout_ms)
    {
      HAL_CAN_AbortTxRequest(&hcan1, tx_mailbox);
      can_tx_timeout_count++;
      CAN_RecordTxFailure();
      return 0U;
    }

    osDelay(1);
  }

  return 1U;
}

static uint8_t ODrive_RequestFrame(uint8_t node_id, uint32_t cmd_id)
{
  CAN_TxHeaderTypeDef tx_header;
  uint8_t tx_data[1] = {0};
  uint32_t tx_mailbox;

  tx_header.StdId = ODRV_CAN_ID(node_id, cmd_id);
  tx_header.ExtId = 0x00;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_REMOTE;
  tx_header.DLC = 0;
  tx_header.TransmitGlobalTime = DISABLE;

  if (HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox) != HAL_OK)
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

  if (HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox) != HAL_OK)
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
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
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

    if (CAN_RecoverIfBusOff(now_tick) != 0U)
    {
      osDelay(10);
      continue;
    }

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
      jetson_cmd_v_mps = 0.0f;
      jetson_cmd_w_radps = 0.0f;
      ODrive_SetBothTargetVelocity(TEST_STOP_REV_S);
    }
    else
    {
      jetson_cmd_timeout = 0U;
    }

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
