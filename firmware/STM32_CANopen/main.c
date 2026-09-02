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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CO_app_STM32.h"
#include "OD.h"
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

CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim10;

/* USER CODE BEGIN PV */
/*
================================================================
STEP E — CiA 402 상태 머신 추가
main.c USER CODE 구역 교체 코드
================================================================

구현 범위 (방식 A - 최소 구현):
  ✅ Switch On Disabled → Ready to Switch On → Switched On → Operation Enabled
  ✅ Controlword 0x00 / 0x06 / 0x07 / 0x0F 처리
  ✅ Statusword 자동 갱신
  ✅ Operation Enabled 아니면 모터 정지 (게이트)

미구현 (하드웨어/범위 문제, TODO 주석 참고):
  ⬜ Fault 상태               → 전류/온도 센서 필요
  ⬜ Quick Stop        → 감속 프로파일 로직 필요
  ⬜ Halt              → 감속 프로파일 로직 필요
  ⬜ 가속/감속 프로파일     → OD 항목(0x6083/0x6084) 추가 필요
  ⬜ 실제 파워단 제어         → 릴레이 또는 EN핀 드라이버 필요
*/
// ── 로봇 스펙 ──
#define ENCODER_CPR         1320.0f
#define WHEEL_DIAMETER      0.065f
#define WHEEL_CIRCUM        (3.14159f * WHEEL_DIAMETER)

// ── 피드포워드 ──
#define FF_SLOPE            222.7f
#define FF_OFFSET           3.4f

// ── PID 게인 ──
#define KP                  150.0f
#define KI                  300.0f
#define KD                  0.0f

// ── 제어 파라미터 ──
#define CONTROL_PERIOD_MS   20
#define MAX_TARGET_VEL      0.5f
#define INTEGRAL_MAX        200.0f
#define VEL_FILTER          0.6f
#define PWM_MAX             255

// ── CiA 402 상태 정의 ──
typedef enum {
    CIA402_SWITCH_ON_DISABLED = 0,
    CIA402_READY_TO_SWITCH_ON,
    CIA402_SWITCHED_ON,
    CIA402_OPERATION_ENABLED
    // TODO(Fault):     CIA402_FAULT,            전류/온도 센서 추가 시
    // TODO(QuickStop): CIA402_QUICK_STOP_ACTIVE, 감속 프로파일 구현 시
} Cia402State;

// ── Statusword 값 (CiA 402 표준) ──
#define SW_SWITCH_ON_DISABLED   0x0040
#define SW_READY_TO_SWITCH_ON   0x0021
#define SW_SWITCHED_ON          0x0023
#define SW_OPERATION_ENABLED    0x0027
// TODO(Fault): #define SW_FAULT  0x0008

// ── Controlword 명령 (하위 비트만 사용) ──
#define CW_SHUTDOWN             0x06    // → Ready to Switch On
#define CW_SWITCH_ON            0x07    // → Switched On
#define CW_ENABLE_OPERATION     0x0F    // → Operation Enabled
#define CW_DISABLE_VOLTAGE      0x00    // → Switch On Disabled
// TODO(Fault):     #define CW_FAULT_RESET  0x80  (비트7)
// TODO(QuickStop): Controlword 비트2를 0으로 → Quick Stop

// ── 운전 모드 (0x6060) ──
#define MODE_PROFILE_VELOCITY   3

// ── 축 구조체 ──
typedef struct {
    // 하드웨어 자원
    TIM_HandleTypeDef* enc_tim;
    TIM_HandleTypeDef* pwm_tim;
    uint32_t           pwm_channel;
    GPIO_TypeDef*      dir_port;
    uint16_t           dir_pin;
    int                enc_sign;
    int                dir_invert;

    // OD 연결
    uint16_t*          od_controlword;
    uint16_t*          od_statusword;
    int8_t*            od_mode;
    int8_t*            od_mode_display;
    int32_t*           od_target;
    int32_t*           od_actual;

    // CiA 402 상태
    Cia402State        state;
    uint16_t           prev_controlword;

    // 제어 상태
    uint16_t prev_enc_raw;
    float    actual_vel;
    float    integral;
    float    prev_error;

    // 디버그
    float    dbg_pwm_f;
    float    dbg_target;
    float    dbg_error;
    float    dbg_ff;
    long     dbg_delta;

    // TODO(Fault): 아래 필드는 센서 추가 시 사용
    // float   current_a;      전류 센서(ACS712 등) 측정값
    // float   temperature_c;  온도 센서(NTC) 측정값
    // uint16_t fault_code;    에러 코드

    // TODO(Profile): 가속/감속 프로파일 구현 시
    // float   profile_target;  램프 처리된 중간 목표속도
} MotorAxis;

MotorAxis axis_r;
MotorAxis axis_l;

uint32_t prev_control_ms = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// ── 모터 PWM 출력 (부호로 방향 결정) ──
void setMotorAxis(MotorAxis* ax, int pwm)
{
    if (pwm >  PWM_MAX) pwm =  PWM_MAX;
    if (pwm < -PWM_MAX) pwm = -PWM_MAX;

    GPIO_PinState fwd = ax->dir_invert ? GPIO_PIN_RESET : GPIO_PIN_SET;
    GPIO_PinState rev = ax->dir_invert ? GPIO_PIN_SET   : GPIO_PIN_RESET;

    if (pwm >= 0) {
        HAL_GPIO_WritePin(ax->dir_port, ax->dir_pin, fwd);
        __HAL_TIM_SET_COMPARE(ax->pwm_tim, ax->pwm_channel, pwm);
    } else {
        HAL_GPIO_WritePin(ax->dir_port, ax->dir_pin, rev);
        __HAL_TIM_SET_COMPARE(ax->pwm_tim, ax->pwm_channel, -pwm);
    }
}

// ── 피드포워드 ──
float feedforward(float v)
{
    if (v >  0.001f) return FF_SLOPE * v + FF_OFFSET;
    if (v < -0.001f) return FF_SLOPE * v - FF_OFFSET;
    return 0.0f;
}

// ── 제어 상태 리셋 (상태 전이 시 호출) ──
void resetControlState(MotorAxis* ax)
{
    ax->integral   = 0.0f;
    ax->prev_error = 0.0f;
}

// ── CiA 402 상태 머신 처리 ──
void updateStateMachine(MotorAxis* ax)
{
    uint16_t cw = *ax->od_controlword;

    // Controlword 하위 4비트만 사용 (최소 구현)
    // TODO(Fault):     비트7(0x80) Fault Reset 처리 추가
    // TODO(QuickStop): 비트2(0x04) 확인해 Quick Stop 처리 추가
    // TODO(Halt):      비트8(0x100) 확인해 Halt 처리 추가
    uint16_t cmd = cw & 0x0F;

    switch (ax->state) {

        case CIA402_SWITCH_ON_DISABLED:
            // 0x06 (Shutdown) → Ready to Switch On
            if (cmd == CW_SHUTDOWN) {
                ax->state = CIA402_READY_TO_SWITCH_ON;
            }
            break;

        case CIA402_READY_TO_SWITCH_ON:
            // 0x07 (Switch On) → Switched On
            if (cmd == CW_SWITCH_ON) {
                ax->state = CIA402_SWITCHED_ON;
                // TODO(PowerStage): 여기서 실제 파워단 전원 인가
                  //  릴레이 추가 시: HAL_GPIO_WritePin(RELAY_PORT, RELAY_PIN, SET);
            }
            // 0x00 (Disable Voltage) → 되돌아감
            else if (cmd == CW_DISABLE_VOLTAGE) {
                ax->state = CIA402_SWITCH_ON_DISABLED;
            }
            break;

        case CIA402_SWITCHED_ON:
            // 0x0F (Enable Operation) → Operation Enabled
            if (cmd == CW_ENABLE_OPERATION) {
                ax->state = CIA402_OPERATION_ENABLED;
                resetControlState(ax);   // 적분 리셋 후 제어 시작
            }
            // 0x06 (Shutdown) → 되돌아감
            else if (cmd == CW_SHUTDOWN) {
                ax->state = CIA402_READY_TO_SWITCH_ON;
                // TODO(PowerStage): 파워단 전원 차단
            }
            else if (cmd == CW_DISABLE_VOLTAGE) {
                ax->state = CIA402_SWITCH_ON_DISABLED;
            }
            break;

        case CIA402_OPERATION_ENABLED:
            // 0x07 → Switched On (제어 중지, 전원 유지)
            if (cmd == CW_SWITCH_ON) {
                ax->state = CIA402_SWITCHED_ON;
                resetControlState(ax);
            }
            // 0x06 → Ready to Switch On
            else if (cmd == CW_SHUTDOWN) {
                ax->state = CIA402_READY_TO_SWITCH_ON;
                resetControlState(ax);
            }
            // 0x00 → Switch On Disabled
            else if (cmd == CW_DISABLE_VOLTAGE) {
                ax->state = CIA402_SWITCH_ON_DISABLED;
                resetControlState(ax);
            }
            break;

        // TODO(Fault): Fault 상태 처리
        //   case CIA402_FAULT:
        //       if (cw & 0x80) {              // Fault Reset 비트
        //           if (에러 원인 해소됨) {
        //               ax->fault_code = 0;
        //               ax->state = CIA402_SWITCH_ON_DISABLED;
        //           }
        //       }
        //       break;

        default:
            ax->state = CIA402_SWITCH_ON_DISABLED;
            break;
    }

    ax->prev_controlword = cw;

    // ── Statusword 갱신 ──
    switch (ax->state) {
        case CIA402_SWITCH_ON_DISABLED:
            *ax->od_statusword = SW_SWITCH_ON_DISABLED;  break;
        case CIA402_READY_TO_SWITCH_ON:
            *ax->od_statusword = SW_READY_TO_SWITCH_ON;  break;
        case CIA402_SWITCHED_ON:
            *ax->od_statusword = SW_SWITCHED_ON;         break;
        case CIA402_OPERATION_ENABLED:
            *ax->od_statusword = SW_OPERATION_ENABLED;   break;
        // TODO(Fault): case CIA402_FAULT: *ax->od_statusword = SW_FAULT; break;
        default:
            *ax->od_statusword = SW_SWITCH_ON_DISABLED;  break;
    }

    // ── 운전 모드 반영 (0x6060 → 0x6061) ──
    *ax->od_mode_display = *ax->od_mode;
}

// ── 축 하나의 PID 제어 ──
void controlLoopAxis(MotorAxis* ax, float dt)
{
    // ── 0) CiA 402 상태 머신 먼저 처리 ──
    updateStateMachine(ax);

    // ── 1) 엔코더 읽기 (상태와 무관하게 항상 측정) ──
    uint16_t curr_raw  = (uint16_t)__HAL_TIM_GET_COUNTER(ax->enc_tim);
    int16_t  delta_raw = (int16_t)(curr_raw - ax->prev_enc_raw);
    long     delta     = (long)delta_raw * ax->enc_sign;

    float raw_vel = ((float)delta / ENCODER_CPR) * WHEEL_CIRCUM / dt;
    ax->actual_vel = VEL_FILTER * ax->actual_vel
                   + (1.0f - VEL_FILTER) * raw_vel;

    // 실제속도는 항상 보고 (상태와 무관)
    *ax->od_actual = (int32_t)(ax->actual_vel * 1000.0f);
    ax->prev_enc_raw = curr_raw;
    ax->dbg_delta = delta;

    // TODO(Fault): 센서 추가 시 여기서 감지
    //   ax->current_a = readCurrentSensor(ax);
    //   if (ax->current_a > MAX_CURRENT) {
    //       ax->state = CIA402_FAULT;
    //       ax->fault_code = 0x2310;     // CiA 표준 과전류 코드
    //       setMotorAxis(ax, 0);
    //       return;
    //   }

    // ── 2) Operation Enabled 아니면 모터 정지 (게이트) ──
    if (ax->state != CIA402_OPERATION_ENABLED) {
        setMotorAxis(ax, 0);
        ax->integral   = 0.0f;
        ax->prev_error = 0.0f;
        ax->dbg_pwm_f  = 0.0f;
        ax->dbg_target = 0.0f;
        ax->dbg_error  = 0.0f;
        ax->dbg_ff     = 0.0f;
        return;      // 목표속도를 무시함
    }

    // ── 3) 운전 모드 확인 (Profile Velocity만 지원) ──
    if (*ax->od_mode != MODE_PROFILE_VELOCITY) {
        setMotorAxis(ax, 0);
        ax->dbg_pwm_f = 0.0f;
        return;
    }

    // ── 4) 목표속도 읽기 ──
    float target_vel = (float)(*ax->od_target) / 1000.0f;

    // TODO(Profile): 가속/감속 프로파일 적용
    //   float accel = OD_RAM.x6083_profileAcceleration / 1000.0f;
    //   램프 처리로 profile_target을 target_vel 쪽으로 서서히 이동
    //   target_vel = ax->profile_target;

    // TODO(QuickStop): Quick Stop 상태면 감속 프로파일로 target_vel 대체

    if (target_vel >  MAX_TARGET_VEL) target_vel =  MAX_TARGET_VEL;
    if (target_vel < -MAX_TARGET_VEL) target_vel = -MAX_TARGET_VEL;

    // ── 5) 목표 0이면 정지 ──
    if (target_vel > -0.001f && target_vel < 0.001f) {
        setMotorAxis(ax, 0);
        ax->integral   = 0.0f;
        ax->prev_error = 0.0f;
        ax->dbg_pwm_f  = 0.0f;
        ax->dbg_target = 0.0f;
        ax->dbg_error  = 0.0f;
        ax->dbg_ff     = 0.0f;
        return;
    }

    // ── 6) PID 계산 ──
    float error = target_vel - ax->actual_vel;

    ax->integral += error * dt;
    float int_limit = INTEGRAL_MAX / KI;
    if (ax->integral >  int_limit) ax->integral =  int_limit;
    if (ax->integral < -int_limit) ax->integral = -int_limit;

    float d_term = (error - ax->prev_error) / dt;
    float ff     = feedforward(target_vel);

    float pwm_f = ff + KP * error + KI * ax->integral + KD * d_term;

    ax->dbg_pwm_f  = pwm_f;
    ax->dbg_target = target_vel;
    ax->dbg_error  = error;
    ax->dbg_ff     = ff;

    // ── 7) 모터 구동 ──
    setMotorAxis(ax, (int)pwm_f);

    ax->prev_error = error;
}

// ── 축 초기 설정 ──
void initAxes(void)
{
    // ── 오른쪽 (축1, 0x60xx) ──
    axis_r.enc_tim         = &htim3;
    axis_r.pwm_tim         = &htim1;
    axis_r.pwm_channel     = TIM_CHANNEL_1;
    axis_r.dir_port        = GPIOB;
    axis_r.dir_pin         = GPIO_PIN_0;
    axis_r.enc_sign        = -1;
    axis_r.dir_invert      = 0;
    axis_r.od_controlword  = &OD_RAM.x6040_controlword;
    axis_r.od_statusword   = &OD_RAM.x6041_statusword;
    axis_r.od_mode         = &OD_RAM.x6060_modesOfOperation;
    axis_r.od_mode_display = &OD_RAM.x6061_modesOfOperationDisplay;
    axis_r.od_target       = &OD_RAM.x60FF_targetVelocity;
    axis_r.od_actual       = &OD_RAM.x606C_velocityActualValue;

    // ── 왼쪽 (축2, 0x68xx) ──
    axis_l.enc_tim         = &htim4;
    axis_l.pwm_tim         = &htim1;
    axis_l.pwm_channel     = TIM_CHANNEL_2;
    axis_l.dir_port        = GPIOB;
    axis_l.dir_pin         = GPIO_PIN_1;
    axis_l.enc_sign        = +1;
    axis_l.dir_invert      = 0;
    axis_l.od_controlword  = &OD_RAM.x6840_controlwordAxis2;
    axis_l.od_statusword   = &OD_RAM.x6841_statuswordAxis2;
    axis_l.od_mode         = &OD_RAM.x6860_modesOfOperationAxis2;
    axis_l.od_mode_display = &OD_RAM.x6861_modesOfOperationDisplayAxis2;
    axis_l.od_target       = &OD_RAM.x68FF_targetVelocityAxis2;
    axis_l.od_actual       = &OD_RAM.x686C_velocityActualValueAxis2;

    // 상태 초기화
    axis_r.state = axis_l.state = CIA402_SWITCH_ON_DISABLED;
    axis_r.prev_controlword = axis_l.prev_controlword = 0;

    axis_r.prev_enc_raw = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    axis_l.prev_enc_raw = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
    axis_r.actual_vel = axis_l.actual_vel = 0.0f;
    axis_r.integral   = axis_l.integral   = 0.0f;
    axis_r.prev_error = axis_l.prev_error = 0.0f;

    // Statusword 초기값
    OD_RAM.x6041_statusword      = SW_SWITCH_ON_DISABLED;
    OD_RAM.x6841_statuswordAxis2 = SW_SWITCH_ON_DISABLED;
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
  MX_CAN1_Init();
  MX_TIM10_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  CANopenNodeSTM32 canOpenNodeSTM32;
  canOpenNodeSTM32.CANHandle = &hcan1;
  canOpenNodeSTM32.HWInitFunction = MX_CAN1_Init;
  canOpenNodeSTM32.timerHandle = &htim10;
  canOpenNodeSTM32.desiredNodeID = 1;
  canOpenNodeSTM32.baudrate = 500;
  canopen_app_init(&canOpenNodeSTM32);

  // 엔코더 시작 (좌우)
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

  // PWM 시작 (좌우)
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

  // 축 초기화
  initAxes();
  prev_control_ms = HAL_GetTick();
  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED2);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	canopen_app_process();

	uint32_t now = HAL_GetTick();
	if (now - prev_control_ms >= CONTROL_PERIOD_MS) {
		float dt = (now - prev_control_ms) / 1000.0f;
	    prev_control_ms = now;

	    controlLoopAxis(&axis_r, dt);
	    controlLoopAxis(&axis_l, dt);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
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
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
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
  htim1.Init.Prescaler = 87;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 255;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 8399;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 9;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

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
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM10) {
        canopen_app_interrupt();
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
