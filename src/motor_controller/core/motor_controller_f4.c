/**
 * @file motor_controller_f4.c
 * @brief Motor Controller Implementation for STM32F4 (STM32F411RE)
 * @author AKS Development Team
 * @date 2025
 * @version 1.0.0
 */

#include "motor_controller.h"
#include "aks_pin_definitions.h"
#include "aks_can.h"
#include "aks_safety.h"

#ifdef USE_HAL_DRIVER
#include "stm32f4xx_hal.h"
#endif

/* Private variables */
static motor_controller_state_t motor_state = MOTOR_STATE_INIT;
static motor_controller_config_t motor_config;
static motor_foc_params_t foc_params;
static motor_measurements_t measurements;
static motor_security_t security_status;

/* HAL handles for STM32F4 */
static ADC_HandleTypeDef hadc1;
static TIM_HandleTypeDef htim1;  /* PWM generation */
static TIM_HandleTypeDef htim3;  /* Hall sensor capture */
static CAN_HandleTypeDef hcan1;  /* CAN communication */

/* Security and control variables */
static uint32_t last_comm_time = 0;
static uint32_t watchdog_counter = 0;
static bool emergency_brake_active = false;
static float torque_limit_factor = 1.0f;

/* Private function prototypes */
static aks_result_t init_gpio_f4(void);
static aks_result_t init_adc_f4(void);
static aks_result_t init_pwm_f4(void);
static aks_result_t init_can_f4(void);
static aks_result_t init_hall_sensors_f4(void);
static void foc_control_loop_f4(void);
static void update_pwm_outputs_f4(void);
static void process_hall_sensors_f4(void);
static aks_result_t send_status_message_f4(void);
static void apply_security_limits(void);
static void motor_safety_check(void);

/**
 * @brief Initialize Motor Controller for STM32F4
 */
aks_result_t motor_controller_init(const motor_controller_config_t* config)
{
    if (config == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    motor_config = *config;
    motor_state = MOTOR_STATE_INIT;
    
    /* Initialize security status */
    security_status.isolation_ok = true;
    security_status.temperature_ok = true;
    security_status.current_ok = true;
    security_status.voltage_ok = true;
    security_status.communication_ok = true;
    
    /* Initialize GPIO */
    aks_result_t result = init_gpio_f4();
    if (result != AKS_OK) return result;
    
    /* Initialize ADC for current and voltage sensing */
    result = init_adc_f4();
    if (result != AKS_OK) return result;
    
    /* Initialize PWM for motor control */
    result = init_pwm_f4();
    if (result != AKS_OK) return result;
    
    /* Initialize Hall sensor inputs */
    result = init_hall_sensors_f4();
    if (result != AKS_OK) return result;
    
    /* Initialize CAN communication */
    result = init_can_f4();
    if (result != AKS_OK) return result;
    
    /* Initialize FOC parameters */
    foc_params.target_speed = 0.0f;
    foc_params.target_torque = 0.0f;
    foc_params.kp_speed = 1.0f;
    foc_params.ki_speed = 0.1f;
    foc_params.kd_speed = 0.01f;
    foc_params.kp_current = 5.0f;
    foc_params.ki_current = 0.5f;
    
    motor_state = MOTOR_STATE_READY;
    last_comm_time = HAL_GetTick();
    
    return AKS_OK;
}

/**
 * @brief Start Motor Controller
 */
aks_result_t motor_controller_start(void)
{
    if (motor_state != MOTOR_STATE_READY) {
        return AKS_ERROR_NOT_READY;
    }
    
    /* Start PWM generation */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    
    /* Start ADC conversions */
    HAL_ADC_Start(&hadc1);
    
    /* Start Hall sensor capture */
    HAL_TIM_IC_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_IC_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_IC_Start(&htim3, TIM_CHANNEL_3);
    
    motor_state = MOTOR_STATE_RUNNING;
    
    return AKS_OK;
}

/**
 * @brief Motor Controller Main Task
 */
aks_result_t motor_controller_task(void)
{
    if (motor_state != MOTOR_STATE_RUNNING) {
        return AKS_ERROR_NOT_READY;
    }
    
    uint32_t current_time = HAL_GetTick();
    
    /* Safety checks */
    motor_safety_check();
    
    /* Communication timeout check */
    if ((current_time - last_comm_time) > MOTOR_COMM_TIMEOUT_MS) {
        security_status.communication_ok = false;
        foc_params.target_torque = 0.0f;  /* Safe stop */
    }
    
    /* Read sensors */
    process_hall_sensors_f4();
    
    /* Run FOC control loop */
    foc_control_loop_f4();
    
    /* Apply security limits */
    apply_security_limits();
    
    /* Update PWM outputs */
    update_pwm_outputs_f4();
    
    /* Send status message every 10ms */
    static uint32_t last_status_time = 0;
    if ((current_time - last_status_time) >= 10) {
        send_status_message_f4();
        last_status_time = current_time;
    }
    
    /* Increment watchdog */
    watchdog_counter++;
    
    return AKS_OK;
}

/**
 * @brief Set Motor Control Command
 */
aks_result_t motor_controller_set_command(const aks_motor_control_t* command)
{
    if (command == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    last_comm_time = HAL_GetTick();
    security_status.communication_ok = true;
    
    /* Apply security limits to commands */
    foc_params.target_torque = command->torque_request;
    foc_params.target_speed = command->speed_request;
    
    /* Limit torque based on security status */
    if (foc_params.target_torque > motor_config.max_torque * torque_limit_factor) {
        foc_params.target_torque = motor_config.max_torque * torque_limit_factor;
    }
    
    /* Emergency brake override */
    if (emergency_brake_active || !command->enable) {
        foc_params.target_torque = 0.0f;
        foc_params.target_speed = 0.0f;
    }
    
    return AKS_OK;
}

/**
 * @brief Emergency Stop Motor
 */
aks_result_t motor_controller_emergency_stop(void)
{
    emergency_brake_active = true;
    foc_params.target_torque = 0.0f;
    foc_params.target_speed = 0.0f;
    
    /* Disable PWM outputs immediately */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    
    motor_state = MOTOR_STATE_EMERGENCY;
    
    return AKS_OK;
}

/* Private function implementations */

/**
 * @brief Initialize GPIO for STM32F4
 */
static aks_result_t init_gpio_f4(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    /* Enable GPIO clocks */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    
    /* Configure PWM output pins (PA8, PA9, PA10 for TIM1 CH1,2,3) */
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    /* Configure Hall sensor inputs (PB6, PB7, PB8 for TIM3 CH1,2,3) */
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    /* Configure ADC pins (PA0, PA1 for current sensing) */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    return AKS_OK;
}

/**
 * @brief Initialize ADC for STM32F4
 */
static aks_result_t init_adc_f4(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    
    /* Enable ADC clock */
    __HAL_RCC_ADC1_CLK_ENABLE();
    
    /* Configure ADC */
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = ENABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 2;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    
    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        return AKS_ERROR;
    }
    
    /* Configure ADC channels */
    sConfig.Channel = ADC_CHANNEL_0;  /* Current sensor A */
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    
    sConfig.Channel = ADC_CHANNEL_1;  /* Current sensor B */
    sConfig.Rank = 2;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    
    return AKS_OK;
}

/**
 * @brief Initialize PWM for STM32F4
 */
static aks_result_t init_pwm_f4(void)
{
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    
    /* Enable TIM1 clock */
    __HAL_RCC_TIM1_CLK_ENABLE();
    
    /* Configure TIM1 for PWM generation */
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 4199;  /* 20kHz PWM frequency at 84MHz */
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
        return AKS_ERROR;
    }
    
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);
    
    /* Configure PWM channels */
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3);
    
    return AKS_OK;
}

/**
 * @brief Initialize CAN for STM32F4
 */
static aks_result_t init_can_f4(void)
{
    CAN_FilterTypeDef sFilterConfig;
    
    /* Enable CAN clock */
    __HAL_RCC_CAN1_CLK_ENABLE();
    
    /* Configure CAN */
    hcan1.Instance = CAN1;
    hcan1.Init.Prescaler = 6;
    hcan1.Init.Mode = CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
    hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
    hcan1.Init.TimeTriggeredMode = DISABLE;
    hcan1.Init.AutoBusOff = DISABLE;
    hcan1.Init.AutoWakeUp = DISABLE;
    hcan1.Init.AutoRetransmission = DISABLE;
    hcan1.Init.ReceiveFifoLocked = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;
    
    if (HAL_CAN_Init(&hcan1) != HAL_OK) {
        return AKS_ERROR;
    }
    
    /* Configure CAN filter */
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;
    
    if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
        return AKS_ERROR;
    }
    
    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        return AKS_ERROR;
    }
    
    return AKS_OK;
}

/**
 * @brief Initialize Hall sensors for STM32F4
 */
static aks_result_t init_hall_sensors_f4(void)
{
    TIM_IC_InitTypeDef sConfigIC = {0};
    
    /* Enable TIM3 clock */
    __HAL_RCC_TIM3_CLK_ENABLE();
    
    /* Configure TIM3 for Hall sensor capture */
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 83;  /* 1MHz timer clock */
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 0xFFFF;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    
    if (HAL_TIM_IC_Init(&htim3) != HAL_OK) {
        return AKS_ERROR;
    }
    
    /* Configure input capture channels */
    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 0;
    
    HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1);
    HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2);
    HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_3);
    
    return AKS_OK;
}

/**
 * @brief FOC Control Loop for STM32F4
 */
static void foc_control_loop_f4(void)
{
    /* Read current measurements */
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    uint32_t adc_value_a = HAL_ADC_GetValue(&hadc1);
    
    HAL_ADC_PollForConversion(&hadc1, 10);
    uint32_t adc_value_b = HAL_ADC_GetValue(&hadc1);
    
    /* Convert ADC values to current (simplified) */
    measurements.current_a = (float)(adc_value_a - 2048) * 0.01f;  /* Assuming ±20A range */
    measurements.current_b = (float)(adc_value_b - 2048) * 0.01f;
    measurements.current_c = -(measurements.current_a + measurements.current_b);
    
    /* Calculate actual speed from Hall sensors */
    static uint32_t last_hall_time = 0;
    uint32_t current_time = HAL_GetTick();
    if (current_time != last_hall_time) {
        measurements.speed = foc_params.actual_speed;  /* Simplified */
        last_hall_time = current_time;
    }
    
    /* Speed control loop */
    float speed_error = foc_params.target_speed - measurements.speed;
    static float speed_integral = 0.0f;
    static float speed_prev_error = 0.0f;
    
    speed_integral += speed_error * 0.001f;  /* 1ms sample time */
    float speed_derivative = (speed_error - speed_prev_error) / 0.001f;
    
    /* Clamp integral */
    if (speed_integral > 10.0f) speed_integral = 10.0f;
    if (speed_integral < -10.0f) speed_integral = -10.0f;
    
    float torque_from_speed = foc_params.kp_speed * speed_error + 
                              foc_params.ki_speed * speed_integral + 
                              foc_params.kd_speed * speed_derivative;
    
    /* Use direct torque command or speed-derived torque */
    float target_torque = (foc_params.target_torque != 0.0f) ? 
                          foc_params.target_torque : torque_from_speed;
    
    /* Current control loop (simplified) */
    measurements.torque = target_torque;  /* Direct assignment for now */
    
    speed_prev_error = speed_error;
}

/**
 * @brief Update PWM outputs for STM32F4
 */
static void update_pwm_outputs_f4(void)
{
    if (motor_state != MOTOR_STATE_RUNNING || emergency_brake_active) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
        return;
    }
    
    /* Simplified PWM generation based on torque */
    float duty_cycle = measurements.torque / motor_config.max_torque;
    
    /* Clamp duty cycle */
    if (duty_cycle > 0.95f) duty_cycle = 0.95f;
    if (duty_cycle < -0.95f) duty_cycle = -0.95f;
    
    uint32_t pulse_value = (uint32_t)((duty_cycle + 1.0f) * 2099);  /* 0-4199 range */
    
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse_value);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pulse_value);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pulse_value);
}

/**
 * @brief Process Hall sensors for STM32F4
 */
static void process_hall_sensors_f4(void)
{
    /* Read Hall sensor states */
    bool hall_a = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6);
    bool hall_b = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7);
    bool hall_c = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8);
    
    /* Calculate rotor position (simplified) */
    uint8_t hall_state = (hall_c << 2) | (hall_b << 1) | hall_a;
    
    static uint8_t prev_hall_state = 0;
    static uint32_t hall_transitions = 0;
    
    if (hall_state != prev_hall_state) {
        hall_transitions++;
        prev_hall_state = hall_state;
        
        /* Calculate speed based on transitions (simplified) */
        static uint32_t last_transition_time = 0;
        uint32_t current_time = HAL_GetTick();
        if (current_time > last_transition_time) {
            uint32_t period = current_time - last_transition_time;
            if (period > 0) {
                foc_params.actual_speed = 60000.0f / (period * 6.0f);  /* RPM */
            }
            last_transition_time = current_time;
        }
    }
}

/**
 * @brief Send status message for STM32F4
 */
static aks_result_t send_status_message_f4(void)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox;
    
    TxHeader.StdId = AKS_CAN_ID_MOTOR_STATUS;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = 8;
    
    /* Pack status data */
    TxData[0] = (uint8_t)motor_state;
    TxData[1] = (uint8_t)(measurements.speed / 10.0f);  /* Speed in 10 RPM units */
    TxData[2] = (uint8_t)((measurements.torque + 50.0f) * 2.0f);  /* Torque offset and scale */
    TxData[3] = (uint8_t)(measurements.current_a * 10.0f + 128);  /* Current offset */
    TxData[4] = security_status.isolation_ok ? 0x01 : 0x00;
    TxData[5] = security_status.temperature_ok ? 0x01 : 0x00;
    TxData[6] = (uint8_t)(watchdog_counter & 0xFF);
    TxData[7] = emergency_brake_active ? 0xFF : 0x00;
    
    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
    
    return AKS_OK;
}

/**
 * @brief Apply security limits
 */
static void apply_security_limits(void)
{
    /* Temperature limiting */
    if (measurements.temperature > MOTOR_MAX_TEMP_C) {
        security_status.temperature_ok = false;
        torque_limit_factor = 0.5f;  /* Reduce to 50% */
    } else if (measurements.temperature < (MOTOR_MAX_TEMP_C - 10.0f)) {
        security_status.temperature_ok = true;
        torque_limit_factor = 1.0f;  /* Full power */
    }
    
    /* Current limiting */
    float max_current = fmaxf(fabsf(measurements.current_a), 
                             fmaxf(fabsf(measurements.current_b), 
                                   fabsf(measurements.current_c)));
    
    if (max_current > motor_config.max_current) {
        security_status.current_ok = false;
        foc_params.target_torque *= 0.9f;  /* Reduce torque */
    } else {
        security_status.current_ok = true;
    }
    
    /* Voltage monitoring */
    if (measurements.dc_voltage < MOTOR_MIN_VOLTAGE_V || 
        measurements.dc_voltage > MOTOR_MAX_VOLTAGE_V) {
        security_status.voltage_ok = false;
        emergency_brake_active = true;
    } else {
        security_status.voltage_ok = true;
    }
}

/**
 * @brief Motor safety check
 */
static void motor_safety_check(void)
{
    /* Check for isolation fault */
    if (!aks_safety_get_isolation_status()) {
        security_status.isolation_ok = false;
        motor_controller_emergency_stop();
    }
    
    /* Check for emergency stop */
    if (aks_safety_is_emergency_active()) {
        motor_controller_emergency_stop();
    }
    
    /* Watchdog check */
    static uint32_t last_watchdog_reset = 0;
    uint32_t current_time = HAL_GetTick();
    
    if ((current_time - last_watchdog_reset) > MOTOR_WATCHDOG_TIMEOUT_MS) {
        /* Reset external watchdog */
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);  /* Assuming PC13 is watchdog pin */
        last_watchdog_reset = current_time;
    }
}