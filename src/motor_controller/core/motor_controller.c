/**
 * @file motor_controller.c
 * @brief Motor Controller Implementation (STM32F103C8T6)
 * @author AKS Development Team
 * @date 2025
 * @version 1.0.0
 */

#include "motor_controller.h"
#include "aks_pin_definitions.h"

#ifdef USE_HAL_DRIVER
#include "stm32f4xx_hal.h"
#endif

/* Private variables */
static motor_controller_state_t motor_state = MOTOR_STATE_INIT;
static motor_controller_config_t motor_config;
static motor_foc_params_t foc_params;
static motor_measurements_t measurements;
static motor_protection_t protection_status;

/* FOC Control Variables */
static float pid_speed_kp = 3.5f;
static float pid_speed_ki = 0.2f;
static float pid_speed_kd = 0.8f;
static float pid_speed_integral = 0.0f;
static float pid_speed_prev_error = 0.0f;

/* PWM and Timing */
static TIM_HandleTypeDef htim1;     /* Main PWM timer */
static TIM_HandleTypeDef htim3;     /* Hall sensor timer */
static ADC_HandleTypeDef hadc1;     /* Current measurements */
static CAN_HandleTypeDef hcan1;     /* CAN communication */

/* Hall sensor state */
static uint8_t hall_state = 0;
static uint8_t hall_sequence[6] = {0x05, 0x01, 0x03, 0x02, 0x06, 0x04};
static uint8_t commutation_step = 0;

/* Current measurements */
static uint16_t adc_values[3];      /* U, V, W phase currents */
static float phase_currents[3];    /* Converted currents in A */

/* Private function prototypes */
static aks_result_t init_gpio(void);
static aks_result_t init_pwm(void);
static aks_result_t init_adc(void);
static aks_result_t init_can(void);
static aks_result_t init_hall_sensors(void);
static void update_hall_state(void);
static void foc_control_loop(void);
static void six_step_commutation(void);
static void update_current_measurements(void);
static void protection_check(void);
static void send_status_message(void);
static float pi_controller(float reference, float feedback, float kp, float ki, float* integral);

/**
 * @brief Initialize Motor Controller
 * @param config Pointer to motor configuration
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t motor_controller_init(const motor_controller_config_t* config)
{
    if (config == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    motor_config = *config;
    
    /* Initialize HAL */
#ifdef USE_HAL_DRIVER
    HAL_Init();
    
    /* Configure system clock */
    SystemClock_Config();
#endif
    
    /* Initialize subsystems */
    aks_result_t result;
    
    result = init_gpio();
    if (result != AKS_OK) return result;
    
    result = init_pwm();
    if (result != AKS_OK) return result;
    
    result = init_adc();
    if (result != AKS_OK) return result;
    
    result = init_hall_sensors();
    if (result != AKS_OK) return result;
    
    result = init_can();
    if (result != AKS_OK) return result;
    
    /* Initialize FOC parameters */
    foc_params.target_speed = 0.0f;
    foc_params.actual_speed = 0.0f;
    foc_params.target_torque = 0.0f;
    foc_params.id_reference = 0.0f;
    foc_params.iq_reference = 0.0f;
    foc_params.vd_output = 0.0f;
    foc_params.vq_output = 0.0f;
    foc_params.electrical_angle = 0.0f;
    
    /* Initialize protection */
    protection_status.overcurrent = false;
    protection_status.overvoltage = false;
    protection_status.overtemperature = false;
    protection_status.hall_fault = false;
    protection_status.fault_active = false;
    
    motor_state = MOTOR_STATE_READY;
    
    return AKS_OK;
}

/**
 * @brief Start Motor Controller
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t motor_controller_start(void)
{
    if (motor_state != MOTOR_STATE_READY) {
        return AKS_ERROR_NOT_READY;
    }
    
    /* Enable PWM outputs */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    
    /* Start ADC conversions */
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_values, 3);
    
    /* Enable motor */
    HAL_GPIO_WritePin(GPIOA, MOTOR_ENABLE_PIN, GPIO_PIN_SET);
    
    motor_state = MOTOR_STATE_RUNNING;
    
    return AKS_OK;
}

/**
 * @brief Stop Motor Controller
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t motor_controller_stop(void)
{
    /* Disable motor */
    HAL_GPIO_WritePin(GPIOA, MOTOR_ENABLE_PIN, GPIO_PIN_RESET);
    
    /* Stop PWM outputs */
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    
    /* Stop ADC */
    HAL_ADC_Stop_DMA(&hadc1);
    
    motor_state = MOTOR_STATE_READY;
    
    return AKS_OK;
}

/**
 * @brief Motor controller task - should be called at 1kHz
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t motor_controller_task(void)
{
    if (motor_state != MOTOR_STATE_RUNNING) {
        return AKS_ERROR_NOT_READY;
    }
    
    /* Update measurements */
    update_current_measurements();
    update_hall_state();
    
    /* Protection checks */
    protection_check();
    
    if (protection_status.fault_active) {
        motor_controller_emergency_stop();
        return AKS_ERROR;
    }
    
    /* Control algorithm */
    if (motor_config.control_mode == MOTOR_CONTROL_FOC) {
        foc_control_loop();
    } else {
        six_step_commutation();
    }
    
    /* Send status periodically */
    static uint32_t last_status_time = 0;
    uint32_t current_time = HAL_GetTick();
    if ((current_time - last_status_time) >= 100) {  /* 10Hz */
        send_status_message();
        last_status_time = current_time;
    }
    
    return AKS_OK;
}

/**
 * @brief Set motor speed reference
 * @param speed_rpm Speed in RPM
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t motor_controller_set_speed(float speed_rpm)
{
    if (speed_rpm < 0 || speed_rpm > motor_config.max_speed) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    foc_params.target_speed = speed_rpm;
    
    return AKS_OK;
}

/**
 * @brief Set motor torque reference
 * @param torque_nm Torque in Nm
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t motor_controller_set_torque(float torque_nm)
{
    if (torque_nm < 0 || torque_nm > motor_config.max_torque) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    foc_params.target_torque = torque_nm;
    
    return AKS_OK;
}

/**
 * @brief Emergency stop motor
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t motor_controller_emergency_stop(void)
{
    /* Immediately disable motor */
    HAL_GPIO_WritePin(GPIOA, MOTOR_ENABLE_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, MOTOR_BRAKE_PIN, GPIO_PIN_SET);
    
    /* Set all PWM to zero */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    
    /* Reset control parameters */
    foc_params.target_speed = 0.0f;
    foc_params.target_torque = 0.0f;
    pid_speed_integral = 0.0f;
    
    motor_state = MOTOR_STATE_FAULT;
    
    return AKS_OK;
}

/**
 * @brief Get motor status
 * @param status Pointer to status structure
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t motor_controller_get_status(motor_controller_status_t* status)
{
    if (status == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    status->state = motor_state;
    status->actual_speed = foc_params.actual_speed;
    status->target_speed = foc_params.target_speed;
    status->actual_torque = foc_params.iq_reference * motor_config.torque_constant;
    status->dc_bus_voltage = measurements.dc_bus_voltage;
    status->motor_temperature = measurements.motor_temperature;
    status->phase_current_rms = measurements.phase_current_rms;
    status->hall_state = hall_state;
    status->protection_status = protection_status;
    
    return AKS_OK;
}

/* Private function implementations */

/**
 * @brief Initialize GPIO pins
 */
static aks_result_t init_gpio(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    /* Enable GPIO clocks */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    /* Configure PWM pins (Timer 1) */
    GPIO_InitStruct.Pin = MOTOR_PWM_UH_PIN | MOTOR_PWM_VH_PIN | MOTOR_PWM_WH_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = MOTOR_PWM_UL_PIN | MOTOR_PWM_VL_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    /* Configure Hall sensor pins */
    GPIO_InitStruct.Pin = MOTOR_HALL_A_PIN | MOTOR_HALL_B_PIN | MOTOR_HALL_C_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    /* Configure control pins */
    GPIO_InitStruct.Pin = MOTOR_ENABLE_PIN | MOTOR_BRAKE_PIN | MOTOR_FAULT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    return AKS_OK;
}

/**
 * @brief Initialize PWM timer
 */
static aks_result_t init_pwm(void)
{
    __HAL_RCC_TIM1_CLK_ENABLE();
    
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;
    htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
    htim1.Init.Period = 3600;  /* 20kHz PWM frequency */
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
        return AKS_ERROR;
    }
    
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK ||
        HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
        return AKS_ERROR;
    }
    
    return AKS_OK;
}

/**
 * @brief Initialize ADC for current measurements
 */
static aks_result_t init_adc(void)
{
    __HAL_RCC_ADC1_CLK_ENABLE();
    
    hadc1.Instance = ADC1;
    hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadc1.Init.ContinuousConvMode = ENABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 3;
    
    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        return AKS_ERROR;
    }
    
    /* Configure channels for phase currents */
    ADC_ChannelConfTypeDef sConfig = {0};
    
    sConfig.Channel = ADC_CHANNEL_0;  /* Phase U current */
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    
    sConfig.Channel = ADC_CHANNEL_1;  /* Phase V current */
    sConfig.Rank = 2;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    
    sConfig.Channel = ADC_CHANNEL_2;  /* Phase W current */
    sConfig.Rank = 3;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    
    return AKS_OK;
}

/**
 * @brief Initialize CAN communication
 */
static aks_result_t init_can(void)
{
    __HAL_RCC_CAN1_CLK_ENABLE();
    
    hcan1.Instance = CAN1;
    hcan1.Init.Prescaler = 8;
    hcan1.Init.Mode = CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1 = CAN_BS1_6TQ;
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
    CAN_FilterTypeDef sFilterConfig;
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
 * @brief Initialize Hall sensor interface
 */
static aks_result_t init_hall_sensors(void)
{
    /* Read initial Hall state */
    update_hall_state();
    
    return AKS_OK;
}

/**
 * @brief Update Hall sensor state
 */
static void update_hall_state(void)
{
    uint8_t new_hall_state = 0;
    
    if (HAL_GPIO_ReadPin(GPIOA, MOTOR_HALL_A_PIN)) new_hall_state |= 0x01;
    if (HAL_GPIO_ReadPin(GPIOA, MOTOR_HALL_B_PIN)) new_hall_state |= 0x02;
    if (HAL_GPIO_ReadPin(GPIOA, MOTOR_HALL_C_PIN)) new_hall_state |= 0x04;
    
    if (new_hall_state != hall_state) {
        hall_state = new_hall_state;
        
        /* Find commutation step */
        for (int i = 0; i < 6; i++) {
            if (hall_sequence[i] == hall_state) {
                commutation_step = i;
                break;
            }
        }
        
        /* Calculate electrical angle */
        foc_params.electrical_angle = (commutation_step * 60.0f) * (M_PI / 180.0f);
    }
}

/**
 * @brief FOC control loop
 */
static void foc_control_loop(void)
{
    /* Speed control loop */
    float speed_error = foc_params.target_speed - foc_params.actual_speed;
    foc_params.iq_reference = pi_controller(foc_params.target_speed, foc_params.actual_speed, 
                                           pid_speed_kp, pid_speed_ki, &pid_speed_integral);
    
    /* Limit current reference */
    if (foc_params.iq_reference > motor_config.max_current) {
        foc_params.iq_reference = motor_config.max_current;
    }
    if (foc_params.iq_reference < -motor_config.max_current) {
        foc_params.iq_reference = -motor_config.max_current;
    }
    
    /* Clarke and Park transformations would go here */
    /* For simplicity, using direct PWM control based on Hall sensors */
    six_step_commutation();
}

/**
 * @brief Six-step commutation
 */
static void six_step_commutation(void)
{
    uint16_t pwm_value = (uint16_t)(foc_params.iq_reference * 1800.0f / motor_config.max_current + 1800.0f);
    
    /* Commutation table based on Hall state */
    switch (commutation_step) {
        case 0: /* Hall state 5 (101) */
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_value);  /* U+ */
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);          /* V0 */
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);          /* W- */
            break;
        case 1: /* Hall state 1 (001) */
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_value);  /* U+ */
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);          /* V- */
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);          /* W0 */
            break;
        case 2: /* Hall state 3 (011) */
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);          /* U0 */
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm_value);  /* V+ */
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);          /* W- */
            break;
        case 3: /* Hall state 2 (010) */
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);          /* U- */
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm_value);  /* V+ */
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);          /* W0 */
            break;
        case 4: /* Hall state 6 (110) */
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);          /* U- */
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);          /* V0 */
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm_value);  /* W+ */
            break;
        case 5: /* Hall state 4 (100) */
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);          /* U0 */
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);          /* V- */
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm_value);  /* W+ */
            break;
        default:
            /* Invalid Hall state - stop motor */
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
            protection_status.hall_fault = true;
            break;
    }
}

/**
 * @brief Update current measurements
 */
static void update_current_measurements(void)
{
    /* Convert ADC values to actual currents */
    for (int i = 0; i < 3; i++) {
        float voltage = (adc_values[i] * 3.3f) / 4096.0f;
        phase_currents[i] = (voltage - 1.65f) / 0.01f;  /* ACS758: 10mV/A, 1.65V offset */
    }
    
    /* Calculate RMS current */
    float current_sum_squares = phase_currents[0] * phase_currents[0] +
                               phase_currents[1] * phase_currents[1] +
                               phase_currents[2] * phase_currents[2];
    measurements.phase_current_rms = sqrtf(current_sum_squares / 3.0f);
    
    /* Update other measurements */
    measurements.dc_bus_voltage = 72.0f;  /* Should be measured */
    measurements.motor_temperature = 25.0f;  /* Should be measured */
}

/**
 * @brief Protection checks
 */
static void protection_check(void)
{
    /* Overcurrent protection */
    if (measurements.phase_current_rms > motor_config.max_current) {
        protection_status.overcurrent = true;
        protection_status.fault_active = true;
    }
    
    /* Overtemperature protection */
    if (measurements.motor_temperature > 85.0f) {
        protection_status.overtemperature = true;
        protection_status.fault_active = true;
    }
    
    /* Overvoltage protection */
    if (measurements.dc_bus_voltage > 80.0f) {
        protection_status.overvoltage = true;
        protection_status.fault_active = true;
    }
}

/**
 * @brief Send status message via CAN
 */
static void send_status_message(void)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox;
    
    TxHeader.StdId = CAN_ID_MOTOR_STATUS;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = 8;
    
    /* Pack status data */
    TxData[0] = (uint8_t)motor_state;
    TxData[1] = (uint8_t)(foc_params.actual_speed / 10.0f);  /* Speed in 10 RPM units */
    TxData[2] = (uint8_t)(measurements.phase_current_rms);    /* Current in A */
    TxData[3] = (uint8_t)measurements.motor_temperature;      /* Temperature in °C */
    TxData[4] = hall_state;
    TxData[5] = protection_status.fault_active ? 0xFF : 0x00;
    TxData[6] = 0x00; /* Reserved */
    TxData[7] = 0x00; /* Reserved */
    
    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
}

/**
 * @brief PI Controller implementation
 */
static float pi_controller(float reference, float feedback, float kp, float ki, float* integral)
{
    float error = reference - feedback;
    *integral += error;
    
    /* Anti-windup */
    if (*integral > 25.0f) *integral = 25.0f;
    if (*integral < -25.0f) *integral = -25.0f;
    
    return kp * error + ki * (*integral);
}

/**
 * @brief System clock configuration for STM32F103C8T6
 */
void SystemClock_Config(void)
{
#ifdef USE_HAL_DRIVER
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }
    
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
#endif
}

/**
 * @brief Error handler
 */
void Error_Handler(void)
{
    __disable_irq();
    while (1) {
        /* Error handling */
    }
}