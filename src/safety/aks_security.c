/**
 * @file aks_security.c
 * @brief AKS Security and Memory Protection Implementation
 * @author AKS Development Team
 * @date 2025
 * @version 1.0.0
 */

#include "aks_security.h"
#include "aks_types.h"
#include "aks_core.h"

#ifdef USE_HAL_DRIVER
#include "stm32f4xx_hal.h"
#endif

/* Global variables for security system */
IWDG_HandleTypeDef hiwdg;  /* Independent watchdog handle */
uint32_t __stack_chk_guard = 0xDEADBEEF;  /* Stack canary */

/* Security configuration */
static aks_security_config_t security_config;
static aks_security_status_t security_status;
static uint32_t security_violations = 0;
static uint32_t last_heartbeat_time = 0;
static bool security_initialized = false;

/* Memory protection regions */
#define MPU_REGION_COUNT 8
static MPU_Region_InitTypeDef mpu_regions[MPU_REGION_COUNT];

/* Security event log */
#define SECURITY_LOG_SIZE 32
static aks_security_event_t security_log[SECURITY_LOG_SIZE];
static uint8_t security_log_index = 0;

/* Private function prototypes */
static aks_result_t configure_memory_protection(void);
static aks_result_t configure_secure_boot(void);
static aks_result_t validate_flash_integrity(void);
static void log_security_event(aks_security_event_type_t event_type, uint32_t address);
static bool verify_stack_integrity(void);
static aks_result_t enable_hardware_security(void);

/**
 * @brief Initialize AKS Security System
 */
aks_result_t aks_security_init(const aks_security_config_t* config)
{
    if (config == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    security_config = *config;
    
    /* Initialize security status */
    security_status.memory_protection_enabled = false;
    security_status.flash_integrity_ok = false;
    security_status.stack_protection_enabled = false;
    security_status.watchdog_enabled = false;
    security_status.secure_boot_verified = false;
    security_status.violation_count = 0;
    
#ifdef AKS_MEMORY_PROTECTION
    /* Configure Memory Protection Unit (MPU) */
    aks_result_t result = configure_memory_protection();
    if (result != AKS_OK) {
        log_security_event(AKS_SECURITY_EVENT_MPU_INIT_FAIL, 0);
        return result;
    }
    security_status.memory_protection_enabled = true;
#endif

#ifdef AKS_SECURITY_ENABLED
    /* Configure secure boot verification */
    result = configure_secure_boot();
    if (result != AKS_OK) {
        log_security_event(AKS_SECURITY_EVENT_SECURE_BOOT_FAIL, 0);
        return result;
    }
    security_status.secure_boot_verified = true;
    
    /* Validate flash integrity */
    result = validate_flash_integrity();
    if (result != AKS_OK) {
        log_security_event(AKS_SECURITY_EVENT_FLASH_INTEGRITY_FAIL, 0);
        return result;
    }
    security_status.flash_integrity_ok = true;
#endif

#ifdef AKS_WATCHDOG_ENABLED
    /* Enable hardware security features */
    result = enable_hardware_security();
    if (result != AKS_OK) {
        log_security_event(AKS_SECURITY_EVENT_HARDWARE_INIT_FAIL, 0);
        return result;
    }
    security_status.watchdog_enabled = true;
#endif
    
    security_initialized = true;
    last_heartbeat_time = aks_core_get_tick();
    
    log_security_event(AKS_SECURITY_EVENT_SYSTEM_INIT, 0);
    
    return AKS_OK;
}

/**
 * @brief Security monitoring task
 */
aks_result_t aks_security_task(void)
{
    if (!security_initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    uint32_t current_time = aks_core_get_tick();
    
    /* Periodic security checks */
    if ((current_time - last_heartbeat_time) >= AKS_SECURITY_CHECK_INTERVAL_MS) {
        
        /* Verify stack integrity */
        if (!verify_stack_integrity()) {
            security_violations++;
            security_status.violation_count++;
            log_security_event(AKS_SECURITY_EVENT_STACK_OVERFLOW, 0);
            
            if (security_config.auto_recovery_enabled) {
                /* Attempt recovery */
                NVIC_SystemReset();
            }
        }
        
        /* Check flash integrity periodically */
        if (security_config.periodic_flash_check) {
            if (validate_flash_integrity() != AKS_OK) {
                security_violations++;
                security_status.violation_count++;
                log_security_event(AKS_SECURITY_EVENT_FLASH_INTEGRITY_FAIL, 0);
            }
        }
        
        /* Reset watchdog */
        if (security_status.watchdog_enabled) {
            HAL_IWDG_Refresh(&hiwdg);
        }
        
        last_heartbeat_time = current_time;
    }
    
    return AKS_OK;
}

/**
 * @brief Get security status
 */
aks_result_t aks_security_get_status(aks_security_status_t* status)
{
    if (status == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    *status = security_status;
    return AKS_OK;
}

/**
 * @brief Log security violation
 */
aks_result_t aks_security_log_violation(aks_security_event_type_t event_type, uint32_t address)
{
    security_violations++;
    security_status.violation_count++;
    
    log_security_event(event_type, address);
    
    /* Check if violation threshold exceeded */
    if (security_violations > security_config.max_violations) {
        if (security_config.auto_recovery_enabled) {
            /* Force system reset */
            NVIC_SystemReset();
        }
    }
    
    return AKS_OK;
}

/**
 * @brief Get security event log
 */
aks_result_t aks_security_get_log(aks_security_event_t* log, uint8_t* count)
{
    if (log == NULL || count == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    uint8_t entries = (security_log_index < SECURITY_LOG_SIZE) ? 
                      security_log_index : SECURITY_LOG_SIZE;
    
    for (uint8_t i = 0; i < entries; i++) {
        log[i] = security_log[i];
    }
    
    *count = entries;
    return AKS_OK;
}

/* Private function implementations */

/**
 * @brief Configure Memory Protection Unit
 */
static aks_result_t configure_memory_protection(void)
{
#ifdef AKS_MEMORY_PROTECTION
    HAL_MPU_Disable();
    
    /* Region 0: Flash memory (Read-only, Execute) */
    mpu_regions[0].Enable = MPU_REGION_ENABLE;
    mpu_regions[0].Number = MPU_REGION_NUMBER0;
    mpu_regions[0].BaseAddress = 0x08000000;
    mpu_regions[0].Size = MPU_REGION_SIZE_512KB;
    mpu_regions[0].SubRegionDisable = 0x00;
    mpu_regions[0].TypeExtField = MPU_TEX_LEVEL0;
    mpu_regions[0].AccessPermission = MPU_REGION_PRIV_RO;
    mpu_regions[0].DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
    mpu_regions[0].IsShareable = MPU_ACCESS_NOT_SHAREABLE;
    mpu_regions[0].IsCacheable = MPU_ACCESS_CACHEABLE;
    mpu_regions[0].IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
    HAL_MPU_ConfigRegion(&mpu_regions[0]);
    
    /* Region 1: SRAM (Read-write, No execute) */
    mpu_regions[1].Enable = MPU_REGION_ENABLE;
    mpu_regions[1].Number = MPU_REGION_NUMBER1;
    mpu_regions[1].BaseAddress = 0x20000000;
    mpu_regions[1].Size = MPU_REGION_SIZE_128KB;
    mpu_regions[1].SubRegionDisable = 0x00;
    mpu_regions[1].TypeExtField = MPU_TEX_LEVEL1;
    mpu_regions[1].AccessPermission = MPU_REGION_FULL_ACCESS;
    mpu_regions[1].DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
    mpu_regions[1].IsShareable = MPU_ACCESS_NOT_SHAREABLE;
    mpu_regions[1].IsCacheable = MPU_ACCESS_CACHEABLE;
    mpu_regions[1].IsBufferable = MPU_ACCESS_BUFFERABLE;
    HAL_MPU_ConfigRegion(&mpu_regions[1]);
    
    /* Region 2: Stack Guard (No access) */
    mpu_regions[2].Enable = MPU_REGION_ENABLE;
    mpu_regions[2].Number = MPU_REGION_NUMBER2;
    mpu_regions[2].BaseAddress = 0x2001F000;  /* End of SRAM - 4KB */
    mpu_regions[2].Size = MPU_REGION_SIZE_4KB;
    mpu_regions[2].SubRegionDisable = 0x00;
    mpu_regions[2].TypeExtField = MPU_TEX_LEVEL0;
    mpu_regions[2].AccessPermission = MPU_REGION_NO_ACCESS;
    mpu_regions[2].DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
    mpu_regions[2].IsShareable = MPU_ACCESS_NOT_SHAREABLE;
    mpu_regions[2].IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
    mpu_regions[2].IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
    HAL_MPU_ConfigRegion(&mpu_regions[2]);
    
    /* Region 3: Peripheral region (Read-write, No execute) */
    mpu_regions[3].Enable = MPU_REGION_ENABLE;
    mpu_regions[3].Number = MPU_REGION_NUMBER3;
    mpu_regions[3].BaseAddress = 0x40000000;
    mpu_regions[3].Size = MPU_REGION_SIZE_512MB;
    mpu_regions[3].SubRegionDisable = 0x00;
    mpu_regions[3].TypeExtField = MPU_TEX_LEVEL0;
    mpu_regions[3].AccessPermission = MPU_REGION_FULL_ACCESS;
    mpu_regions[3].DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
    mpu_regions[3].IsShareable = MPU_ACCESS_SHAREABLE;
    mpu_regions[3].IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
    mpu_regions[3].IsBufferable = MPU_ACCESS_BUFFERABLE;
    HAL_MPU_ConfigRegion(&mpu_regions[3]);
    
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
    
    security_status.stack_protection_enabled = true;
    
    return AKS_OK;
#else
    return AKS_ERROR_UNSUPPORTED;
#endif
}

/**
 * @brief Configure secure boot verification
 */
static aks_result_t configure_secure_boot(void)
{
#ifdef AKS_SECURITY_ENABLED
    /* Simple boot verification - check for magic number */
    const uint32_t BOOT_MAGIC = 0xDEADBEEF;
    uint32_t* boot_signature = (uint32_t*)(0x08000000 + 0x1FC);  /* End of vector table */
    
    if (*boot_signature != BOOT_MAGIC) {
        return AKS_ERROR;
    }
    
    return AKS_OK;
#else
    return AKS_ERROR_UNSUPPORTED;
#endif
}

/**
 * @brief Validate flash memory integrity
 */
static aks_result_t validate_flash_integrity(void)
{
#ifdef AKS_SECURITY_ENABLED
    /* Simple CRC check of critical flash regions */
    uint32_t calculated_crc = 0;
    uint32_t* flash_start = (uint32_t*)0x08000000;
    uint32_t flash_size = 0x10000;  /* Check first 64KB */
    
    /* Calculate CRC32 of flash region */
    __HAL_RCC_CRC_CLK_ENABLE();
    
    CRC_HandleTypeDef hcrc;
    hcrc.Instance = CRC;
    hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
    hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
    hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
    hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
    hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_WORDS;
    
    if (HAL_CRC_Init(&hcrc) != HAL_OK) {
        return AKS_ERROR;
    }
    
    calculated_crc = HAL_CRC_Calculate(&hcrc, flash_start, flash_size / 4);
    
    /* For now, just store the CRC - in production, compare with stored value */
    static uint32_t stored_crc = 0;
    if (stored_crc == 0) {
        stored_crc = calculated_crc;  /* First run - store reference */
    } else if (calculated_crc != stored_crc) {
        return AKS_ERROR;  /* Flash integrity compromised */
    }
    
    return AKS_OK;
#else
    return AKS_ERROR_UNSUPPORTED;
#endif
}

/**
 * @brief Verify stack integrity
 */
static bool verify_stack_integrity(void)
{
    /* Check stack canary value */
    extern uint32_t __stack_chk_guard;
    static const uint32_t STACK_CANARY = 0xDEADBEEF;
    
    /* Simple stack overflow detection */
    volatile uint32_t stack_marker = 0xCAFEBABE;
    uint32_t* current_sp = (uint32_t*)&stack_marker;
    uint32_t* stack_end = (uint32_t*)0x20000000;  /* Bottom of SRAM */
    
    /* Check if stack pointer is in valid range */
    if (current_sp < stack_end || current_sp > (uint32_t*)0x20020000) {
        return false;
    }
    
    /* Check for stack canary corruption (if enabled) */
    if (__stack_chk_guard != STACK_CANARY) {
        return false;
    }
    
    return true;
}

/**
 * @brief Enable hardware security features
 */
static aks_result_t enable_hardware_security(void)
{
#ifdef AKS_WATCHDOG_ENABLED
    /* Configure Independent Watchdog */
    IWDG_HandleTypeDef hiwdg;
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
    hiwdg.Init.Reload = 4095;  /* ~4 second timeout */
    
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
        return AKS_ERROR;
    }
    
    return AKS_OK;
#else
    return AKS_ERROR_UNSUPPORTED;
#endif
}

/**
 * @brief Log security event
 */
static void log_security_event(aks_security_event_type_t event_type, uint32_t address)
{
    aks_security_event_t* event = &security_log[security_log_index];
    
    event->event_type = event_type;
    event->timestamp = aks_core_get_tick();
    event->address = address;
    event->violation_count = security_violations;
    
    security_log_index = (security_log_index + 1) % SECURITY_LOG_SIZE;
}

/**
 * @brief Memory fault handler override
 */
void MemManage_Handler(void)
{
    /* Log memory management fault */
    aks_security_log_violation(AKS_SECURITY_EVENT_MEMORY_FAULT, SCB->MMFAR);
    
    /* System reset if auto recovery enabled */
    if (security_config.auto_recovery_enabled) {
        NVIC_SystemReset();
    }
    
    while(1) {
        /* Hang if auto recovery disabled */
    }
}

/**
 * @brief Hard fault handler override
 */
void HardFault_Handler(void)
{
    /* Log hard fault */
    aks_security_log_violation(AKS_SECURITY_EVENT_HARD_FAULT, SCB->HFSR);
    
    /* System reset if auto recovery enabled */
    if (security_config.auto_recovery_enabled) {
        NVIC_SystemReset();
    }
    
    while(1) {
        /* Hang if auto recovery disabled */
    }
}

/**
 * @brief Stack overflow detection
 */
void __stack_chk_fail(void)
{
    /* Log stack overflow */
    aks_security_log_violation(AKS_SECURITY_EVENT_STACK_OVERFLOW, 0);
    
    /* System reset */
    NVIC_SystemReset();
}