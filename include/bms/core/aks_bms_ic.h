/**
 * @file aks_bms_ic.h
 * @brief AKS Battery Monitoring IC Interface
 * @version 1.0.0
 * @date 2025-01-27
 * 
 * @copyright Copyright (c) 2025 AKS Development Team
 * 
 * This file provides interface functions for battery monitoring ICs
 * compatible with LTC6811-1 and similar devices.
 */

#ifndef AKS_BMS_IC_H
#define AKS_BMS_IC_H

#ifdef __cplusplus
extern "C" {
#endif

/* =============================================================================
 * INCLUDES
 * ============================================================================= */

#include "aks_common.h"

/* =============================================================================
 * DEFINES
 * ============================================================================= */

#define AKS_BMS_IC_MAX_MODULES          8       /**< Maximum number of BMS ICs */
#define AKS_BMS_IC_MAX_CELLS_PER_IC     12      /**< Maximum cells per IC */
#define AKS_BMS_IC_VOLTAGE_RESOLUTION   100e-6f /**< Voltage resolution (100µV) */
#define AKS_BMS_IC_TEMP_CHANNELS        8       /**< Temperature channels per IC */

/* LTC6811-1 Compatible Commands */
#define AKS_BMS_IC_CMD_ADCV             0x0260  /**< Start cell voltage ADC */
#define AKS_BMS_IC_CMD_ADAX             0x0460  /**< Start auxiliary ADC */
#define AKS_BMS_IC_CMD_RDCVA            0x0004  /**< Read cell voltage A */
#define AKS_BMS_IC_CMD_RDCVB            0x0006  /**< Read cell voltage B */
#define AKS_BMS_IC_CMD_RDCVC            0x0008  /**< Read cell voltage C */
#define AKS_BMS_IC_CMD_RDCVD            0x000A  /**< Read cell voltage D */

/* =============================================================================
 * TYPE DEFINITIONS
 * ============================================================================= */

/**
 * @brief BMS IC configuration structure
 */
typedef struct {
    uint8_t ic_address;                 /**< IC address on daisy chain */
    uint8_t num_cells;                  /**< Number of cells monitored */
    uint8_t num_temp_sensors;           /**< Number of temperature sensors */
    bool enable_cell_balancing;         /**< Enable cell balancing */
    bool enable_temperature_monitoring; /**< Enable temperature monitoring */
    uint16_t balance_time_ms;           /**< Balancing time per cycle */
    float voltage_threshold_balance;    /**< Voltage threshold for balancing */
} aks_bms_ic_config_t;

/**
 * @brief BMS IC status structure
 */
typedef struct {
    bool communication_ok;              /**< Communication status */
    bool adc_conversion_complete;       /**< ADC conversion status */
    uint8_t fault_flags;                /**< Fault flags */
    uint32_t last_update_time;          /**< Last successful update */
} aks_bms_ic_status_t;

/* =============================================================================
 * FUNCTION PROTOTYPES
 * ============================================================================= */

/**
 * @brief Initialize BMS IC interface
 * @param config Pointer to configuration structure
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_bms_ic_init(const aks_bms_ic_config_t* config);

/**
 * @brief Deinitialize BMS IC interface
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_bms_ic_deinit(void);

/**
 * @brief Read cell voltage from specific IC and cell
 * @param ic_address IC address (0-7)
 * @param cell_index Cell index (0-11)
 * @param voltage_raw Pointer to store raw voltage value
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_bms_ic_read_cell_voltage(uint8_t ic_address, uint8_t cell_index, uint16_t* voltage_raw);

/**
 * @brief Read all cell voltages from specific IC
 * @param ic_address IC address (0-7)
 * @param voltages Array to store voltage values
 * @param num_cells Number of cells to read
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_bms_ic_read_all_voltages(uint8_t ic_address, uint16_t* voltages, uint8_t num_cells);

/**
 * @brief Read temperature from auxiliary ADC
 * @param ic_address IC address (0-7)
 * @param temp_channel Temperature channel (0-7)
 * @param temp_raw Pointer to store raw temperature value
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_bms_ic_read_temperature(uint8_t ic_address, uint8_t temp_channel, uint16_t* temp_raw);

/**
 * @brief Start cell voltage conversion
 * @param ic_address IC address (0-7), 0xFF for all ICs
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_bms_ic_start_voltage_conversion(uint8_t ic_address);

/**
 * @brief Start auxiliary ADC conversion
 * @param ic_address IC address (0-7), 0xFF for all ICs
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_bms_ic_start_aux_conversion(uint8_t ic_address);

/**
 * @brief Enable/disable cell balancing
 * @param ic_address IC address (0-7)
 * @param cell_mask Bitmask of cells to balance
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_bms_ic_set_cell_balancing(uint8_t ic_address, uint16_t cell_mask);

/**
 * @brief Check if ADC conversion is complete
 * @param ic_address IC address (0-7)
 * @param complete Pointer to store completion status
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_bms_ic_is_conversion_complete(uint8_t ic_address, bool* complete);

/**
 * @brief Get IC status
 * @param ic_address IC address (0-7)
 * @param status Pointer to status structure
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_bms_ic_get_status(uint8_t ic_address, aks_bms_ic_status_t* status);

/**
 * @brief Perform IC self-test
 * @param ic_address IC address (0-7)
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_bms_ic_self_test(uint8_t ic_address);

/**
 * @brief Wake up IC from sleep mode
 * @param ic_address IC address (0-7), 0xFF for all ICs
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_bms_ic_wakeup(uint8_t ic_address);

/**
 * @brief Put IC into sleep mode
 * @param ic_address IC address (0-7), 0xFF for all ICs
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_bms_ic_sleep(uint8_t ic_address);

#ifdef __cplusplus
}
#endif

#endif /* AKS_BMS_IC_H */ 