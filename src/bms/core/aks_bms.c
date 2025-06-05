/**
 * @file aks_bms.c
 * @brief Battery Management System core implementation for AKS
 * @author AKS Development Team
 * @date 2025
 * @version 1.0.0
 */

#include "aks_core.h"
#include "aks_safety.h"
#include <string.h>
#include <math.h>

/* BMS Configuration */
#define AKS_BMS_MAX_CELLS              20      // Maximum number of cells
#define AKS_BMS_MAX_MODULES            4       // Maximum number of modules
#define AKS_BMS_CELLS_PER_MODULE       5       // Cells per module
#define AKS_BMS_NOMINAL_VOLTAGE        3.6f    // Nominal cell voltage
#define AKS_BMS_MAX_CELL_VOLTAGE       4.2f    // Maximum cell voltage
#define AKS_BMS_MIN_CELL_VOLTAGE       2.5f    // Minimum cell voltage
#define AKS_BMS_BALANCE_THRESHOLD      0.05f   // Balance threshold in V
#define AKS_BMS_MAX_TEMP               60.0f   // Maximum temperature
#define AKS_BMS_MIN_TEMP               -20.0f  // Minimum temperature
#define AKS_BMS_MAX_CURRENT            100.0f  // Maximum current in A
#define AKS_BMS_CAPACITY               100.0f  // Battery capacity in Ah

/* BMS States */
typedef enum {
    AKS_BMS_STATE_INIT = 0,
    AKS_BMS_STATE_READY,
    AKS_BMS_STATE_CHARGING,
    AKS_BMS_STATE_DISCHARGING,
    AKS_BMS_STATE_BALANCING,
    AKS_BMS_STATE_FAULT,
    AKS_BMS_STATE_EMERGENCY
} aks_bms_state_t;

/* Cell Data Structure */
typedef struct {
    float voltage;                  /**< Cell voltage in V */
    float temperature;              /**< Cell temperature in °C */
    bool balancing;                 /**< Balancing active */
    uint16_t balance_time;          /**< Balancing time in seconds */
    uint32_t fault_flags;           /**< Cell fault flags */
} aks_bms_cell_t;

/* Module Data Structure */
typedef struct {
    aks_bms_cell_t cells[AKS_BMS_CELLS_PER_MODULE]; /**< Cells in module */
    float module_voltage;           /**< Total module voltage */
    float module_temperature;       /**< Average module temperature */
    float max_cell_voltage;         /**< Maximum cell voltage in module */
    float min_cell_voltage;         /**< Minimum cell voltage in module */
    bool communication_ok;          /**< Communication status */
    uint32_t last_update_time;      /**< Last update timestamp */
} aks_bms_module_t;

/* BMS Configuration Structure */
typedef struct {
    float max_cell_voltage;         /**< Maximum allowed cell voltage */
    float min_cell_voltage;         /**< Minimum allowed cell voltage */
    float balance_threshold;        /**< Voltage difference for balancing */
    float max_temperature;          /**< Maximum temperature */
    float min_temperature;          /**< Minimum temperature */
    float max_charge_current;       /**< Maximum charge current */
    float max_discharge_current;    /**< Maximum discharge current */
    float capacity;                 /**< Battery capacity */
    uint8_t num_modules;            /**< Number of modules */
    uint8_t cells_per_module;       /**< Cells per module */
    uint32_t monitoring_interval;   /**< Monitoring interval in ms */
    bool enable_balancing;          /**< Enable cell balancing */
    bool enable_thermal_management; /**< Enable thermal management */
} aks_bms_config_t;

/* BMS Status Structure */
typedef struct {
    aks_bms_state_t state;          /**< Current BMS state */
    float pack_voltage;             /**< Total pack voltage */
    float pack_current;             /**< Pack current (+ charge, - discharge) */
    float pack_temperature;         /**< Average pack temperature */
    float soc;                      /**< State of charge in % */
    float soh;                      /**< State of health in % */
    float max_cell_voltage;         /**< Maximum cell voltage */
    float min_cell_voltage;         /**< Minimum cell voltage */
    float voltage_delta;            /**< Voltage difference between cells */
    uint8_t balancing_cells;        /**< Number of cells balancing */
    uint32_t fault_flags;           /**< BMS fault flags */
    bool charging_allowed;          /**< Charging allowed flag */
    bool discharging_allowed;       /**< Discharging allowed flag */
} aks_bms_status_t;

/* BMS Handle Structure */
typedef struct {
    bool initialized;               /**< Initialization flag */
    aks_bms_config_t config;        /**< Configuration */
    aks_bms_status_t status;        /**< Current status */
    
    aks_bms_module_t modules[AKS_BMS_MAX_MODULES]; /**< Battery modules */
    
    uint32_t last_monitoring_time;  /**< Last monitoring time */
    uint32_t last_balancing_time;   /**< Last balancing time */
    uint32_t total_energy;          /**< Total energy consumed/generated */
    
    float coulomb_counter;          /**< Coulomb counter for SOC */
    float last_current;             /**< Last current measurement */
    
    bool contactors_closed;         /**< Main contactors status */
    bool precharge_complete;        /**< Precharge status */
    
} aks_bms_handle_t;

/* Global Variables */
static aks_bms_handle_t g_bms_handle = {0};

/* Private Function Prototypes */
static aks_result_t aks_bms_hardware_init(void);
static aks_result_t aks_bms_read_cell_voltages(void);
static aks_result_t aks_bms_read_temperatures(void);
static aks_result_t aks_bms_calculate_pack_values(void);
static aks_result_t aks_bms_update_soc(void);
static aks_result_t aks_bms_check_faults(void);
static aks_result_t aks_bms_balance_cells(void);
static aks_result_t aks_bms_thermal_management(void);
static aks_result_t aks_bms_control_contactors(bool close);
static float aks_bms_calculate_soc_from_voltage(float voltage);
static float aks_bms_calculate_soh(void);

/**
 * @brief Initialize BMS system
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_bms_init(void)
{
    if (g_bms_handle.initialized) {
        return AKS_ERROR_ALREADY_INITIALIZED;
    }
    
    /* Initialize configuration with default values */
    g_bms_handle.config.max_cell_voltage = AKS_BMS_MAX_CELL_VOLTAGE;
    g_bms_handle.config.min_cell_voltage = AKS_BMS_MIN_CELL_VOLTAGE;
    g_bms_handle.config.balance_threshold = AKS_BMS_BALANCE_THRESHOLD;
    g_bms_handle.config.max_temperature = AKS_BMS_MAX_TEMP;
    g_bms_handle.config.min_temperature = AKS_BMS_MIN_TEMP;
    g_bms_handle.config.max_charge_current = AKS_BMS_MAX_CURRENT;
    g_bms_handle.config.max_discharge_current = AKS_BMS_MAX_CURRENT;
    g_bms_handle.config.capacity = AKS_BMS_CAPACITY;
    g_bms_handle.config.num_modules = AKS_BMS_MAX_MODULES;
    g_bms_handle.config.cells_per_module = AKS_BMS_CELLS_PER_MODULE;
    g_bms_handle.config.monitoring_interval = 100; // 100ms
    g_bms_handle.config.enable_balancing = true;
    g_bms_handle.config.enable_thermal_management = true;
    
    /* Initialize status */
    g_bms_handle.status.state = AKS_BMS_STATE_INIT;
    g_bms_handle.status.soc = 50.0f; // Start with 50% SOC
    g_bms_handle.status.soh = 100.0f; // Start with 100% SOH
    g_bms_handle.status.charging_allowed = false;
    g_bms_handle.status.discharging_allowed = false;
    
    /* Clear modules data */
    memset(g_bms_handle.modules, 0, sizeof(g_bms_handle.modules));
    
    /* Initialize hardware */
    aks_result_t result = aks_bms_hardware_init();
    if (result != AKS_OK) {
        return result;
    }
    
    g_bms_handle.initialized = true;
    g_bms_handle.status.state = AKS_BMS_STATE_READY;
    g_bms_handle.last_monitoring_time = aks_core_get_tick();
    g_bms_handle.contactors_closed = false;
    g_bms_handle.precharge_complete = false;
    
    return AKS_OK;
}

/**
 * @brief Deinitialize BMS system
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_bms_deinit(void)
{
    if (!g_bms_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    /* Open contactors */
    aks_bms_control_contactors(false);
    
    /* Stop balancing */
    for (uint8_t module = 0; module < g_bms_handle.config.num_modules; module++) {
        for (uint8_t cell = 0; cell < g_bms_handle.config.cells_per_module; cell++) {
            g_bms_handle.modules[module].cells[cell].balancing = false;
        }
    }
    
    /* Reset state */
    g_bms_handle.initialized = false;
    g_bms_handle.status.state = AKS_BMS_STATE_INIT;
    
    return AKS_OK;
}

/**
 * @brief Start BMS operation
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_bms_start(void)
{
    if (!g_bms_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    if (g_bms_handle.status.state == AKS_BMS_STATE_FAULT) {
        return AKS_ERROR_INVALID_STATE;
    }
    
    /* Perform initial readings */
    aks_bms_read_cell_voltages();
    aks_bms_read_temperatures();
    aks_bms_calculate_pack_values();
    aks_bms_check_faults();
    
    if (g_bms_handle.status.fault_flags == 0) {
        g_bms_handle.status.charging_allowed = true;
        g_bms_handle.status.discharging_allowed = true;
        g_bms_handle.status.state = AKS_BMS_STATE_READY;
    } else {
        g_bms_handle.status.state = AKS_BMS_STATE_FAULT;
        return AKS_ERROR_FAULT;
    }
    
    return AKS_OK;
}

/**
 * @brief Stop BMS operation
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_bms_stop(void)
{
    if (!g_bms_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    /* Open contactors */
    aks_bms_control_contactors(false);
    
    /* Stop balancing */
    for (uint8_t module = 0; module < g_bms_handle.config.num_modules; module++) {
        for (uint8_t cell = 0; cell < g_bms_handle.config.cells_per_module; cell++) {
            g_bms_handle.modules[module].cells[cell].balancing = false;
        }
    }
    
    g_bms_handle.status.charging_allowed = false;
    g_bms_handle.status.discharging_allowed = false;
    g_bms_handle.status.state = AKS_BMS_STATE_READY;
    
    return AKS_OK;
}

/**
 * @brief BMS monitoring task - should be called periodically
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_bms_task(void)
{
    if (!g_bms_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    uint32_t current_time = aks_core_get_tick();
    
    /* Check if it's time for monitoring */
    if ((current_time - g_bms_handle.last_monitoring_time) >= g_bms_handle.config.monitoring_interval) {
        g_bms_handle.last_monitoring_time = current_time;
        
        /* Read all measurements */
        aks_bms_read_cell_voltages();
        aks_bms_read_temperatures();
        aks_bms_calculate_pack_values();
        aks_bms_update_soc();
        aks_bms_check_faults();
        
        /* Perform balancing if enabled */
        if (g_bms_handle.config.enable_balancing) {
            aks_bms_balance_cells();
        }
        
        /* Thermal management */
        if (g_bms_handle.config.enable_thermal_management) {
            aks_bms_thermal_management();
        }
        
        /* Update state based on current operation */
        if (g_bms_handle.status.pack_current > 1.0f) {
            g_bms_handle.status.state = AKS_BMS_STATE_CHARGING;
        } else if (g_bms_handle.status.pack_current < -1.0f) {
            g_bms_handle.status.state = AKS_BMS_STATE_DISCHARGING;
        } else if (g_bms_handle.status.balancing_cells > 0) {
            g_bms_handle.status.state = AKS_BMS_STATE_BALANCING;
        } else if (g_bms_handle.status.fault_flags != 0) {
            g_bms_handle.status.state = AKS_BMS_STATE_FAULT;
        } else {
            g_bms_handle.status.state = AKS_BMS_STATE_READY;
        }
    }
    
    return AKS_OK;
}

/**
 * @brief Get BMS status
 * @param status Pointer to status structure
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_bms_get_status(aks_bms_status_t* status)
{
    if (!g_bms_handle.initialized || status == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    *status = g_bms_handle.status;
    return AKS_OK;
}

/**
 * @brief Set charging enable
 * @param enable Charging enable state
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_bms_set_charging_enable(bool enable)
{
    if (!g_bms_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    if (enable && g_bms_handle.status.fault_flags != 0) {
        return AKS_ERROR_INVALID_STATE;
    }
    
    g_bms_handle.status.charging_allowed = enable;
    
    return AKS_OK;
}

/**
 * @brief Emergency stop BMS
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_bms_emergency_stop(void)
{
    if (!g_bms_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    /* Immediately open contactors */
    aks_bms_control_contactors(false);
    
    /* Disable charging and discharging */
    g_bms_handle.status.charging_allowed = false;
    g_bms_handle.status.discharging_allowed = false;
    
    /* Set emergency state */
    g_bms_handle.status.state = AKS_BMS_STATE_EMERGENCY;
    
    return AKS_OK;
}

/* Private Functions */

/**
 * @brief Initialize BMS hardware
 */
static aks_result_t aks_bms_hardware_init(void)
{
    /* Initialize SPI/I2C for cell monitoring ICs */
    /* Initialize GPIO for contactors and balancing */
    /* Initialize ADC for current and temperature sensing */
    
    return AKS_OK;
}

/**
 * @brief Read cell voltages from all modules
 */
static aks_result_t aks_bms_read_cell_voltages(void)
{
    /* Read voltages from LTC6811 or similar cell monitoring IC */
    
    for (uint8_t module = 0; module < g_bms_handle.config.num_modules; module++) {
        float module_voltage = 0.0f;
        
        for (uint8_t cell = 0; cell < g_bms_handle.config.cells_per_module; cell++) {
            /* Read actual cell voltage via LTC6811-1 or similar BMS IC */
            uint16_t raw_voltage;
            aks_result_t result = aks_bms_ic_read_cell_voltage(module, cell, &raw_voltage);
            
            if (result == AKS_OK) {
                /* Convert raw ADC value to voltage */
                /* LTC6811-1: 16-bit ADC, 0-5V range with 100µV resolution */
                float voltage = (float)raw_voltage * 0.0001f; /* 100µV per LSB */
                
                /* Apply calibration and filtering */
                voltage *= g_bms_handle.calibration_factors[module][cell];
                
                /* Simple low-pass filter to reduce noise */
                float alpha = 0.9f; /* Filter coefficient */
                g_bms_handle.modules[module].cells[cell].voltage = 
                    alpha * g_bms_handle.modules[module].cells[cell].voltage + 
                    (1.0f - alpha) * voltage;
                
                /* Validate voltage range */
                if (voltage < 0.5f || voltage > 5.0f) {
                    g_bms_handle.modules[module].cells[cell].fault_flags |= AKS_BMS_FAULT_VOLTAGE_SENSOR;
                } else {
                    g_bms_handle.modules[module].cells[cell].fault_flags &= ~AKS_BMS_FAULT_VOLTAGE_SENSOR;
                }
                
                module_voltage += g_bms_handle.modules[module].cells[cell].voltage;
            } else {
                /* Communication error with BMS IC */
                g_bms_handle.modules[module].communication_ok = false;
                g_bms_handle.modules[module].cells[cell].fault_flags |= AKS_BMS_FAULT_COMMUNICATION;
                return AKS_ERROR_COMMUNICATION;
            }
        }
        
        g_bms_handle.modules[module].module_voltage = module_voltage;
        g_bms_handle.modules[module].communication_ok = true;
        g_bms_handle.modules[module].last_update_time = aks_core_get_tick();
    }
    
    return AKS_OK;
}

/**
 * @brief Read temperatures from all modules
 */
static aks_result_t aks_bms_read_temperatures(void)
{
    /* Read temperatures from thermistors */
    
    for (uint8_t module = 0; module < g_bms_handle.config.num_modules; module++) {
        float total_temp = 0.0f;
        
        for (uint8_t cell = 0; cell < g_bms_handle.config.cells_per_module; cell++) {
            /* Simulate temperature reading */
            g_bms_handle.modules[module].cells[cell].temperature = 25.0f + (float)(rand() % 20 - 10);
            total_temp += g_bms_handle.modules[module].cells[cell].temperature;
        }
        
        g_bms_handle.modules[module].module_temperature = total_temp / g_bms_handle.config.cells_per_module;
    }
    
    return AKS_OK;
}

/**
 * @brief Calculate pack-level values
 */
static aks_result_t aks_bms_calculate_pack_values(void)
{
    float total_voltage = 0.0f;
    float total_temp = 0.0f;
    float max_voltage = 0.0f;
    float min_voltage = 5.0f;
    uint8_t total_cells = 0;
    
    /* Calculate pack values from all modules */
    for (uint8_t module = 0; module < g_bms_handle.config.num_modules; module++) {
        total_voltage += g_bms_handle.modules[module].module_voltage;
        total_temp += g_bms_handle.modules[module].module_temperature;
        
        for (uint8_t cell = 0; cell < g_bms_handle.config.cells_per_module; cell++) {
            float cell_voltage = g_bms_handle.modules[module].cells[cell].voltage;
            
            if (cell_voltage > max_voltage) {
                max_voltage = cell_voltage;
            }
            
            if (cell_voltage < min_voltage) {
                min_voltage = cell_voltage;
            }
            
            total_cells++;
        }
    }
    
    g_bms_handle.status.pack_voltage = total_voltage;
    g_bms_handle.status.pack_temperature = total_temp / g_bms_handle.config.num_modules;
    g_bms_handle.status.max_cell_voltage = max_voltage;
    g_bms_handle.status.min_cell_voltage = min_voltage;
    g_bms_handle.status.voltage_delta = max_voltage - min_voltage;
    
    /* Read pack current */
    g_bms_handle.status.pack_current = aks_current_read(AKS_CURRENT_SENSOR_BATTERY);
    
    return AKS_OK;
}

/**
 * @brief Update State of Charge
 */
static aks_result_t aks_bms_update_soc(void)
{
    /* Coulomb counting method */
    float current_diff = g_bms_handle.status.pack_current - g_bms_handle.last_current;
    g_bms_handle.coulomb_counter += current_diff * (g_bms_handle.config.monitoring_interval / 1000.0f / 3600.0f);
    g_bms_handle.last_current = g_bms_handle.status.pack_current;
    
    /* Update SOC based on coulomb counting */
    float soc_change = (g_bms_handle.coulomb_counter / g_bms_handle.config.capacity) * 100.0f;
    g_bms_handle.status.soc += soc_change;
    
    /* Clamp SOC to valid range */
    if (g_bms_handle.status.soc > 100.0f) g_bms_handle.status.soc = 100.0f;
    if (g_bms_handle.status.soc < 0.0f) g_bms_handle.status.soc = 0.0f;
    
    /* Reset coulomb counter */
    g_bms_handle.coulomb_counter = 0.0f;
    
    /* Calculate SOH */
    g_bms_handle.status.soh = aks_bms_calculate_soh();
    
    return AKS_OK;
}

/**
 * @brief Check for faults
 */
static aks_result_t aks_bms_check_faults(void)
{
    uint32_t fault_flags = 0;
    
    /* Check cell voltage limits */
    if (g_bms_handle.status.max_cell_voltage > g_bms_handle.config.max_cell_voltage) {
        fault_flags |= (1 << 0); // Overvoltage
    }
    
    if (g_bms_handle.status.min_cell_voltage < g_bms_handle.config.min_cell_voltage) {
        fault_flags |= (1 << 1); // Undervoltage
    }
    
    /* Check temperature limits */
    if (g_bms_handle.status.pack_temperature > g_bms_handle.config.max_temperature) {
        fault_flags |= (1 << 2); // Overtemperature
    }
    
    if (g_bms_handle.status.pack_temperature < g_bms_handle.config.min_temperature) {
        fault_flags |= (1 << 3); // Undertemperature
    }
    
    /* Check current limits */
    if (fabsf(g_bms_handle.status.pack_current) > g_bms_handle.config.max_discharge_current) {
        fault_flags |= (1 << 4); // Overcurrent
    }
    
    /* Check communication */
    for (uint8_t module = 0; module < g_bms_handle.config.num_modules; module++) {
        if (!g_bms_handle.modules[module].communication_ok) {
            fault_flags |= (1 << 5); // Communication fault
        }
    }
    
    g_bms_handle.status.fault_flags = fault_flags;
    
    /* Update charging/discharging allowed flags */
    g_bms_handle.status.charging_allowed = (fault_flags == 0);
    g_bms_handle.status.discharging_allowed = (fault_flags == 0);
    
    return AKS_OK;
}

/**
 * @brief Perform cell balancing
 */
static aks_result_t aks_bms_balance_cells(void)
{
    uint8_t balancing_count = 0;
    
    /* Only balance if voltage delta exceeds threshold */
    if (g_bms_handle.status.voltage_delta > g_bms_handle.config.balance_threshold) {
        
        for (uint8_t module = 0; module < g_bms_handle.config.num_modules; module++) {
            for (uint8_t cell = 0; cell < g_bms_handle.config.cells_per_module; cell++) {
                float cell_voltage = g_bms_handle.modules[module].cells[cell].voltage;
                
                /* Balance cells that are above average */
                if (cell_voltage > (g_bms_handle.status.min_cell_voltage + g_bms_handle.config.balance_threshold)) {
                    g_bms_handle.modules[module].cells[cell].balancing = true;
                    g_bms_handle.modules[module].cells[cell].balance_time++;
                    balancing_count++;
                } else {
                    g_bms_handle.modules[module].cells[cell].balancing = false;
                }
            }
        }
    } else {
        /* Stop all balancing */
        for (uint8_t module = 0; module < g_bms_handle.config.num_modules; module++) {
            for (uint8_t cell = 0; cell < g_bms_handle.config.cells_per_module; cell++) {
                g_bms_handle.modules[module].cells[cell].balancing = false;
            }
        }
    }
    
    g_bms_handle.status.balancing_cells = balancing_count;
    
    return AKS_OK;
}

/**
 * @brief Thermal management
 */
static aks_result_t aks_bms_thermal_management(void)
{
    /* Control cooling fans/pumps based on temperature */
    if (g_bms_handle.status.pack_temperature > 40.0f) {
        /* Turn on cooling */
    } else if (g_bms_handle.status.pack_temperature < 35.0f) {
        /* Turn off cooling */
    }
    
    /* Control heating if temperature too low */
    if (g_bms_handle.status.pack_temperature < 0.0f) {
        /* Turn on heating */
    } else if (g_bms_handle.status.pack_temperature > 5.0f) {
        /* Turn off heating */
    }
    
    return AKS_OK;
}

/**
 * @brief Control main contactors
 */
static aks_result_t aks_bms_control_contactors(bool close)
{
    if (close && g_bms_handle.status.fault_flags == 0) {
        /* Close contactors */
        g_bms_handle.contactors_closed = true;
    } else {
        /* Open contactors */
        g_bms_handle.contactors_closed = false;
    }
    
    return AKS_OK;
}

/**
 * @brief Calculate SOC from voltage
 */
static float aks_bms_calculate_soc_from_voltage(float voltage)
{
    /* Simple linear approximation */
    float soc = ((voltage - AKS_BMS_MIN_CELL_VOLTAGE) / 
                (AKS_BMS_MAX_CELL_VOLTAGE - AKS_BMS_MIN_CELL_VOLTAGE)) * 100.0f;
    
    if (soc > 100.0f) soc = 100.0f;
    if (soc < 0.0f) soc = 0.0f;
    
    return soc;
}

/**
 * @brief Calculate State of Health
 */
static float aks_bms_calculate_soh(void)
{
    /* Simple SOH calculation based on voltage delta */
    float soh = 100.0f - (g_bms_handle.status.voltage_delta * 1000.0f); // Reduce SOH by 1% per 10mV delta
    
    if (soh > 100.0f) soh = 100.0f;
    if (soh < 0.0f) soh = 0.0f;
    
    return soh;
} 