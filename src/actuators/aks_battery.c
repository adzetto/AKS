/**
 * @file aks_battery.c
 * @brief Battery management implementation for AKS
 * @author AKS Development Team
 * @date 2025
 * @version 1.0.0
 */

#include "aks_types.h"
#include "aks_config.h"
#include "aks_adc.h"
#include "aks_current.h"
#include "aks_temperature.h"
#include "aks_math.h"
#include <string.h>
#include <math.h>

/* Battery Configuration */
#define AKS_BATTERY_MAX_CELLS           20          /**< Maximum cells in series */
#define AKS_BATTERY_MAX_MODULES         4           /**< Maximum battery modules */
#define AKS_BATTERY_NOMINAL_VOLTAGE     72.0f       /**< Nominal pack voltage (V) */
#define AKS_BATTERY_NOMINAL_CAPACITY    100.0f      /**< Nominal capacity (Ah) */
#define AKS_BATTERY_CELL_NOMINAL_V      3.6f        /**< Nominal cell voltage (V) */
#define AKS_BATTERY_CELL_MIN_V          2.8f        /**< Minimum cell voltage (V) */
#define AKS_BATTERY_CELL_MAX_V          4.2f        /**< Maximum cell voltage (V) */
#define AKS_BATTERY_BALANCE_THRESHOLD   0.05f       /**< Balance threshold (V) */
#define AKS_BATTERY_UPDATE_RATE_HZ      10          /**< Update rate (Hz) */

/* Battery States */
typedef enum {
    AKS_BATTERY_STATE_IDLE = 0,         /**< Idle state */
    AKS_BATTERY_STATE_CHARGING,         /**< Charging */
    AKS_BATTERY_STATE_DISCHARGING,      /**< Discharging */
    AKS_BATTERY_STATE_BALANCING,        /**< Cell balancing */
    AKS_BATTERY_STATE_PROTECTION,       /**< Protection mode */
    AKS_BATTERY_STATE_FAULT,            /**< Fault state */
    AKS_BATTERY_STATE_MAINTENANCE       /**< Maintenance mode */
} aks_battery_state_t;

/* Battery Cell Information */
typedef struct {
    float voltage;                      /**< Cell voltage (V) */
    float temperature;                  /**< Cell temperature (°C) */
    float soc;                          /**< State of charge (%) */
    float soh;                          /**< State of health (%) */
    uint32_t cycle_count;               /**< Charge cycles */
    float internal_resistance;          /**< Internal resistance (mΩ) */
    bool balancing_active;              /**< Balancing active flag */
    bool fault_detected;                /**< Fault flag */
    uint32_t fault_count;               /**< Fault counter */
} aks_battery_cell_t;

/* Battery Module Information */
typedef struct {
    aks_battery_cell_t cells[AKS_BATTERY_MAX_CELLS / AKS_BATTERY_MAX_MODULES]; /**< Module cells */
    float module_voltage;               /**< Module total voltage (V) */
    float module_current;               /**< Module current (A) */
    float module_temperature;           /**< Module temperature (°C) */
    float min_cell_voltage;             /**< Minimum cell voltage in module */
    float max_cell_voltage;             /**< Maximum cell voltage in module */
    float voltage_imbalance;            /**< Voltage imbalance */
    uint8_t cell_count;                 /**< Number of cells in module */
    bool module_active;                 /**< Module active flag */
    bool thermal_protection;            /**< Thermal protection active */
} aks_battery_module_t;

/* Battery Pack Information */
typedef struct {
    aks_battery_module_t modules[AKS_BATTERY_MAX_MODULES]; /**< Battery modules */
    aks_battery_state_t state;          /**< Battery state */
    float pack_voltage;                 /**< Pack voltage (V) */
    float pack_current;                 /**< Pack current (A) */
    float pack_power;                   /**< Pack power (W) */
    float pack_soc;                     /**< Pack state of charge (%) */
    float pack_soh;                     /**< Pack state of health (%) */
    float pack_capacity;                /**< Pack capacity (Ah) */
    float energy_remaining;             /**< Energy remaining (Wh) */
    float energy_consumed;              /**< Energy consumed (Wh) */
    float min_cell_voltage;             /**< Minimum cell voltage */
    float max_cell_voltage;             /**< Maximum cell voltage */
    float average_cell_voltage;         /**< Average cell voltage */
    float max_temperature;              /**< Maximum temperature */
    float min_temperature;              /**< Minimum temperature */
    float average_temperature;          /**< Average temperature */
    float charge_rate;                  /**< Charge rate (C) */
    float discharge_rate;               /**< Discharge rate (C) */
    uint32_t total_cycles;              /**< Total charge cycles */
    uint32_t last_update_time;          /**< Last update timestamp */
    bool charging_allowed;              /**< Charging allowed flag */
    bool discharging_allowed;           /**< Discharging allowed flag */
    bool balancing_required;            /**< Balancing required flag */
    bool thermal_fault;                 /**< Thermal fault flag */
    bool voltage_fault;                 /**< Voltage fault flag */
    bool current_fault;                 /**< Current fault flag */
} aks_battery_pack_t;

/* Battery Protection Limits */
typedef struct {
    float min_cell_voltage;             /**< Minimum cell voltage limit */
    float max_cell_voltage;             /**< Maximum cell voltage limit */
    float max_pack_voltage;             /**< Maximum pack voltage limit */
    float min_pack_voltage;             /**< Minimum pack voltage limit */
    float max_charge_current;           /**< Maximum charge current */
    float max_discharge_current;        /**< Maximum discharge current */
    float max_temperature;              /**< Maximum temperature limit */
    float min_temperature;              /**< Minimum temperature limit */
    float max_voltage_imbalance;        /**< Maximum voltage imbalance */
    float max_charge_rate;              /**< Maximum charge rate (C) */
    float max_discharge_rate;           /**< Maximum discharge rate (C) */
} aks_battery_limits_t;

/* Static battery pack */
static aks_battery_pack_t g_battery_pack = {0};
static bool g_battery_initialized = false;

/* Battery protection limits */
static const aks_battery_limits_t g_battery_limits = {
    .min_cell_voltage = AKS_BATTERY_CELL_MIN_V,
    .max_cell_voltage = AKS_BATTERY_CELL_MAX_V,
    .max_pack_voltage = 84.0f,          /* 20S * 4.2V */
    .min_pack_voltage = 56.0f,          /* 20S * 2.8V */
    .max_charge_current = 50.0f,        /* 0.5C charge rate */
    .max_discharge_current = 200.0f,    /* 2C discharge rate */
    .max_temperature = 60.0f,
    .min_temperature = -20.0f,
    .max_voltage_imbalance = 0.2f,
    .max_charge_rate = 1.0f,            /* 1C */
    .max_discharge_rate = 3.0f          /* 3C */
};

/* Statistics */
static struct {
    uint32_t total_charge_cycles;
    uint32_t protection_events;
    uint32_t balancing_events;
    float total_energy_charged;
    float total_energy_discharged;
    float peak_power_charge;
    float peak_power_discharge;
} g_battery_stats = {0};

/* Private function prototypes */
static void aks_battery_update_cells(void);
static void aks_battery_update_modules(void);
static void aks_battery_update_pack_status(void);
static void aks_battery_check_protection_limits(void);
static void aks_battery_update_soc(void);
static void aks_battery_update_soh(void);
static void aks_battery_cell_balancing(void);
static float aks_battery_calculate_soc_from_voltage(float voltage);
static float aks_battery_calculate_soh(aks_battery_cell_t* cell);
static bool aks_battery_check_thermal_limits(void);
static void aks_battery_update_energy_calculations(float dt);

/**
 * @brief Initialize battery management system
 */
aks_result_t aks_battery_init(void)
{
    if (g_battery_initialized) {
        return AKS_OK;
    }
    
    /* Clear battery pack structure */
    memset(&g_battery_pack, 0, sizeof(g_battery_pack));
    
    /* Initialize modules and cells */
    for (uint8_t m = 0; m < AKS_BATTERY_MAX_MODULES; m++) {
        aks_battery_module_t* module = &g_battery_pack.modules[m];
        module->cell_count = AKS_BATTERY_MAX_CELLS / AKS_BATTERY_MAX_MODULES;
        module->module_active = true;
        
        for (uint8_t c = 0; c < module->cell_count; c++) {
            aks_battery_cell_t* cell = &module->cells[c];
            cell->voltage = AKS_BATTERY_CELL_NOMINAL_V;
            cell->temperature = 25.0f;
            cell->soc = 50.0f;
            cell->soh = 100.0f;
            cell->cycle_count = 0;
            cell->internal_resistance = 2.0f; /* 2mΩ typical */
            cell->balancing_active = false;
            cell->fault_detected = false;
        }
    }
    
    /* Initialize pack parameters */
    g_battery_pack.state = AKS_BATTERY_STATE_IDLE;
    g_battery_pack.pack_capacity = AKS_BATTERY_NOMINAL_CAPACITY;
    g_battery_pack.pack_soc = 50.0f;
    g_battery_pack.pack_soh = 100.0f;
    g_battery_pack.charging_allowed = true;
    g_battery_pack.discharging_allowed = true;
    
    /* Clear statistics */
    memset(&g_battery_stats, 0, sizeof(g_battery_stats));
    
    g_battery_initialized = true;
    
    return AKS_OK;
}

/**
 * @brief Update battery management system
 */
aks_result_t aks_battery_update(float dt)
{
    if (!g_battery_initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    /* Update cell measurements */
    aks_battery_update_cells();
    
    /* Update module status */
    aks_battery_update_modules();
    
    /* Update pack status */
    aks_battery_update_pack_status();
    
    /* Check protection limits */
    aks_battery_check_protection_limits();
    
    /* Update state of charge */
    aks_battery_update_soc();
    
    /* Update state of health */
    aks_battery_update_soh();
    
    /* Perform cell balancing if needed */
    if (g_battery_pack.balancing_required) {
        aks_battery_cell_balancing();
    }
    
    /* Update energy calculations */
    aks_battery_update_energy_calculations(dt);
    
    /* Update timestamp */
    g_battery_pack.last_update_time = 0; /* Would use HAL_GetTick() */
    
    return AKS_OK;
}

/**
 * @brief Get battery pack voltage
 */
float aks_battery_get_pack_voltage(void)
{
    return g_battery_pack.pack_voltage;
}

/**
 * @brief Get battery pack current
 */
float aks_battery_get_pack_current(void)
{
    return g_battery_pack.pack_current;
}

/**
 * @brief Get battery pack state of charge
 */
float aks_battery_get_soc(void)
{
    return g_battery_pack.pack_soc;
}

/**
 * @brief Get battery pack state of health
 */
float aks_battery_get_soh(void)
{
    return g_battery_pack.pack_soh;
}

/**
 * @brief Get battery pack power
 */
float aks_battery_get_power(void)
{
    return g_battery_pack.pack_power;
}

/**
 * @brief Get remaining energy
 */
float aks_battery_get_remaining_energy(void)
{
    return g_battery_pack.energy_remaining;
}

/**
 * @brief Get battery state
 */
aks_battery_state_t aks_battery_get_state(void)
{
    return g_battery_pack.state;
}

/**
 * @brief Check if charging is allowed
 */
bool aks_battery_is_charging_allowed(void)
{
    return g_battery_pack.charging_allowed;
}

/**
 * @brief Check if discharging is allowed
 */
bool aks_battery_is_discharging_allowed(void)
{
    return g_battery_pack.discharging_allowed;
}

/**
 * @brief Get cell voltage
 */
aks_result_t aks_battery_get_cell_voltage(uint8_t module_id, uint8_t cell_id, float* voltage)
{
    if (!g_battery_initialized || voltage == NULL || 
        module_id >= AKS_BATTERY_MAX_MODULES || 
        cell_id >= (AKS_BATTERY_MAX_CELLS / AKS_BATTERY_MAX_MODULES)) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    *voltage = g_battery_pack.modules[module_id].cells[cell_id].voltage;
    return AKS_OK;
}

/**
 * @brief Get temperature range
 */
aks_result_t aks_battery_get_temperature_range(float* min_temp, float* max_temp, float* avg_temp)
{
    if (!g_battery_initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    if (min_temp != NULL) {
        *min_temp = g_battery_pack.min_temperature;
    }
    
    if (max_temp != NULL) {
        *max_temp = g_battery_pack.max_temperature;
    }
    
    if (avg_temp != NULL) {
        *avg_temp = g_battery_pack.average_temperature;
    }
    
    return AKS_OK;
}

/**
 * @brief Check battery faults
 */
bool aks_battery_has_fault(void)
{
    return (g_battery_pack.thermal_fault || 
            g_battery_pack.voltage_fault || 
            g_battery_pack.current_fault ||
            g_battery_pack.state == AKS_BATTERY_STATE_FAULT);
}

/**
 * @brief Get battery statistics
 */
aks_result_t aks_battery_get_stats(uint32_t* charge_cycles, uint32_t* protection_events,
                                   float* total_energy_charged, float* total_energy_discharged)
{
    if (!g_battery_initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    if (charge_cycles != NULL) {
        *charge_cycles = g_battery_stats.total_charge_cycles;
    }
    
    if (protection_events != NULL) {
        *protection_events = g_battery_stats.protection_events;
    }
    
    if (total_energy_charged != NULL) {
        *total_energy_charged = g_battery_stats.total_energy_charged;
    }
    
    if (total_energy_discharged != NULL) {
        *total_energy_discharged = g_battery_stats.total_energy_discharged;
    }
    
    return AKS_OK;
}

/* Private function implementations */

/**
 * @brief Update cell measurements
 */
static void aks_battery_update_cells(void)
{
    for (uint8_t m = 0; m < AKS_BATTERY_MAX_MODULES; m++) {
        aks_battery_module_t* module = &g_battery_pack.modules[m];
        
        for (uint8_t c = 0; c < module->cell_count; c++) {
            aks_battery_cell_t* cell = &module->cells[c];
            
            /* Read cell voltage from battery monitoring IC (LTC6811-1 compatible) */
            uint16_t cell_voltage_raw;
            float cell_voltage_adc;
            
            /* Calculate BMS IC channel mapping */
            uint8_t ic_address = m / 4; /* 4 modules per IC */
            uint8_t cell_channel = (m % 4) * (module->cell_count) + c;
            
            /* Read voltage from BMS IC via SPI */
            if (aks_bms_read_cell_voltage(ic_address, cell_channel, &cell_voltage_raw) == AKS_OK) {
                /* Convert raw ADC to voltage (LTC6811 has 16-bit resolution, 0-5V range) */
                cell_voltage_adc = (float)cell_voltage_raw * (5.0f / 65535.0f);
                
                /* Apply voltage divider compensation if needed */
                cell->voltage = cell_voltage_adc * AKS_BATTERY_VOLTAGE_SCALE_FACTOR;
                
                /* Validate voltage reading */
                if (cell->voltage < 0.5f || cell->voltage > 5.0f) {
                    cell->fault_detected = true;
                    cell->fault_count++;
                } else {
                    /* Update SOC based on voltage */
                    cell->soc = aks_battery_calculate_soc_from_voltage(cell->voltage);
                    
                    /* Update SOH based on capacity degradation */
                    cell->soh = aks_battery_calculate_soh(cell);
                }
            } else {
                /* Communication fault with BMS IC */
                cell->fault_detected = true;
                cell->fault_count++;
            }
            
            /* Read cell temperature from dedicated thermistor */
            uint16_t temp_raw;
            float temp_voltage;
            uint8_t temp_channel = AKS_ADC_CH_BATTERY_TEMP_BASE + (m * module->cell_count + c);
            
            if (aks_adc_read_channel(temp_channel, &temp_raw, &temp_voltage) == AKS_OK) {
                /* Convert thermistor voltage to temperature using beta equation */
                /* Assuming 10kΩ NTC thermistor with β=3950K */
                if (temp_voltage > 0.1f && temp_voltage < 3.2f) {
                    float resistance = (temp_voltage * 10000.0f) / (3.3f - temp_voltage);
                    float ln_r = logf(resistance / 10000.0f); /* R0 = 10kΩ at 25°C */
                    float temp_kelvin = 1.0f / ((1.0f / 298.15f) + (ln_r / 3950.0f));
                    cell->temperature = temp_kelvin - 273.15f;
                } else {
                    /* Sensor fault */
                    cell->temperature = 85.0f; /* Assume high temp for safety */
                    cell->fault_detected = true;
                }
            } else {
                cell->temperature = 85.0f; /* Default to high temp on comm failure */
                cell->fault_detected = true;
            }
            
            /* Check for cell faults */
            if (cell->voltage < g_battery_limits.min_cell_voltage || 
                cell->voltage > g_battery_limits.max_cell_voltage ||
                cell->temperature > g_battery_limits.max_temperature ||
                cell->temperature < g_battery_limits.min_temperature) {
                
                cell->fault_detected = true;
                cell->fault_count++;
            } else {
                cell->fault_detected = false;
            }
        }
    }
}

/**
 * @brief Update module status
 */
static void aks_battery_update_modules(void)
{
    for (uint8_t m = 0; m < AKS_BATTERY_MAX_MODULES; m++) {
        aks_battery_module_t* module = &g_battery_pack.modules[m];
        
        float module_voltage_sum = 0.0f;
        float module_temp_sum = 0.0f;
        float min_voltage = 10.0f;
        float max_voltage = 0.0f;
        
        for (uint8_t c = 0; c < module->cell_count; c++) {
            aks_battery_cell_t* cell = &module->cells[c];
            
            module_voltage_sum += cell->voltage;
            module_temp_sum += cell->temperature;
            
            if (cell->voltage < min_voltage) {
                min_voltage = cell->voltage;
            }
            if (cell->voltage > max_voltage) {
                max_voltage = cell->voltage;
            }
        }
        
        module->module_voltage = module_voltage_sum;
        module->module_temperature = module_temp_sum / module->cell_count;
        module->min_cell_voltage = min_voltage;
        module->max_cell_voltage = max_voltage;
        module->voltage_imbalance = max_voltage - min_voltage;
        
        /* Check thermal protection */
        if (module->module_temperature > g_battery_limits.max_temperature * 0.9f) {
            module->thermal_protection = true;
        } else {
            module->thermal_protection = false;
        }
        
        /* Get module current (simulated) */
        module->module_current = g_battery_pack.pack_current / AKS_BATTERY_MAX_MODULES;
    }
}

/**
 * @brief Update pack status
 */
static void aks_battery_update_pack_status(void)
{
    float pack_voltage_sum = 0.0f;
    float pack_temp_sum = 0.0f;
    float min_cell_voltage = 10.0f;
    float max_cell_voltage = 0.0f;
    float min_temp = 100.0f;
    float max_temp = -100.0f;
    uint32_t active_cells = 0;
    
    for (uint8_t m = 0; m < AKS_BATTERY_MAX_MODULES; m++) {
        aks_battery_module_t* module = &g_battery_pack.modules[m];
        
        if (module->module_active) {
            pack_voltage_sum += module->module_voltage;
            pack_temp_sum += module->module_temperature;
            active_cells += module->cell_count;
            
            if (module->min_cell_voltage < min_cell_voltage) {
                min_cell_voltage = module->min_cell_voltage;
            }
            if (module->max_cell_voltage > max_cell_voltage) {
                max_cell_voltage = module->max_cell_voltage;
            }
            if (module->module_temperature < min_temp) {
                min_temp = module->module_temperature;
            }
            if (module->module_temperature > max_temp) {
                max_temp = module->module_temperature;
            }
        }
    }
    
    g_battery_pack.pack_voltage = pack_voltage_sum;
    g_battery_pack.min_cell_voltage = min_cell_voltage;
    g_battery_pack.max_cell_voltage = max_cell_voltage;
    g_battery_pack.average_cell_voltage = pack_voltage_sum / active_cells;
    g_battery_pack.min_temperature = min_temp;
    g_battery_pack.max_temperature = max_temp;
    g_battery_pack.average_temperature = pack_temp_sum / AKS_BATTERY_MAX_MODULES;
    
    /* Get pack current */
    aks_current_get_reading(AKS_CURRENT_LOCATION_BATTERY, &g_battery_pack.pack_current);
    
    /* Calculate pack power */
    g_battery_pack.pack_power = g_battery_pack.pack_voltage * g_battery_pack.pack_current;
    
    /* Calculate charge/discharge rates */
    g_battery_pack.charge_rate = fabsf(g_battery_pack.pack_current) / g_battery_pack.pack_capacity;
    g_battery_pack.discharge_rate = g_battery_pack.charge_rate;
    
    /* Check if balancing is required */
    float voltage_spread = g_battery_pack.max_cell_voltage - g_battery_pack.min_cell_voltage;
    g_battery_pack.balancing_required = (voltage_spread > AKS_BATTERY_BALANCE_THRESHOLD);
}

/**
 * @brief Check protection limits
 */
static void aks_battery_check_protection_limits(void)
{
    bool protection_active = false;
    
    /* Check voltage limits */
    if (g_battery_pack.pack_voltage > g_battery_limits.max_pack_voltage ||
        g_battery_pack.pack_voltage < g_battery_limits.min_pack_voltage ||
        g_battery_pack.max_cell_voltage > g_battery_limits.max_cell_voltage ||
        g_battery_pack.min_cell_voltage < g_battery_limits.min_cell_voltage) {
        
        g_battery_pack.voltage_fault = true;
        protection_active = true;
    } else {
        g_battery_pack.voltage_fault = false;
    }
    
    /* Check current limits */
    if (fabsf(g_battery_pack.pack_current) > g_battery_limits.max_discharge_current) {
        g_battery_pack.current_fault = true;
        protection_active = true;
    } else {
        g_battery_pack.current_fault = false;
    }
    
    /* Check thermal limits */
    if (!aks_battery_check_thermal_limits()) {
        g_battery_pack.thermal_fault = true;
        protection_active = true;
    } else {
        g_battery_pack.thermal_fault = false;
    }
    
    /* Update protection flags */
    if (protection_active) {
        g_battery_pack.state = AKS_BATTERY_STATE_PROTECTION;
        g_battery_pack.charging_allowed = false;
        g_battery_pack.discharging_allowed = false;
        g_battery_stats.protection_events++;
    } else {
        if (g_battery_pack.state == AKS_BATTERY_STATE_PROTECTION) {
            g_battery_pack.state = AKS_BATTERY_STATE_IDLE;
        }
        g_battery_pack.charging_allowed = true;
        g_battery_pack.discharging_allowed = true;
    }
}

/**
 * @brief Update state of charge
 */
static void aks_battery_update_soc(void)
{
    /* Calculate SOC based on voltage and current integration */
    float voltage_based_soc = aks_battery_calculate_soc_from_voltage(g_battery_pack.average_cell_voltage);
    
    /* For now, use voltage-based SOC */
    g_battery_pack.pack_soc = voltage_based_soc;
    
    /* Calculate remaining energy */
    g_battery_pack.energy_remaining = (g_battery_pack.pack_soc / 100.0f) * 
                                      g_battery_pack.pack_capacity * 
                                      g_battery_pack.pack_voltage;
}

/**
 * @brief Update state of health
 */
static void aks_battery_update_soh(void)
{
    float total_soh = 0.0f;
    uint32_t cell_count = 0;
    
    for (uint8_t m = 0; m < AKS_BATTERY_MAX_MODULES; m++) {
        aks_battery_module_t* module = &g_battery_pack.modules[m];
        
        for (uint8_t c = 0; c < module->cell_count; c++) {
            total_soh += module->cells[c].soh;
            cell_count++;
        }
    }
    
    g_battery_pack.pack_soh = total_soh / cell_count;
}

/**
 * @brief Perform cell balancing
 */
static void aks_battery_cell_balancing(void)
{
    g_battery_pack.state = AKS_BATTERY_STATE_BALANCING;
    g_battery_stats.balancing_events++;
    
    /* Find cells that need balancing */
    for (uint8_t m = 0; m < AKS_BATTERY_MAX_MODULES; m++) {
        aks_battery_module_t* module = &g_battery_pack.modules[m];
        
        for (uint8_t c = 0; c < module->cell_count; c++) {
            aks_battery_cell_t* cell = &module->cells[c];
            
            /* Enable balancing for cells above average */
            if (cell->voltage > g_battery_pack.average_cell_voltage + AKS_BATTERY_BALANCE_THRESHOLD/2) {
                cell->balancing_active = true;
                /* In real implementation, this would control balancing switches */
            } else {
                cell->balancing_active = false;
            }
        }
    }
}

/**
 * @brief Calculate SOC from voltage
 */
static float aks_battery_calculate_soc_from_voltage(float voltage)
{
    /* Simple linear approximation */
    float soc = ((voltage - AKS_BATTERY_CELL_MIN_V) / 
                 (AKS_BATTERY_CELL_MAX_V - AKS_BATTERY_CELL_MIN_V)) * 100.0f;
    
    /* Clamp to 0-100% */
    if (soc < 0.0f) soc = 0.0f;
    if (soc > 100.0f) soc = 100.0f;
    
    return soc;
}

/**
 * @brief Calculate state of health
 */
static float aks_battery_calculate_soh(aks_battery_cell_t* cell)
{
    /* Simple SOH calculation based on cycle count and internal resistance */
    float cycle_degradation = (float)cell->cycle_count * 0.01f; /* 1% per 100 cycles */
    float resistance_degradation = (cell->internal_resistance - 2.0f) * 10.0f; /* Normalized */
    
    float soh = 100.0f - cycle_degradation - resistance_degradation;
    
    /* Clamp to 0-100% */
    if (soh < 0.0f) soh = 0.0f;
    if (soh > 100.0f) soh = 100.0f;
    
    return soh;
}

/**
 * @brief Check thermal limits
 */
static bool aks_battery_check_thermal_limits(void)
{
    return (g_battery_pack.max_temperature <= g_battery_limits.max_temperature &&
            g_battery_pack.min_temperature >= g_battery_limits.min_temperature);
}

/**
 * @brief Update energy calculations
 */
static void aks_battery_update_energy_calculations(float dt)
{
    float power = g_battery_pack.pack_power;
    float energy_delta = power * dt / 3600.0f; /* Convert to Wh */
    
    if (power > 0.0f) {
        /* Discharging */
        g_battery_pack.energy_consumed += energy_delta;
        g_battery_stats.total_energy_discharged += energy_delta;
        
        if (power > g_battery_stats.peak_power_discharge) {
            g_battery_stats.peak_power_discharge = power;
        }
    } else {
        /* Charging */
        g_battery_stats.total_energy_charged += fabsf(energy_delta);
        
        if (fabsf(power) > g_battery_stats.peak_power_charge) {
            g_battery_stats.peak_power_charge = fabsf(power);
        }
    }
}