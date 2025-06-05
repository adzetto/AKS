/**
 * @file aks_crc.c
 * @brief CRC calculation utilities for AKS
 * @author AKS Development Team
 * @date 2025
 * @version 1.0.0
 */

#include "aks_types.h"
#include <stdint.h>
#include <stddef.h>

/* CRC-8 polynomial (x^8 + x^2 + x^1 + 1) */
#define AKS_CRC8_POLYNOMIAL     0x07

/* CRC-16 polynomial (CRC-16-CCITT: x^16 + x^12 + x^5 + 1) */
#define AKS_CRC16_POLYNOMIAL    0x1021

/* CRC-32 polynomial (CRC-32 IEEE 802.3: x^32 + x^26 + x^23 + x^22 + x^16 + x^12 + x^11 + x^10 + x^8 + x^7 + x^5 + x^4 + x^2 + x^1 + 1) */
#define AKS_CRC32_POLYNOMIAL    0x04C11DB7

/* CRC tables for fast calculation */
static uint8_t g_crc8_table[256] = {0};
static uint16_t g_crc16_table[256] = {0};
static uint32_t g_crc32_table[256] = {0};
static bool g_tables_initialized = false;

/* Private function prototypes */
static void aks_crc_init_tables(void);
static uint8_t aks_crc8_calculate_slow(const uint8_t* data, size_t length, uint8_t initial);
static uint16_t aks_crc16_calculate_slow(const uint8_t* data, size_t length, uint16_t initial);
static uint32_t aks_crc32_calculate_slow(const uint8_t* data, size_t length, uint32_t initial);

/**
 * @brief Initialize CRC calculation module
 */
aks_result_t aks_crc_init(void)
{
    if (!g_tables_initialized) {
        aks_crc_init_tables();
        g_tables_initialized = true;
    }
    
    return AKS_OK;
}

/**
 * @brief Calculate CRC-8 checksum
 */
uint8_t aks_crc8_calculate(const uint8_t* data, size_t length)
{
    return aks_crc8_calculate_with_initial(data, length, 0x00);
}

/**
 * @brief Calculate CRC-8 checksum with initial value
 */
uint8_t aks_crc8_calculate_with_initial(const uint8_t* data, size_t length, uint8_t initial)
{
    if (data == NULL || length == 0) {
        return initial;
    }
    
    if (!g_tables_initialized) {
        return aks_crc8_calculate_slow(data, length, initial);
    }
    
    uint8_t crc = initial;
    
    for (size_t i = 0; i < length; i++) {
        crc = g_crc8_table[crc ^ data[i]];
    }
    
    return crc;
}

/**
 * @brief Calculate CRC-16 checksum
 */
uint16_t aks_crc16_calculate(const uint8_t* data, size_t length)
{
    return aks_crc16_calculate_with_initial(data, length, 0xFFFF);
}

/**
 * @brief Calculate CRC-16 checksum with initial value
 */
uint16_t aks_crc16_calculate_with_initial(const uint8_t* data, size_t length, uint16_t initial)
{
    if (data == NULL || length == 0) {
        return initial;
    }
    
    if (!g_tables_initialized) {
        return aks_crc16_calculate_slow(data, length, initial);
    }
    
    uint16_t crc = initial;
    
    for (size_t i = 0; i < length; i++) {
        uint8_t tbl_idx = ((crc >> 8) ^ data[i]) & 0xFF;
        crc = (crc << 8) ^ g_crc16_table[tbl_idx];
    }
    
    return crc;
}

/**
 * @brief Calculate CRC-32 checksum
 */
uint32_t aks_crc32_calculate(const uint8_t* data, size_t length)
{
    return aks_crc32_calculate_with_initial(data, length, 0xFFFFFFFF) ^ 0xFFFFFFFF;
}

/**
 * @brief Calculate CRC-32 checksum with initial value
 */
uint32_t aks_crc32_calculate_with_initial(const uint8_t* data, size_t length, uint32_t initial)
{
    if (data == NULL || length == 0) {
        return initial;
    }
    
    if (!g_tables_initialized) {
        return aks_crc32_calculate_slow(data, length, initial);
    }
    
    uint32_t crc = initial;
    
    for (size_t i = 0; i < length; i++) {
        uint8_t tbl_idx = ((crc >> 24) ^ data[i]) & 0xFF;
        crc = (crc << 8) ^ g_crc32_table[tbl_idx];
    }
    
    return crc;
}

/**
 * @brief Verify CRC-8 checksum
 */
bool aks_crc8_verify(const uint8_t* data, size_t length, uint8_t expected_crc)
{
    if (data == NULL || length == 0) {
        return false;
    }
    
    uint8_t calculated_crc = aks_crc8_calculate(data, length);
    return (calculated_crc == expected_crc);
}

/**
 * @brief Verify CRC-16 checksum
 */
bool aks_crc16_verify(const uint8_t* data, size_t length, uint16_t expected_crc)
{
    if (data == NULL || length == 0) {
        return false;
    }
    
    uint16_t calculated_crc = aks_crc16_calculate(data, length);
    return (calculated_crc == expected_crc);
}

/**
 * @brief Verify CRC-32 checksum
 */
bool aks_crc32_verify(const uint8_t* data, size_t length, uint32_t expected_crc)
{
    if (data == NULL || length == 0) {
        return false;
    }
    
    uint32_t calculated_crc = aks_crc32_calculate(data, length);
    return (calculated_crc == expected_crc);
}

/**
 * @brief Update CRC-8 with additional data
 */
uint8_t aks_crc8_update(uint8_t current_crc, const uint8_t* data, size_t length)
{
    return aks_crc8_calculate_with_initial(data, length, current_crc);
}

/**
 * @brief Update CRC-16 with additional data
 */
uint16_t aks_crc16_update(uint16_t current_crc, const uint8_t* data, size_t length)
{
    return aks_crc16_calculate_with_initial(data, length, current_crc);
}

/**
 * @brief Update CRC-32 with additional data
 */
uint32_t aks_crc32_update(uint32_t current_crc, const uint8_t* data, size_t length)
{
    return aks_crc32_calculate_with_initial(data, length, current_crc);
}

/**
 * @brief Calculate CRC-8 for single byte
 */
uint8_t aks_crc8_byte(uint8_t current_crc, uint8_t byte)
{
    if (!g_tables_initialized) {
        /* Simple polynomial division for single byte */
        uint8_t crc = current_crc ^ byte;
        for (int i = 0; i < 8; i++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ AKS_CRC8_POLYNOMIAL;
            } else {
                crc <<= 1;
            }
        }
        return crc;
    }
    
    return g_crc8_table[current_crc ^ byte];
}

/**
 * @brief Calculate CRC-16 for single byte
 */
uint16_t aks_crc16_byte(uint16_t current_crc, uint8_t byte)
{
    if (!g_tables_initialized) {
        /* Simple polynomial division for single byte */
        uint16_t crc = current_crc ^ (byte << 8);
        for (int i = 0; i < 8; i++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ AKS_CRC16_POLYNOMIAL;
            } else {
                crc <<= 1;
            }
        }
        return crc;
    }
    
    uint8_t tbl_idx = ((current_crc >> 8) ^ byte) & 0xFF;
    return (current_crc << 8) ^ g_crc16_table[tbl_idx];
}

/**
 * @brief Calculate CRC-32 for single byte
 */
uint32_t aks_crc32_byte(uint32_t current_crc, uint8_t byte)
{
    if (!g_tables_initialized) {
        /* Simple polynomial division for single byte */
        uint32_t crc = current_crc ^ ((uint32_t)byte << 24);
        for (int i = 0; i < 8; i++) {
            if (crc & 0x80000000) {
                crc = (crc << 1) ^ AKS_CRC32_POLYNOMIAL;
            } else {
                crc <<= 1;
            }
        }
        return crc;
    }
    
    uint8_t tbl_idx = ((current_crc >> 24) ^ byte) & 0xFF;
    return (current_crc << 8) ^ g_crc32_table[tbl_idx];
}

/* Private function implementations */

/**
 * @brief Initialize CRC lookup tables
 */
static void aks_crc_init_tables(void)
{
    /* Initialize CRC-8 table */
    for (int i = 0; i < 256; i++) {
        uint8_t crc = i;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ AKS_CRC8_POLYNOMIAL;
            } else {
                crc <<= 1;
            }
        }
        g_crc8_table[i] = crc;
    }
    
    /* Initialize CRC-16 table */
    for (int i = 0; i < 256; i++) {
        uint16_t crc = i << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ AKS_CRC16_POLYNOMIAL;
            } else {
                crc <<= 1;
            }
        }
        g_crc16_table[i] = crc;
    }
    
    /* Initialize CRC-32 table */
    for (int i = 0; i < 256; i++) {
        uint32_t crc = (uint32_t)i << 24;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80000000) {
                crc = (crc << 1) ^ AKS_CRC32_POLYNOMIAL;
            } else {
                crc <<= 1;
            }
        }
        g_crc32_table[i] = crc;
    }
}

/**
 * @brief Calculate CRC-8 without lookup table (slow method)
 */
static uint8_t aks_crc8_calculate_slow(const uint8_t* data, size_t length, uint8_t initial)
{
    uint8_t crc = initial;
    
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ AKS_CRC8_POLYNOMIAL;
            } else {
                crc <<= 1;
            }
        }
    }
    
    return crc;
}

/**
 * @brief Calculate CRC-16 without lookup table (slow method)
 */
static uint16_t aks_crc16_calculate_slow(const uint8_t* data, size_t length, uint16_t initial)
{
    uint16_t crc = initial;
    
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ AKS_CRC16_POLYNOMIAL;
            } else {
                crc <<= 1;
            }
        }
    }
    
    return crc;
}

/**
 * @brief Calculate CRC-32 without lookup table (slow method)
 */
static uint32_t aks_crc32_calculate_slow(const uint8_t* data, size_t length, uint32_t initial)
{
    uint32_t crc = initial;
    
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint32_t)data[i] << 24;
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x80000000) {
                crc = (crc << 1) ^ AKS_CRC32_POLYNOMIAL;
            } else {
                crc <<= 1;
            }
        }
    }
    
    return crc;
}
