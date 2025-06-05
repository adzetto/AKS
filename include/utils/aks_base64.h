/**
 * @file aks_base64.h
 * @brief Base64 encoding/decoding utilities
 * @version 1.0.0
 * @date 2025
 */

#ifndef AKS_BASE64_H
#define AKS_BASE64_H

#include "aks_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Encode binary data to base64 string
 * @param input Input binary data
 * @param input_length Length of input data
 * @param output Output buffer for base64 string
 * @param output_size Size of output buffer
 * @return Length of encoded string, 0 on error
 */
uint16_t aks_base64_encode(const uint8_t* input, uint16_t input_length, 
                          char* output, uint16_t output_size);

/**
 * @brief Decode base64 string to binary data
 * @param input Input base64 string
 * @param output Output buffer for binary data
 * @param output_size Size of output buffer
 * @return Length of decoded data, 0 on error
 */
uint16_t aks_base64_decode(const char* input, uint8_t* output, uint16_t output_size);

#ifdef __cplusplus
}
#endif

#endif /* AKS_BASE64_H */ 