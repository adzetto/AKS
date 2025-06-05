/**
 * @file aks_isolation.h
 * @brief Isolation monitoring system header
 * @version 1.0.0
 * @date 2025
 */

#ifndef AKS_ISOLATION_H
#define AKS_ISOLATION_H

#include "aks_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Function declarations */
aks_result_t aks_isolation_init(void);
aks_result_t aks_isolation_deinit(void);
aks_result_t aks_isolation_start(void);
aks_result_t aks_isolation_stop(void);
aks_result_t aks_isolation_task(void);
aks_result_t aks_isolation_measure(void);
aks_result_t aks_isolation_get_status(aks_isolation_status_t* status);
aks_result_t aks_isolation_get_resistance(float* resistance);
aks_result_t aks_isolation_emergency_stop(void);

/**
 * @brief Perform isolation monitoring self-test
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_isolation_self_test(void);

#ifdef __cplusplus
}
#endif

#endif /* AKS_ISOLATION_H */ 