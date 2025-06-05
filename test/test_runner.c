/**
 * @file test_runner.c
 * @brief Test runner for AKS unit and integration tests
 * @author AKS Development Team
 * @date 2025
 * @version 1.0.0
 */

#include "unity.h"
#include <stdio.h>

/* External test function declarations */
/* Unit Tests */
extern void test_aks_core_init(void);
extern void test_aks_core_state_management(void);
extern void test_aks_core_error_handling(void);

extern void test_aks_can_init(void);
extern void test_aks_can_transmit(void);
extern void test_aks_can_receive(void);
extern void test_aks_can_filters(void);

extern void test_aks_telemetry_init(void);
extern void test_aks_telemetry_data_formatting(void);
extern void test_aks_telemetry_transmission(void);

extern void test_aks_safety_isolation_monitoring(void);
extern void test_aks_safety_emergency_stop(void);
extern void test_aks_safety_fault_logging(void);

extern void test_aks_uart_basic_operations(void);
extern void test_aks_uart_dma_operations(void);
extern void test_aks_uart_error_handling(void);

extern void test_aks_spi_basic_operations(void);
extern void test_aks_spi_device_operations(void);
extern void test_aks_spi_error_handling(void);

extern void test_aks_math_functions(void);
extern void test_aks_crc_calculations(void);
extern void test_aks_buffer_operations(void);
extern void test_aks_queue_operations(void);

/* Integration Tests */
extern void test_communication_integration(void);
extern void test_telemetry_integration(void);
extern void test_safety_integration(void);
extern void test_system_integration(void);

/* Test Setup and Teardown */
void setUp(void)
{
    /* Setup code that runs before each test */
}

void tearDown(void)
{
    /* Cleanup code that runs after each test */
}

/* Test Suite Runner Functions */
static void run_core_tests(void)
{
    printf("\n=== Running Core Tests ===\n");
    RUN_TEST(test_aks_core_init);
    RUN_TEST(test_aks_core_state_management);
    RUN_TEST(test_aks_core_error_handling);
}

static void run_communication_tests(void)
{
    printf("\n=== Running Communication Tests ===\n");
    RUN_TEST(test_aks_can_init);
    RUN_TEST(test_aks_can_transmit);
    RUN_TEST(test_aks_can_receive);
    RUN_TEST(test_aks_can_filters);
    
    RUN_TEST(test_aks_uart_basic_operations);
    RUN_TEST(test_aks_uart_dma_operations);
    RUN_TEST(test_aks_uart_error_handling);
    
    RUN_TEST(test_aks_spi_basic_operations);
    RUN_TEST(test_aks_spi_device_operations);
    RUN_TEST(test_aks_spi_error_handling);
}

static void run_telemetry_tests(void)
{
    printf("\n=== Running Telemetry Tests ===\n");
    RUN_TEST(test_aks_telemetry_init);
    RUN_TEST(test_aks_telemetry_data_formatting);
    RUN_TEST(test_aks_telemetry_transmission);
}

static void run_safety_tests(void)
{
    printf("\n=== Running Safety Tests ===\n");
    RUN_TEST(test_aks_safety_isolation_monitoring);
    RUN_TEST(test_aks_safety_emergency_stop);
    RUN_TEST(test_aks_safety_fault_logging);
}

static void run_utility_tests(void)
{
    printf("\n=== Running Utility Tests ===\n");
    RUN_TEST(test_aks_math_functions);
    RUN_TEST(test_aks_crc_calculations);
    RUN_TEST(test_aks_buffer_operations);
    RUN_TEST(test_aks_queue_operations);
}

static void run_integration_tests(void)
{
    printf("\n=== Running Integration Tests ===\n");
    RUN_TEST(test_communication_integration);
    RUN_TEST(test_telemetry_integration);
    RUN_TEST(test_safety_integration);
    RUN_TEST(test_system_integration);
}

/* Main test runner */
int main(void)
{
    printf("AKS Test Suite v1.0.0\n");
    printf("==============================\n");
    
    UNITY_BEGIN();
    
    /* Determine test type from command line or environment */
    char* test_type = getenv("TEST_TYPE");
    
    if (test_type == NULL || strcmp(test_type, "unit") == 0) {
        /* Run unit tests */
        printf("Running Unit Tests...\n");
        run_core_tests();
        run_communication_tests();
        run_telemetry_tests();
        run_safety_tests();
        run_utility_tests();
    }
    else if (strcmp(test_type, "integration") == 0) {
        /* Run integration tests */
        printf("Running Integration Tests...\n");
        run_integration_tests();
    }
    else if (strcmp(test_type, "all") == 0) {
        /* Run all tests */
        printf("Running All Tests...\n");
        run_core_tests();
        run_communication_tests();
        run_telemetry_tests();
        run_safety_tests();
        run_utility_tests();
        run_integration_tests();
    }
    else {
        printf("Unknown test type: %s\n", test_type);
        printf("Valid types: unit, integration, all\n");
        return -1;
    }
    
    int result = UNITY_END();
    
    printf("\n==============================\n");
    if (result == 0) {
        printf("All tests PASSED!\n");
    } else {
        printf("Some tests FAILED!\n");
    }
    
    return result;
}