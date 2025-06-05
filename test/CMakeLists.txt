# Test CMakeLists.txt

# Find required packages for testing
find_package(PkgConfig)

# Check if Unity test framework is available
pkg_check_modules(UNITY unity)

if(NOT UNITY_FOUND)
    # Download and build Unity if not found
    include(FetchContent)
    FetchContent_Declare(
        unity
        GIT_REPOSITORY https://github.com/ThrowTheSwitch/Unity.git
        GIT_TAG v2.5.2
    )
    FetchContent_MakeAvailable(unity)
    set(UNITY_LIBRARIES unity)
    set(UNITY_INCLUDE_DIRS ${unity_SOURCE_DIR}/src)
endif()

# Enable testing
enable_testing()

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/core
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/communication
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/sensors
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/actuators
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/safety
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/utils
    ${CMAKE_CURRENT_SOURCE_DIR}/mocks
    ${UNITY_INCLUDE_DIRS}
)

# Compiler flags for tests
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DTEST_BUILD -DUNITY_INCLUDE_DOUBLE")

# Mock source files
set(MOCK_SOURCES
    mocks/mock_hal.c
    mocks/mock_stm32f4xx.c
    mocks/mock_can.c
    mocks/mock_uart.c
    mocks/mock_spi.c
    mocks/mock_adc.c
    mocks/mock_gpio.c
)

# Unit test source files
set(UNIT_TEST_SOURCES
    unit/test_aks_core.c
    unit/test_aks_can.c
    unit/test_aks_telemetry.c
    unit/test_aks_safety.c
    unit/test_aks_uart.c
    unit/test_aks_spi.c
    unit/test_aks_math.c
    unit/test_aks_crc.c
    unit/test_aks_buffer.c
    unit/test_aks_queue.c
)

# Integration test source files
set(INTEGRATION_TEST_SOURCES
    integration/test_communication_integration.c
    integration/test_telemetry_integration.c
    integration/test_safety_integration.c
    integration/test_system_integration.c
)

# Test runner
set(TEST_RUNNER_SOURCE
    test_runner.c
)

# Create mock library
add_library(aks_mocks STATIC ${MOCK_SOURCES})
target_include_directories(aks_mocks PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/mocks)

# Create test executable for unit tests
add_executable(aks_unit_tests ${UNIT_TEST_SOURCES} ${TEST_RUNNER_SOURCE})
target_link_libraries(aks_unit_tests aks_system aks_mocks ${UNITY_LIBRARIES})

# Create test executable for integration tests
add_executable(aks_integration_tests ${INTEGRATION_TEST_SOURCES} ${TEST_RUNNER_SOURCE})
target_link_libraries(aks_integration_tests aks_system aks_mocks ${UNITY_LIBRARIES})

# Add tests to CTest
add_test(NAME UnitTests COMMAND aks_unit_tests)
add_test(NAME IntegrationTests COMMAND aks_integration_tests)

# Set test properties
set_tests_properties(UnitTests PROPERTIES
    TIMEOUT 60
    LABELS "unit"
)

set_tests_properties(IntegrationTests PROPERTIES
    TIMEOUT 120
    LABELS "integration"
)

# Custom target to run all tests
add_custom_target(run_all_tests
    COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure
    DEPENDS aks_unit_tests aks_integration_tests
    COMMENT "Running all tests"
)

# Custom target for unit tests only
add_custom_target(run_unit_tests
    COMMAND ${CMAKE_CTEST_COMMAND} -L unit --output-on-failure
    DEPENDS aks_unit_tests
    COMMENT "Running unit tests"
)

# Custom target for integration tests only
add_custom_target(run_integration_tests
    COMMAND ${CMAKE_CTEST_COMMAND} -L integration --output-on-failure
    DEPENDS aks_integration_tests
    COMMENT "Running integration tests"
)

# Test coverage (if gcov available)
option(ENABLE_COVERAGE "Enable test coverage" OFF)
if(ENABLE_COVERAGE)
    find_program(GCOV_PATH gcov)
    find_program(LCOV_PATH lcov)
    find_program(GENHTML_PATH genhtml)
    
    if(GCOV_PATH AND LCOV_PATH AND GENHTML_PATH)
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage -fprofile-arcs -ftest-coverage")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")
        
        add_custom_target(coverage
            COMMAND ${CMAKE_CTEST_COMMAND}
            COMMAND ${LCOV_PATH} --directory . --capture --output-file coverage.info
            COMMAND ${LCOV_PATH} --remove coverage.info '/usr/*' '*/test/*' '*/mocks/*' --output-file coverage.info.cleaned
            COMMAND ${GENHTML_PATH} coverage.info.cleaned --output-directory coverage_html
            WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
            COMMENT "Generating test coverage report"
        )
    endif()
endif()

# Valgrind memory check (if available)
find_program(VALGRIND_PATH valgrind)
if(VALGRIND_PATH)
    add_custom_target(memcheck
        COMMAND ${VALGRIND_PATH} --tool=memcheck --leak-check=full --show-reachable=yes --track-origins=yes --error-exitcode=1 ./aks_unit_tests
        DEPENDS aks_unit_tests
        COMMENT "Running memory check with Valgrind"
    )
endif()