# Source files CMakeLists.txt

# Core source files
set(CORE_SOURCES
    core/aks_core.c
    core/aks_system.c
    core/aks_config.c
)

# Communication source files
set(COMMUNICATION_SOURCES
    communication/aks_can.c
    communication/aks_uart.c
    communication/aks_spi.c
    communication/aks_telemetry.c
    communication/aks_protocol.c
)

# Sensor source files
set(SENSOR_SOURCES
    sensors/aks_adc.c
    sensors/aks_temperature.c
    sensors/aks_current.c
    sensors/aks_voltage.c
    sensors/aks_gps.c
)

# Actuator source files
set(ACTUATOR_SOURCES
    actuators/aks_motor.c
    actuators/aks_brake.c
    actuators/aks_steering.c
    actuators/aks_battery.c
)

# Safety source files
set(SAFETY_SOURCES
    safety/aks_safety.c
    safety/aks_isolation.c
    safety/aks_monitoring.c
    safety/aks_emergency.c
)

# Utility source files
set(UTILITY_SOURCES
    utils/aks_crc.c
    utils/aks_buffer.c
    utils/aks_queue.c
    utils/aks_timer.c
    utils/aks_math.c
    utils/aks_logger.c
)

# Main application
set(MAIN_SOURCES
    main.c
)

# All source files
set(ALL_SOURCES
    ${CORE_SOURCES}
    ${COMMUNICATION_SOURCES}
    ${SENSOR_SOURCES}
    ${ACTUATOR_SOURCES}
    ${SAFETY_SOURCES}
    ${UTILITY_SOURCES}
    ${MAIN_SOURCES}
)

# Create library for AKS system (excluding main.c)
list(REMOVE_ITEM ALL_SOURCES main.c)
add_library(aks_system STATIC ${ALL_SOURCES})

# Set target properties
set_target_properties(aks_system PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
)

# Target compile definitions
target_compile_definitions(aks_system PRIVATE
    $<$<CONFIG:Debug>:DEBUG>
    $<$<CONFIG:Release>:NDEBUG>
    USE_HAL_DRIVER
    STM32F411xE
)

# Target include directories
target_include_directories(aks_system PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/core
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/communication
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/sensors
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/actuators
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/safety
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/utils
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/hal
)

# Main executable (for embedded target)
if(DEFINED CROSS_COMPILE)
    add_executable(aks_main main.c)
    target_link_libraries(aks_main aks_system)
    
    # STM32 specific linker script
    set_target_properties(aks_main PROPERTIES
        LINK_FLAGS "-T${CMAKE_CURRENT_SOURCE_DIR}/../config/STM32F411RETx_FLASH.ld"
    )
    
    # Generate binary and hex files
    add_custom_command(TARGET aks_main POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:aks_main> ${CMAKE_CURRENT_BINARY_DIR}/aks_main.bin
        COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:aks_main> ${CMAKE_CURRENT_BINARY_DIR}/aks_main.hex
        COMMAND ${CMAKE_SIZE} $<TARGET_FILE:aks_main>
        COMMENT "Generating binary and hex files"
    )
endif()

# Group source files for IDEs
source_group("Core" FILES ${CORE_SOURCES})
source_group("Communication" FILES ${COMMUNICATION_SOURCES})
source_group("Sensors" FILES ${SENSOR_SOURCES})
source_group("Actuators" FILES ${ACTUATOR_SOURCES})
source_group("Safety" FILES ${SAFETY_SOURCES})
source_group("Utils" FILES ${UTILITY_SOURCES})