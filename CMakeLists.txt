cmake_minimum_required(VERSION 3.12)

# Check if building natively for Host PC or cross-compiling for Pico
if(NOT PICO_PLATFORM)
    # --- HOST PC TARGET (UNIT TESTS) ---
    project(imu_unit_tests C CXX)
    enable_testing()
    
    # Include GoogleTest or other frameworks
    find_package(GTest REQUIRED)
    
    # Run test
    add_executable(run_tests test/test_imu.cpp imu_processor/imu_processor.c)
    target_link_libraries(run_tests GTest::gtest_main m)
    
    include(GoogleTest)
    gtest_discover_tests(run_tests)

else()
    # --- RASPBERRY PI PICO TARGET ---
    include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

    project(pico_micro_ros_example C CXX ASM)
    set(CMAKE_C_STANDARD 11)
    set(CMAKE_CXX_STANDARD 17)

    pico_sdk_init()

    link_directories(libmicroros)
    
    add_executable(pico_micro_ros_example
        pico_micro_ros_example.c
        pico_uart_transport.c
        imu_processor/imu_processor.c
    )
    
    target_link_libraries(pico_micro_ros_example
        pico_stdlib
        microros
        hardware_i2c
        pico_cyw43_arch_none
    )

    target_include_directories(pico_micro_ros_example PUBLIC
        libmicroros/include
        imu_processor
    )

    set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

    # Configure Pico
    pico_enable_stdio_usb(pico_micro_ros_example 1)
    pico_enable_stdio_uart(pico_micro_ros_example 0)
    
    add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
    add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
    add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

    # Generate UF2
    pico_add_extra_outputs(pico_micro_ros_example)

endif()