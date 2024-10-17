if (NOT FREERTOS_KERNEL_PATH AND NOT DEFINED ENV{FREERTOS_KERNEL_PATH})
    message("Skipping Pico W FreeRTOS examples as FREERTOS_KERNEL_PATH not defined")
else()
    include(FreeRTOS_Kernel_import.cmake)
    
    # Add the executable
    add_executable(DashboardListener 
        DashboardListener.c
        ${PICO_LWIP_CONTRIB_PATH}/apps/ping/ping.c
        )
    
    # Link the necessary libraries
    target_link_libraries(DashboardListener
    pico_stdlib
    pico_cyw43_arch_lwip_threadsafe_background
    hardware_spi
    FreeRTOS-Kernel-Heap4 # FreeRTOS kernel and dynamic heap
    )
    
    # Enable USB output, disable UART output
    pico_enable_stdio_usb(DashboardListener 1)
    pico_enable_stdio_uart(DashboardListener 0)
    
    # Create map/bin/hex/uf2 files
    pico_add_extra_outputs(DashboardListener)
    
    # Include LWIP headers
    target_include_directories(DashboardListener PRIVATE 
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/..
    ${PICO_LWIP_CONTRIB_PATH}/apps/ping
    )
endif()