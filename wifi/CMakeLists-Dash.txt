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

    # FOR FREERTOS FILE (DashboardListenerFREERTOS.c)
    # Add the executable
    add_executable(DashboardListenerFREERTOS
        DashboardListenerFREERTOS.c
        )

    # Link the necessary libraries
    target_link_libraries(DashboardListenerFREERTOS
    pico_cyw43_arch_lwip_sys_freertos
    pico_stdlib
    hardware_spi
    FreeRTOS-Kernel-Heap4 # FreeRTOS kernel and dynamic heap
    )

    # Enable USB output, disable UART output
    pico_enable_stdio_usb(DashboardListenerFREERTOS 1)
    pico_enable_stdio_uart(DashboardListenerFREERTOS 0)

    # Create map/bin/hex/uf2 files
    pico_add_extra_outputs(DashboardListenerFREERTOS)

    # Include LWIP headers
    target_include_directories(DashboardListenerFREERTOS PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/..
    )

    # FOR FREERTOS FILE (DashboardSenderFREERTOS.c)
    # Add the executable
    add_executable(DashboardSenderFREERTOS
        DashboardSenderFREERTOS.c
        )

    # Link the necessary libraries
    target_link_libraries(DashboardSenderFREERTOS
    pico_cyw43_arch_lwip_sys_freertos
    pico_stdlib
    hardware_spi
    FreeRTOS-Kernel-Heap4 # FreeRTOS kernel and dynamic heap
    hardware_adc # For dummy temperature sensor data
    )

    # Enable USB output, disable UART output
    pico_enable_stdio_usb(DashboardSenderFREERTOS 1)
    pico_enable_stdio_uart(DashboardSenderFREERTOS 0)

    # Create map/bin/hex/uf2 files
    pico_add_extra_outputs(DashboardSenderFREERTOS)

    # Include LWIP headers
    target_include_directories(DashboardSenderFREERTOS PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/..
    )

