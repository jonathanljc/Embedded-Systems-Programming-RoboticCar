cmake_minimum_required(VERSION 3.12)

# Pull in SDK (must be before project)
set(PICO_SDK_FETCH_FROM_GIT on)
include(pico_sdk_import.cmake)  
include(FreeRTOS_Kernel_import.cmake)
include(pico_extras_import_optional.cmake)

project(INF2004_TEAM_P1C C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(PICO_BOARD pico_w)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.3.0")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.3.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

# Initialize the SDK
pico_sdk_init()


# Add the directory that contains FreeRTOSConfig.h
include_directories(${CMAKE_SOURCE_DIR})  # Add root directory to include path


if (NOT PICO_NO_HARDWARE)
    add_subdirectory(Station1)
    add_subdirectory(Station2)
    add_subdirectory(Station3)
endif ()