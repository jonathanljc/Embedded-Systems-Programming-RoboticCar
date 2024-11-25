# INF2004 Embedded Systems Programming - Team P1C - Robotic Car
This repository contains the files for the INF2004 Robotic Car project for Team P1C

## P1C Team Members
- Felix Chang (2301105) - Buddy 1
- Ong Jia En Darryl (2301402) - Buddy 2
- Chew Liang Zhi (2300948) - Buddy 3
- Ong Yong Sheng (2301123) - Buddy 4
- Lim Jing Chuan Jonathan (2300923) - Buddy 5

## Requirements
- Pico SDK
- Pico ARM Toolchain
    - CMake
- Pico W
- FreeRTOS Kernel
- Mosquitto MQTT Broker

## File Structure
- `main/` folder contains the files for the 3 Pico Ws (Remote, Car, Dashboard)
- `wifi/` folder contains the files for the Wi-Fi component (Buddy 1)
- `motor/` folder contains the files for the Motor component (Buddy 2)
- `irline/` folder contains the files for the IR Sensors component (Buddy 3)
- `magnenometer/` folder contains the files for the Accelerometer component (Buddy 4)
- `encoder/` folder contains the files for the Encoder component (Buddy 5)

We are utilising file headers and making our components into libraries for the main .c files in `main/`.
When building, only would need to build the remote, car, dashboard executable and place the .uf2 into the respective Pico Ws.


## Variables
| Variable                   | Description                           |
| ----------------------     | ------------------------------------- |
| `PICO_SDK_PATH`            | Path to the Pico SDK directory        |
| `FREERTOS_KERNEL_PATH`     | Path to the FreeRTOS Kernel directory |
| `MQTT_SERVER_IP` in wifi.h | IP of MQTT server                     |
| `MQTT_SSID` in wifi.h      | Wi-Fi network SSID                    |
| `MQTT_PW` in wifi.h        | Wi-Fi network password                |