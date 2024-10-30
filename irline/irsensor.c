#include "irsensor.h"
#include "wifi.h"

uint32_t pulse_start = 0;
uint32_t pulse_end = 0;
bool is_black = false;

MessageBufferHandle_t wifiMessageBuffer;

void init_adc() {
    adc_init();
    adc_gpio_init(IR_SENSOR_PIN);
    adc_select_input(0);  // Select ADC0 (connected to GP26)
}

int read_ir_sensor() {
    return adc_read();  // ADC result (12-bit resolution)
}

void display_line_width(uint32_t pulse_width, const char* surface) {
    float seconds = pulse_width / 1000000.0;
    printf("%s Surface Width: %.6f seconds\n", surface, seconds);
}

void ir_sensor_task(void *pvParameters) {
    init_adc();
    pulse_start = time_us_32();  // Initial time capture

    while (true) {
        int ir_value = read_ir_sensor();

        printf("ADC Value: %d - ", ir_value);
        
        char message[50];
        sniprintf(message, sizeof(message), "ADC Value: %d", ir_value);
        xMessageBufferSend(wifiMessageBuffer, &message, sizeof(message), 0);

        if (ir_value <= WHITE_THRESHOLD) {
            printf("White Surface Detected\n");

            if (is_black) {
                pulse_end = time_us_32();
                uint32_t pulse_width = pulse_end - pulse_start;
                display_line_width(pulse_width, "Black");

                pulse_start = pulse_end;
                is_black = false;
            }
        } else {
            printf("Black Surface Detected\n");

            if (!is_black) {
                pulse_end = time_us_32();
                uint32_t pulse_width = pulse_end - pulse_start;
                display_line_width(pulse_width, "White");

                pulse_start = pulse_end;
                is_black = true;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(500));  // FreeRTOS delay
    }
}

int main() {
    stdio_init_all();
    sleep_ms(5000);

    wifiMessageBuffer = xMessageBufferCreate(1024);

    xTaskCreate(ir_sensor_task, "IR Sensor Task", 256, NULL, 1, NULL);
    xTaskCreate(main_task, "Wifi Task", 256, "remote", 2, NULL);

    vTaskStartScheduler();  // Start FreeRTOS scheduler

    while (true) {
        // The program should never reach here if FreeRTOS is running correctly
    }

    return 0;
}
