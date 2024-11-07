#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "FreeRTOS.h"
#include "task.h"
#include "hardware/adc.h"
#include <stdio.h>
#include <string.h>
#include <limits.h>  
#include <stdlib.h>


#define R_MOTOR_PWM_PIN 10
#define RIGHT_MOTOR_PIN1 14
#define RIGHT_MOTOR_PIN2 15
#define LEFT_MOTOR_PWM 11
#define LEFT_MOTOR_PIN1 12
#define LEFT_MOTOR_PIN2 13



#define MAX_DUTY_CYCLE 12500


#define IR_SENSOR_PIN 26 
#define THRESHOLD 1700   
#define IR_LINE_PIN 27          
#define LINE_THRESHOLD 500  
#define NUM_SAMPLES 10
#define MAX_BARS 29
#define NOISE_THRESHOLD 500    
#define DEBOUNCE_DELAY 500    
volatile uint64_t last_white_time = 0;  
#define MAX_WHITE_TIME 10000000         

// Code 39 Encoding 
const char *code39_patterns[] = {
    "NWNWNWWNWNNWNWNWNWNW", "NNWNNWNNNWNNWNWNWNW", "NWNNNNWNNWNNWNWNWNW", "NNWNNWWNNNWNNWNWNWN", "NWNWNWWNNNNWNWNWNWN",
    "NNWNNWWNNWNNWNWNWNWN", "NWNNNNWWNNWNNWNWNWN", "NNWNNWNNWNWNWNWNWNW", "NWNNNNWNNWNNWNWNWNW", "NNWNNWWNNNNWNWNWNWN", 
    "NNWNNWNNNNWNWNWNWNWN", "NNWNNWNNNWNNWNWNWNW", "NNWNNWNNNWNNWNWNWNW", "NWNWNWNWNWNWNWNWNWN", "NWNNWNNNWNNWNWNWNWN", 
    "NNWNNWNNNWNNWNWNWNWN", "NNWNWNWNWNWNWNWNWNW", "NWNWNWNWNWNWNWNWNWN", "NWNNWNWNWNWNWNWNWNW", "NWNWNWNWNWNWNWNWNWN", 
    "NWNWNWNWNWNWNWNWNWNW", "NNWNWNWNWNWNWNWNWNW", "NWNNWNWNWNWNWNWNWNW", "NWNWNWNWNWNWNWNWNWN", "NWNWNWNWNWNWNWNWNWN", 
    "NNWNWNWNWNWNWNWNWNWN", "NWNWNWNWNWNWNWNWNWN", "NNWNWNWNWNWNWNWNWNW", "NWNWNWNWNWNWNWNWNWN", "NNWNWNWNWNWNWNWNWNW", 
    "NNWNWNWNWNWNWNWNWNWN", "NNWNWNWNWNWNWNWNWNW", "NWNWNWNWNWNWNWNWNWN", "NWNWNWNWNWNWNWNWNWN", "NWNWNWNWNWNWNWNWNWN", 
    "NNWNWNWNWNWNWNWNWNWN", "NNWNWNWNWNWNWNWNWNW", "NNWNWNWNWNWNWNWNWNW", "NWNWNWNWNWNWNWNWNWN"                          
};

const char code39_chars[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-*";

// Function declarations
void setup_pwm(uint32_t Leftgpio, uint32_t Rightgpio);
void init_motor_pins();
void move_forward(uint32_t Leftgpio, uint32_t Rightgpio, float speed);
void straight_line_task(void *pvParameters);
void detect_colour(void *pvParameters);
uint16_t read_adc();
void reset();
char char_decodingacter();
void barcode_decoding();
void display();
int levenshtein_distance(const char *s, const char *t);
char categorize_length(uint64_t length, uint64_t max_length);

// Motor Control Code
void setup_pwm(uint32_t Leftgpio, uint32_t Rightgpio)
{

    gpio_set_function(Rightgpio, GPIO_FUNC_PWM);
    uint32_t slice_num_right = pwm_gpio_to_slice_num(Rightgpio);
    pwm_set_clkdiv(slice_num_right, 100);
    pwm_set_wrap(slice_num_right, MAX_DUTY_CYCLE);
    pwm_set_enabled(slice_num_right, true);
    pwm_set_chan_level(slice_num_right, PWM_CHAN_B, 0);

    gpio_set_function(Leftgpio, GPIO_FUNC_PWM);
    uint32_t slice_num_left = pwm_gpio_to_slice_num(Leftgpio);
    pwm_set_clkdiv(slice_num_left, 100);
    pwm_set_wrap(slice_num_left, MAX_DUTY_CYCLE);
    pwm_set_enabled(slice_num_left, true);
    pwm_set_chan_level(slice_num_left, PWM_CHAN_A, 0);
}

void init_motor_pins()
{
    gpio_init(RIGHT_MOTOR_PIN1);
    gpio_init(RIGHT_MOTOR_PIN2);
    gpio_init(LEFT_MOTOR_PIN1);
    gpio_init(LEFT_MOTOR_PIN2);
    gpio_set_dir(RIGHT_MOTOR_PIN1, GPIO_OUT);
    gpio_set_dir(RIGHT_MOTOR_PIN2, GPIO_OUT);
    gpio_set_dir(LEFT_MOTOR_PIN1, GPIO_OUT);
    gpio_set_dir(LEFT_MOTOR_PIN2, GPIO_OUT);
}

void set_motor_velocity(uint32_t gpio, float speed, bool is_left)
{
    if (speed > 1.0)
        speed = 1.0;
    if (speed < 0.0)
        speed = 0.0;

    uint32_t duty_cycle = (uint32_t)(speed * MAX_DUTY_CYCLE);
    pwm_set_chan_level(pwm_gpio_to_slice_num(gpio), is_left ? PWM_CHAN_A : PWM_CHAN_B, duty_cycle);
}


void move_forward(uint32_t Leftgpio, uint32_t Rightgpio, float speed) {
    gpio_put(LEFT_MOTOR_PIN1, 1);
    gpio_put(LEFT_MOTOR_PIN2, 0);
    gpio_put(RIGHT_MOTOR_PIN1, 1);
    gpio_put(RIGHT_MOTOR_PIN2, 0);
    set_motor_velocity(Leftgpio, speed - 0.08, true);
    set_motor_velocity(Rightgpio, speed, false);
}

void stop_rotation() {
    set_motor_velocity(LEFT_MOTOR_PWM, 0, true);
    set_motor_velocity(R_MOTOR_PWM_PIN, 0, false);
}

bool is_black_detected(uint8_t pin, uint16_t threshold) {
    adc_select_input(pin == IR_LINE_PIN ? 1 : 0);
    uint32_t adc_sum = 0;
    for (int i = 0; i < NUM_SAMPLES; i++) {
        adc_sum += adc_read();
        sleep_us(100);
    }
    uint16_t adc_value = adc_sum / NUM_SAMPLES;
    return adc_value > threshold;
}



void right_rotation(uint32_t Leftgpio, uint32_t Rightgpio) {
    gpio_put(LEFT_MOTOR_PIN1, 1);
    gpio_put(LEFT_MOTOR_PIN2, 0);
    gpio_put(RIGHT_MOTOR_PIN1, 0);
    gpio_put(RIGHT_MOTOR_PIN2, 1);
    set_motor_velocity(Leftgpio, 0.6, true);
    set_motor_velocity(Rightgpio, 0.9, false);

  
    for (int i = 0; i < 10; i++) {  
        if (is_black_detected(IR_LINE_PIN,LINE_THRESHOLD)) {
            stop_rotation();
            return;  
        }
        sleep_ms(50);
    }
    stop_rotation();
}

void left_rotation(uint32_t Leftgpio, uint32_t Rightgpio) {
    gpio_put(LEFT_MOTOR_PIN1, 0);
    gpio_put(LEFT_MOTOR_PIN2, 1);
    gpio_put(RIGHT_MOTOR_PIN1, 1);
    gpio_put(RIGHT_MOTOR_PIN2, 0);
    set_motor_velocity(Leftgpio, 0.6, true);
    set_motor_velocity(Rightgpio, 0.9, false);

   
    for (int i = 0; i < 10; i++) {  
        if (is_black_detected(IR_LINE_PIN,LINE_THRESHOLD)) {
            stop_rotation();
            return;  
        }
        sleep_ms(50);
    }
    stop_rotation();
}

void straight_line_task(void *pvParameters)
{
    float move_speed = 0.0; 
    while (true)
    {
        move_forward(LEFT_MOTOR_PWM, R_MOTOR_PWM_PIN, move_speed);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


volatile uint64_t last_transition_time = 0; 
const char *previous_color = "White";      
bool decoding_active = false;              
bool initial_black_detected = false;       


uint64_t bar_lengths[MAX_BARS];    
const char *bar_colors[MAX_BARS]; 
int bar_count = 0;                
uint64_t max_length = 0;          

uint16_t read_adc()
{
    return adc_read();
}

void reset()
{
    for (int i = 0; i < MAX_BARS; i++)
    {
        bar_lengths[i] = 0;
        bar_colors[i] = "Unknown";
    }
    bar_count = 0;
    max_length = 0;
    initial_black_detected = false;
    previous_color = "White";
    last_white_time = 0;
}

void display()
{
    printf("Captured Bar lengths (during decoding):\n");
    for (int i = 0; i < bar_count; i++)
    {
        printf("Bar #%d: length = %llu us, Color = %s\n", i, bar_lengths[i], bar_colors[i]);
    }
}

char categorize_length(uint64_t length, uint64_t max_length)
{
   
    if (length <= max_length * 0.7)
        return 'N'; 
    else
        return 'W'; 
}


int levenshtein_distance(const char *s, const char *t)
{
    int len_s = strlen(s);
    int len_t = strlen(t);


    if (abs(len_s - len_t) > 10)
    {
        return INT_MAX;
    }

    int matrix[len_s + 1][len_t + 1];

   
    for (int i = 0; i <= len_s; i++)
        matrix[i][0] = i;
    for (int j = 0; j <= len_t; j++)
        matrix[0][j] = j;

  
    for (int i = 1; i <= len_s; i++)
    {
        for (int j = 1; j <= len_t; j++)
        {
            int cost = s[i - 1] == t[j - 1] ? 0 : 1;
            int deletion = matrix[i - 1][j] + 1;
            int insertion = matrix[i][j - 1] + 1;
            int substitution = matrix[i - 1][j - 1] + cost;

            int min = deletion < insertion ? deletion : insertion;
            matrix[i][j] = min < substitution ? min : substitution;
        }
    }
    return matrix[len_s][len_t];
}

void barcode_decoding()
{
    display(); 
    char decoded_char = char_decodingacter(); 

   
    if (decoded_char != '?') { 
        printf("Decoded Character: %c\n", decoded_char);
    } else {
        printf("Decoded Character: Not Recognized\n");
    }

    reset();  
    decoding_active = false;
}

char char_decodingacter()
{

    char pattern[MAX_BARS + 1] = {0};

    for (int i = 0; i < bar_count; i++)
    {
        pattern[i] = categorize_length(bar_lengths[i], max_length);
    }
    pattern[bar_count] = '\0';


    int min_distance = INT_MAX;
    char best_matches[10]; 
    int best_matches_count = 0;
    int best_match_is_reverse[10];  

  
    char reverse_pattern[MAX_BARS + 1] = {0};
    for (int i = 0; i < bar_count; i++)
    {
        reverse_pattern[i] = pattern[bar_count - i - 1];
    }
    reverse_pattern[bar_count] = '\0';

 
    int num_patterns = sizeof(code39_patterns) / sizeof(code39_patterns[0]);
    for (int i = 0; i < num_patterns; i++)
    {
        const char *reference_pattern = code39_patterns[i];

        int distance = levenshtein_distance(pattern, reference_pattern);
        int reverse_distance = levenshtein_distance(reverse_pattern, reference_pattern);


        int current_min_distance = (distance < reverse_distance) ? distance : reverse_distance;
        int is_reverse = (reverse_distance <= distance) ? 1 : 0;

        if (current_min_distance < min_distance)
        {
            min_distance = current_min_distance;
          
            best_matches[0] = code39_chars[i];
            best_match_is_reverse[0] = is_reverse;
            best_matches_count = 1;
        }
        else if (current_min_distance == min_distance)
        {
            if (best_matches_count < sizeof(best_matches))
            {
                best_matches[best_matches_count] = code39_chars[i];
                best_match_is_reverse[best_matches_count] = is_reverse;
                best_matches_count++;
            }
        }
    }


    int max_acceptable_distance = 10; 

    if (min_distance <= max_acceptable_distance)
    {
        for (int i = 0; i < best_matches_count; i++)
        {
            printf("%c%s ", best_matches[i], best_match_is_reverse[i] ? "(Reversed)" : "");
        }
        printf("with Distance: %d\n", min_distance);

       
        return best_matches[0];
    }
    else
    {

        return '?';
    }
}




void detect_colour(void *pvParameters)
{
    adc_init();
    adc_gpio_init(IR_SENSOR_PIN);
    adc_select_input(0);

    while (true)
    {
        uint16_t adc_value = read_adc();

        const char *current_color = (adc_value > THRESHOLD) ? "Black" : "White";
        uint64_t current_time = time_us_64();

        if (!decoding_active && strcmp(current_color, "Black") == 0)
        {
            decoding_active = true;
            reset();
            last_transition_time = current_time;
            previous_color = "Black"; 
            printf("Barcode Pattern\n");
        }

        if (decoding_active && !initial_black_detected && strcmp(current_color, "Black") == 0)
        {
            initial_black_detected = true;
            previous_color = "Black";
        }

        if (decoding_active && initial_black_detected)
        {
            if (strcmp(current_color, previous_color) != 0)
            {
                uint64_t pulse_length = current_time - last_transition_time;

                if (pulse_length > NOISE_THRESHOLD + DEBOUNCE_DELAY && bar_count < MAX_BARS)
                {
                    bar_lengths[bar_count] = pulse_length;
                    bar_colors[bar_count] = previous_color;

                    printf("Colour: %s\n", previous_color);
                    printf("#%d: Pulse_length= %llu us\n", bar_count, pulse_length);

                    if (pulse_length > max_length)
                    {
                        max_length = pulse_length;
                       
                    }
                    bar_count++;
                }

                if (bar_count >= MAX_BARS)
                {
                    barcode_decoding();
                }

                previous_color = current_color;
                last_transition_time = current_time;
            }

           
            if (strcmp(current_color, "White") == 0)
            {
                if (last_white_time == 0)
                {
                    last_white_time = current_time; 
                }
                else if (current_time - last_white_time > MAX_WHITE_TIME)
                {
                    printf("Show Decoding\n");
                    barcode_decoding();
                    last_white_time = 0; 
                }
            }
            else
            {
                last_white_time = 0; 
            }
        }
        else
        {
            
            last_white_time = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(1)); 
    }
}

void detect_line_difference_task(void *pvParameters) {
    adc_init();
    adc_gpio_init(IR_LINE_PIN);
    adc_select_input(1);

   bool on_black = true;

   

    while (true) {
        if (is_black_detected(IR_LINE_PIN,LINE_THRESHOLD)) {
            if (!on_black) { 
                stop_rotation(); 
                move_forward(LEFT_MOTOR_PWM, R_MOTOR_PWM_PIN, 0.5); 
                on_black = true;
               
            }
        } else { 
            if (on_black) {
                stop_rotation(); 
                on_black = false;
            }

        }

        vTaskDelay(pdMS_TO_TICKS(50)); 
    }
}

int main()
{
    stdio_init_all();
    init_motor_pins();
    setup_pwm(LEFT_MOTOR_PWM, R_MOTOR_PWM_PIN);

    TaskHandle_t detect_line_difference_handle, detect_surface_contrast_handle;

    
    xTaskCreate(detect_line_difference_task, "Line Following", 1024, NULL, 2, &detect_line_difference_handle);
    xTaskCreate(detect_colour, "Detecting Barcode", 1024, NULL, 1, &detect_surface_contrast_handle);

    vTaskStartScheduler();
    while (true)
    {
        
    }
}