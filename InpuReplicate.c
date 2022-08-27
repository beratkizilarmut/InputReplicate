#include <stdio.h>
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "esp_attr.h"
#include "soc/rtc.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "esp_log.h"

#define CAP0_INT_EN BIT(27)  //Capture 0 interrupt bit
#define GPIO_CAP0_IN   25   //Set GPIO 25 as  CAP0
#define CAPTURE_TOGGLE_STACK 1024 // 1kB Stack size
#define LED_TIMER 26 // Using GPIO26 for Timer status LED
#define LED_PWM 27 // Using GPIO27 for PWM LED
#define CAPTURE_TOGGLE_PRIO 0 // Priority one
#define LED_DELAY 5000 // 5000 ms delay between led toggles
#define LOG_TAG "Log "

xQueueHandle cap_queue;
static mcpwm_dev_t *MCPWM[2] = {&MCPWM0, &MCPWM1};

/* Global sequence parsing pointer and counter */
uint32_t *parse_array;
uint32_t parse_array_pos=0;

/**
 * @brief Sequence execution function, executes the given array sequence that has ms time values
 */
void Execute_Capture(uint32_t *Sequence, uint32_t Sequence_Size)
{   
    ESP_LOGW(LOG_TAG, "Executing captured led sequence");
    uint32_t Sequence_Time = 0; // Clear execute time window
    gpio_set_level(LED_PWM, 0); // Clear LED
    for(int i=0; i < Sequence_Size; i++)
    {   
        TickType_t xDelay = *(Sequence + i); // Reach sequence array to gather data
        ESP_LOGW(LOG_TAG, "Blink %d ms", xDelay);
        gpio_set_level(LED_PWM, !gpio_get_level(LED_PWM)); // Toggle LED
        Sequence_Time = Sequence_Time + xDelay;
        if(Sequence_Time > LED_DELAY)
        {   // If sequence time surpasses set capture window, break
            gpio_set_level(LED_PWM, 0);
            break;
        }
        vTaskDelay(xDelay/portTICK_PERIOD_MS); // Delay for sequence time
    }
    gpio_set_level(LED_PWM, 0); // Clear LED
} 

/**
 * @brief When button interrupt occurs, we receive the counter value and display the time between any edge and gather the LED sequence
 */
static void disp_captured_signal(void *arg)
{
    uint32_t *current_cap_value = (uint32_t *)malloc(sizeof(uint32_t)); // Allocate memory for current val
    uint32_t *previous_cap_value = (uint32_t *)malloc(sizeof(uint32_t)); // Allocate memory for previous val
    uint32_t capture_signal;
    while (1) 
    {
        xQueueReceive(cap_queue, &capture_signal, portMAX_DELAY); // Receive captured input from queue
        current_cap_value[0] = capture_signal - previous_cap_value[0]; // Subtract previous value to get the differential time
        previous_cap_value[0] = capture_signal; // Set new time as previous value
        current_cap_value[0] = (current_cap_value[0] / 10000) * (10000000000 / rtc_clk_apb_freq_get()); // Get us
        current_cap_value[0] = current_cap_value[0] / 1000; // Get ms
        if(gpio_get_level(LED_TIMER))
        {
            if(current_cap_value[0] >= LED_DELAY)
            {
                current_cap_value[0] = current_cap_value[0] % LED_DELAY; // Clean up the data
            }
            
            if(current_cap_value[0] >= 20)
            {   
                printf("CAP0 : %d ms\n", current_cap_value[0]);
                parse_array[parse_array_pos] = current_cap_value[0]; // Store the sequence value
                parse_array_pos++; // Increase sequence position counter
            }

        }
    }
}

/**
 * @brief Timer callback handler, allocated and deallocates array on memory. Toggles capture inticator LED
 */
static void LED_Toggle_wTimer(TimerHandle_t mypxTimer)
{
    ESP_LOGW(LOG_TAG, "Timer Led Toggle");
    if(gpio_get_level(LED_TIMER))
    {   
        gpio_set_level(LED_TIMER, !gpio_get_level(LED_TIMER)); // Toggle LED
        Execute_Capture(parse_array, parse_array_pos); // Executing captured sequence
        parse_array_pos=0; // Resetting array position counter
        free(parse_array); // Freeing allocated memory
    }
    else if(!gpio_get_level(LED_TIMER))
    {
        gpio_set_level(LED_TIMER, !gpio_get_level(LED_TIMER)); // Toggle LED
        parse_array = (uint32_t *)malloc(sizeof(uint32_t)); // Allocating memory for sequence
    }
}

/**
 * @brief ISR handler function, here we check for interrupt that triggers both edge on CAP0 signal and according take action
 */
static void IRAM_ATTR isr_handler(void)
{
    uint32_t mcpwm_intr_status;
    uint32_t capture_signal;
    mcpwm_intr_status = MCPWM[MCPWM_UNIT_0]->int_st.val; //Read interrupt status
    if (mcpwm_intr_status & CAP0_INT_EN) { //Check for interrupt on both edge on CAP0 signal
        capture_signal = mcpwm_capture_signal_get_value(MCPWM_UNIT_0, MCPWM_SELECT_CAP0); //get capture signal counter value
        xQueueSendFromISR(cap_queue, &capture_signal, NULL);
    }
    MCPWM[MCPWM_UNIT_0]->int_clr.val = mcpwm_intr_status;
}

/**
 * @brief Configuring Input Capture Module
 */
static void capture_init(void *arg)
{
    //mcpwm gpio initialization
    printf("initializing mcpwm gpio...\n");
    mcpwm_pin_config_t pin_config = {
        .mcpwm_cap0_in_num   = GPIO_CAP0_IN,
    };
    mcpwm_set_pin(MCPWM_UNIT_0, &pin_config); // Setting CAP0 pin
    gpio_pulldown_en(GPIO_CAP0_IN); //Enable pull down on CAP0 signal
    //Capture configuration
    //configure CAP0 signal to start capture counter on both edge
    mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP0, MCPWM_BOTH_EDGE, 0);  //capture signal on rising edge, prescale = 0 i.e. 800,000,000 counts is equal to one second
    //enable interrupt, so each this a rising edge occurs interrupt is triggered
    MCPWM[MCPWM_UNIT_0]->int_ena.val = CAP0_INT_EN;  //Enable interrupt on  CAP0, CAP1 and CAP2 signal
    mcpwm_isr_register(MCPWM_UNIT_0, isr_handler, NULL, ESP_INTR_FLAG_IRAM, NULL);  //Set ISR Handler

    gpio_set_direction(LED_PWM, GPIO_MODE_INPUT_OUTPUT); // Captured sequence led
    // Capture window indicator LED Setup
    gpio_set_direction(LED_TIMER, GPIO_MODE_INPUT_OUTPUT); // Setting GPIO26 as both input and output
    TimerHandle_t wTimer = xTimerCreate("Led Timer", (LED_DELAY/portTICK_PERIOD_MS) , pdTRUE, ( void * ) 1, LED_Toggle_wTimer);
    xTimerStart(wTimer, 0); // Initiating timer
    vTaskDelete(NULL);
}

void app_main(void)
{
    printf("Testing MCPWM...\n");
    cap_queue = xQueueCreate(1, sizeof(uint32_t)); // Creating queue
    xTaskCreate(disp_captured_signal, "mcpwm_config", 4096, NULL, 5, NULL); // Captured signal task
    xTaskCreate(capture_init, "capture_init", 4096, NULL, 5, NULL); // Initialize task
}