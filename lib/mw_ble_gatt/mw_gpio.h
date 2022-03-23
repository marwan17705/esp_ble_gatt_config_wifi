#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_event.h"


ESP_EVENT_DECLARE_BASE(GPIO_EVENT); // declaration of the task events family


#define GPIO_OUTPUT_IO_0    12
// #define GPIO_OUTPUT_IO_1    2
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_IO_0)/*||(1ULL<<GPIO_OUTPUT_IO_1)*/)

#define GPIO_INPUT_IO_0     14
#define GPIO_INPUT_PIN_SEL  (1ULL<<GPIO_INPUT_IO_0) 
#define ESP_INTR_FLAG_DEFAULT 10
//hold to change mode 
#define HOLD_PERIOD 1000
// bool blink =0;
enum
{
    INTERUPT_EVENTS, 
};

typedef enum
{
    TASK_TIMER_OVER,
    TASK_TIMER_INTER
} TIMER_EVENT;

void setup_pin();
esp_err_t register_event_gpio(esp_event_handler_t task_iteration_handler);

void light_adv_on();

void light_adv_off();

// void light_wifi_on();

// void light_wifi_off();

void start_blink();

void stop_blink();
