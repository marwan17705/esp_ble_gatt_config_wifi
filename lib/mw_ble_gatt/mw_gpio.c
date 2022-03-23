#include "driver/gpio.h"
// #include "esp_event.h"
// #include "esp_timer.h"
// #include "freertos/task.h"
#include "mw_gpio.h"
#include "freertos/timers.h"

int64_t timestamp = 0 ;
bool event_op = 0;
bool state_wifi = 0 ;
esp_event_loop_handle_t loop_with_task;
TimerHandle_t timer_instance;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    if(gpio_get_level(gpio_num) == 0)
        timestamp = esp_timer_get_time();
    else
    {
        TIMER_EVENT evt;
        int16_t dif= esp_timer_get_time()/1000 - timestamp/1000;
        if(dif > HOLD_PERIOD)
        {
            evt = TASK_TIMER_OVER;
            // ets_printf("upper %ld\n" ,dif);
            if(event_op){
                esp_err_t ret = (esp_event_post_to(loop_with_task, GPIO_EVENT, INTERUPT_EVENTS, &evt, sizeof(evt), portMAX_DELAY));
                if(ret)
                    ets_printf("err becouse %s\n",esp_err_to_name(ret) );
            }
        }
        else
        {
            evt = TASK_TIMER_INTER;
            
            if(event_op){
                // ets_printf("under %ld\n", dif);
                esp_err_t ret = (esp_event_post_to(loop_with_task, GPIO_EVENT, INTERUPT_EVENTS, &evt, sizeof(evt), portMAX_DELAY));
                if(ret)
                    ets_printf("err becouse %s\n",esp_err_to_name(ret));
            }
        }
    }

}

void light_adv_on()
{
    gpio_set_level(GPIO_OUTPUT_IO_0, false);
}

void light_adv_off()
{
    gpio_set_level(GPIO_OUTPUT_IO_0, true);
}

// void light_wifi_on()
// {
//     gpio_set_level(GPIO_OUTPUT_IO_1, false);
// }

// void light_wifi_off()
// {
//     gpio_set_level(GPIO_OUTPUT_IO_1, true);
// }

esp_err_t register_event_gpio(esp_event_handler_t task_iteration_handler)
{
    esp_event_loop_args_t loop_with_task_args = {
        .queue_size = 5,
        .task_name = "gpio_task", // task will be created
        .task_priority = uxTaskPriorityGet(NULL),
        .task_stack_size = 1024*4,
        .task_core_id = tskNO_AFFINITY};
    esp_err_t ret = esp_event_loop_create(&loop_with_task_args, &loop_with_task);
    event_op = 1;
    if(ret)
        return ret;
    return esp_event_handler_instance_register_with(loop_with_task, GPIO_EVENT, INTERUPT_EVENTS, task_iteration_handler, loop_with_task, NULL);

}

void blink_cb(void *timer)
{
    state_wifi = !state_wifi;
    printf(" state %d\n",state_wifi);
    gpio_set_level(GPIO_OUTPUT_IO_0, state_wifi);
}



void setup_pin()
{
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO12
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    light_adv_off();

    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    //bit mask of the pins, use GPI14 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    //change gpio intrrupt type for one pin
    gpio_set_intr_type(GPIO_INPUT_IO_0, GPIO_INTR_ANYEDGE);

    // install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);

    //remove isr handler for gpio number.
    gpio_isr_handler_remove(GPIO_INPUT_IO_0);
    //hook isr handler for specific gpio pin again
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);

    printf("Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());

}

void start_blink()
{
    timer_instance = xTimerCreate("blink_cb", 1000 / portTICK_RATE_MS,
                    true, NULL, blink_cb);
    xTimerStart(timer_instance, 0);
}

void stop_blink()
{
    light_adv_off();
    xTimerStop(timer_instance,0);
}