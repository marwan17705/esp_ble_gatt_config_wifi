#include <string.h>
#include <stdint.h>
#include <stdbool.h>




#define EXAMPLE_ESP_MAXIMUM_RETRY  10


/* FreeRTOS event group to signal when we are connected*/


/* The event group allows multiple bits for each event, but we only care about two events:
    * - we are connected to the AP with an IP
    * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1


//  wifi_config_t wifi_config;

/**
 * @brief  this fucntion is used for initial wifi.
 * 
 * - ssid:  ssid of wifi
 * - pwd: password of wifi
 */
bool wifi_init_sta();

/**
 * @brief 
 * 
 * 
 */
bool wifi_start_sta(void);

/**
 * @brief 
 * 
 * - ssid:  ssid of wifi
 * - pwd: password of wifi
 */
esp_err_t store_wifi_config(char *ssid,char *pwd);


/**
 * @brief 
 * 
 */
esp_err_t deinit_wifi();

/**
 * @brief 
 * 
 */
bool is_wifi_start();