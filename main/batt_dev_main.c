//TODO -> send signal strength as well... (RSSI)

// derrived from ds18b20-ulp-2, added batt_dev_1 code
// https://github.com/fhng/ESP32-ULP-1-Wire.git

// DFROBOT board
// https://wiki.dfrobot.com/FireBeetle_ESP32_IOT_Microcontroller(V3.0)__Supports_Wi-Fi_&_Bluetooth__SKU__DFR0478

/* BATTERY POWERED ESP32

- wake up
- take sensor reading
- send to gateway over espnow --> broadcast to all gateways (broadcast address = ff:ff:ff:ff:ff:ff)
- deepsleep
*/

// SHIFT + ALT + F

#define MEASUREMENT_INTERVAL_SECONDS 9 // should subtract +-0.75 seconds for temp conversion?
// #define MEASUREMENT_INTERVAL_SECONDS 300 // should subtract +-0.75 seconds for temp conversion?

#include <stdio.h>
#include "esp_sleep.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "soc/soc.h"
#include "soc/rtc_io_reg.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp32/ulp.h"
#include "ulp_main.h"

//! batt_dev_1 stuff

#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_now.h"
#include "esp_crc.h"
#include "espnow_batt_dev.h"

#include <driver/adc.h> // for reading bat voltage

#define MIN(x, y) (((x) < (y)) ? (x) : (y))

static const char *TAG = "espnow_batt_dev";

static xQueueHandle s_example_espnow_queue;

static uint8_t s_example_broadcast_mac[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // broadcast (to all devices)

static void example_espnow_deinit(example_espnow_send_param_t *send_param);

SemaphoreHandle_t can_sleep_sema = NULL;

//! end batt_dev_1 stuff

RTC_DATA_ATTR uint32_t ptime_ms;

extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[] asm("_binary_ulp_main_bin_end");

gpio_num_t one_wire_port = GPIO_NUM_27;

gpio_num_t led_pin = GPIO_NUM_2; // DFROBOT

static void init_ulp_program();

//! batt_dev_1 stuff
/* WiFi should start before using ESPNOW */
static void example_wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(ESPNOW_WIFI_MODE));
    ESP_ERROR_CHECK(esp_wifi_start());

#if CONFIG_ESPNOW_ENABLE_LONG_RANGE
    ESP_ERROR_CHECK(esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR));
#endif
}

/* ESPNOW sending or receiving callback function is called in WiFi task.
 * Users should not do lengthy operations from this task. Instead, post
 * necessary data to a queue and handle it from a lower priority task. */
static void example_espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    example_espnow_event_t evt;
    example_espnow_event_send_cb_t *send_cb = &evt.info.send_cb;

    if (mac_addr == NULL)
    {
        ESP_LOGE(TAG, "Send cb arg error");
        return;
    }

    evt.id = EXAMPLE_ESPNOW_SEND_CB;
    memcpy(send_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    send_cb->status = status;
    if (xQueueSend(s_example_espnow_queue, &evt, portMAX_DELAY) != pdTRUE)
    {
        ESP_LOGW(TAG, "Send send queue fail");
    }
}

static void example_espnow_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len)
{
    example_espnow_event_t evt;
    example_espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;

    if (mac_addr == NULL || data == NULL || len <= 0)
    {
        ESP_LOGE(TAG, "Receive cb arg error");
        return;
    }

    evt.id = EXAMPLE_ESPNOW_RECV_CB;
    memcpy(recv_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    recv_cb->data = malloc(len);
    if (recv_cb->data == NULL)
    {
        ESP_LOGE(TAG, "Malloc receive data fail");
        return;
    }
    memcpy(recv_cb->data, data, len);
    recv_cb->data_len = len;
    if (xQueueSend(s_example_espnow_queue, &evt, portMAX_DELAY) != pdTRUE)
    {
        ESP_LOGW(TAG, "Send receive queue fail");
        free(recv_cb->data);
    }
}

void fill_zero(void *buf, size_t len)
{
    assert(buf != NULL);
    uint8_t *buf_bytes = (uint8_t *)buf;
    while (len > 0)
    {
        uint32_t word = 0;
        uint32_t to_copy = MIN(sizeof(word), len);
        memcpy(buf_bytes, &word, to_copy);
        buf_bytes += to_copy;
        len -= to_copy;
    }
}

/* Prepare ESPNOW data to be sent. */
void example_espnow_data_prepare(example_espnow_send_param_t *send_param)
{
    example_espnow_data_t *buf = (example_espnow_data_t *)send_param->buffer;

    assert(send_param->len >= sizeof(example_espnow_data_t));

    buf->crc = 0;

    fill_zero(buf->payload, send_param->len - sizeof(example_espnow_data_t));

    // what to send:
    /*
    1. temperature value (4 bytes, int32_t)
    2. how long the last wake ran for (4 bytes, uint32_t)
    3. battery voltage (4 bytes, uint32_t) <-- can be less but jsut keep it long for future dev

    total length = 2 for crc? + 4 for temp + 4 for time + 4 for batt
    = 14

    payload length = 12
    */

    //* get temp value here
    float temperatureC = 0.0;
    temperatureC = ulp_temperatureC & UINT16_MAX;
    ESP_LOGI(TAG, "temp: %.4f", temperatureC / 16);
    printf("temp: %.4f\n", temperatureC / 16);
    uint32_t temp = ulp_temperatureC & UINT16_MAX; // need to divide by 16 to get temp in deg. C

    // get batmv value here
    int adc_val = adc1_get_raw(ADC1_CHANNEL_0);
    int batmv = (int)(adc_val * 3300.0 / 4095.0) * 2; // x2 because there is a voltage divider

    // reserved for crc??
    // send_param->buffer[0] = 1;
    // send_param->buffer[1] = 2;

    // temp
    for (int i = 0; i < 4; i++)
    {
        buf->payload[i] = ((temp >> (i * 8)) & 0xff); //extract the right-most byte of the shifted variable
    }

    // PREVIOUS program time
    for (int i = 4; i < 8; i++)
    {
        buf->payload[i] = ((ptime_ms >> (i * 8)) & 0xff); //extract the right-most byte of the shifted variable
    }
    ESP_LOGI(TAG, "ptime_ms: %d", ptime_ms);
    printf("ptime_ms: %d\n", ptime_ms);

    // bat voltage
    for (int i = 8; i < 12; i++)
    {
        buf->payload[i] = ((batmv >> (i * 8)) & 0xff); //extract the right-most byte of the shifted variable
    }
    ESP_LOGI(TAG, "batmv: %d", batmv);
    printf("batmv: %d\n", batmv);

    buf->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, send_param->len);

    //* Don't delete... use to print buffer to send
    // for (uint32_t i = 0; i < 12; i++)
    // {
    //     printf("payload[%d]: %d\n", i, buf->payload[i]);
    // }
    //* --------------------------------------------
}

static void main_espnow_task(void *pvParameter)
{
    xSemaphoreTake(can_sleep_sema, portMAX_DELAY); // take semaphore so esp doesn't sleep yet (give at the end of task)

    ESP_LOGI(TAG, "Send sensor data");

    /* Start sending broadcast ESPNOW data. */
    example_espnow_send_param_t *send_param = (example_espnow_send_param_t *)pvParameter;

    if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK)
    {
        ESP_LOGE(TAG, "Send error");
        example_espnow_deinit(send_param);
        vTaskDelete(NULL);
    }

    xSemaphoreGive(can_sleep_sema); // give back semaphore so the esp can sleep

    vTaskDelete(NULL);
}

static esp_err_t main_espnow_init()
{
    example_espnow_send_param_t *send_param;

    s_example_espnow_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(example_espnow_event_t));
    if (s_example_espnow_queue == NULL)
    {
        ESP_LOGE(TAG, "Create mutex fail");
        return ESP_FAIL;
    }

    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(example_espnow_send_cb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(example_espnow_recv_cb));

    /* Set primary master key. */
    ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK));

    /* Add broadcast peer information to peer list. */
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL)
    {
        ESP_LOGE(TAG, "Malloc peer information fail");
        vSemaphoreDelete(s_example_espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = CONFIG_ESPNOW_CHANNEL;
    peer->ifidx = ESPNOW_WIFI_IF;
    peer->encrypt = false;
    memcpy(peer->peer_addr, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK(esp_now_add_peer(peer));
    free(peer);

    /* Initialize sending parameters. */
    send_param = malloc(sizeof(example_espnow_send_param_t));
    memset(send_param, 0, sizeof(example_espnow_send_param_t));
    if (send_param == NULL)
    {
        ESP_LOGE(TAG, "Malloc send parameter fail");
        vSemaphoreDelete(s_example_espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }

    uint8_t length = 14;

    send_param->len = length;
    send_param->buffer = malloc(length);

    if (send_param->buffer == NULL)
    {
        ESP_LOGE(TAG, "Malloc send buffer fail");
        free(send_param);
        vSemaphoreDelete(s_example_espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }

    memcpy(send_param->dest_mac, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);
    example_espnow_data_prepare(send_param);

    BaseType_t xReturned = xTaskCreate(main_espnow_task, "main_espnow_task", 2048, send_param, 4, NULL);

    if (xReturned == pdPASS)
    {
        return ESP_OK;
    }
    else
    {
        return ESP_FAIL;
    }
}

static void example_espnow_deinit(example_espnow_send_param_t *send_param)
{
    free(send_param->buffer);
    free(send_param);
    vSemaphoreDelete(s_example_espnow_queue);
    esp_now_deinit();
}
//! end batt_dev_1 stuff

void app_main(void)
{
    esp_log_level_set(TAG, ESP_LOG_INFO);

    //* led stuff
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = 1ULL << led_pin; // only one pin, otherwise look at this eg: https://github.com/espressif/esp-idf/blob/a20df743f1c51e6d65b021ed2ffd3081a2feec64/examples/peripherals/gpio/generic_gpio/main/gpio_example_main.c
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    // set led low and hold (not sure how much hold helps...)

    gpio_set_level(led_pin, 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    gpio_set_level(led_pin, 0);
    gpio_hold_en(led_pin);
    //* end led stuff

    // //* batmv adc stuff
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11); // GPIO_NUM_36
    //* end batmv adc stuff

    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    if (cause != ESP_SLEEP_WAKEUP_ULP)
    {
        printf("Not ULP wakeup, initializing ULP\n");
        init_ulp_program();
        ptime_ms = 0;
    }
    else
    {
        //! batt_dev_1 stuff

        // Initialize NVS
        esp_err_t ret = nvs_flash_init();
        if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
        {
            ESP_ERROR_CHECK(nvs_flash_erase());
            ret = nvs_flash_init();
        }
        ESP_ERROR_CHECK(ret);

        example_wifi_init();

        can_sleep_sema = xSemaphoreCreateBinary();
        xSemaphoreGive(can_sleep_sema);

        main_espnow_init();

        vTaskDelay(1); //! not sure why I need this... but main_espnow_task doesn't run without it

        xSemaphoreTake(can_sleep_sema, portMAX_DELAY);

        //! end batt_dev_1 stuff
    }

    ESP_ERROR_CHECK(ulp_set_wakeup_period(0, MEASUREMENT_INTERVAL_SECONDS * 1000000));
    ESP_ERROR_CHECK(esp_sleep_enable_ulp_wakeup());

    ptime_ms = (uint32_t)esp_timer_get_time(); // could be an issue with int64_t to uint32_t?

    ESP_LOGI(TAG, "ptime_ms = %d\nSleeping.", ptime_ms);
    printf("ptime_ms = %d\nSleeping.\n", ptime_ms);

    // set blue led off when in deep sleep
    rtc_gpio_set_level(led_pin, 0); //GPIO_NUM_2
    rtc_gpio_hold_en(led_pin);

    esp_deep_sleep_start();
}

static void init_ulp_program()
{
    esp_err_t err = ulp_load_binary(0, ulp_main_bin_start,
                                    (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t));
    ESP_ERROR_CHECK(err);

    /* Initialize some variables used by ULP program.
     * Each 'ulp_xyz' variable corresponds to 'xyz' variable in the ULP program.
     * These variables are declared in an auto generated header file,
     * 'ulp_main.h', name of this file is defined in component.mk as ULP_APP_NAME.
     * These variables are located in RTC_SLOW_MEM and can be accessed both by the
     * ULP and the main CPUs.
     *
     * Note that the ULP reads only the lower 16 bits of these variables.
     */

    rtc_gpio_init(one_wire_port);
    // ESP-IDF has an typo error on RTC_GPIO_MODE_INPUT_OUTUT, missing P, fixed
    // Open drain mode(1-wire/ I2C), should get it to INPUT_ONLY
    rtc_gpio_set_direction(one_wire_port, RTC_GPIO_MODE_INPUT_ONLY);

    /* Set ULP wake up period to T = 20ms (3095 cycles of RTC_SLOW_CLK clock).
     * Minimum pulse width has to be T * (ulp_debounce_counter + 1) = 80ms.
     */
    // REG_SET_FIELD(SENS_ULP_CP_SLEEP_CYC0_REG, SENS_SLEEP_CYCLES_S0, 3095);
    // REG_SET_FIELD(SENS_ULP_CP_SLEEP_CYC0_REG, SENS_SLEEP_CYCLES_S0, 150000); //! this one was uncommented
    // REG_SET_FIELD(SENS_ULP_CP_SLEEP_CYC0_REG, SENS_SLEEP_CYCLES_S0, 1000000); //! this one was uncommented

    /* Start the program */
    err = ulp_run(&ulp_entry - RTC_SLOW_MEM);
    ESP_ERROR_CHECK(err);
}
