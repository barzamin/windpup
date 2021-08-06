#include <cstdio>
#include <cstring>

#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_spi_flash.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#include "lwip/err.h"
#include "lwip/sys.h"

constexpr uart_port_t modem_uart = UART_NUM_0;
constexpr int modem_uart_bufsize = (1024 * 2); // 2KiB rx, tx buffers
static QueueHandle_t modem_uart_queue;

// TODO: store in flash
#define WIFI_SSID CONFIG_WIFI_SSID
#define WIFI_PASS CONFIG_WIFI_PASS
#define WIFI_RETRIES CONFIG_WIFI_RETRIES

static EventGroupHandle_t sig_wifi_event_grp;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

namespace lt {
    static const char* sta = "wifi-sta";
}

void init_uart() {
    uart_config_t modem_uart_cfg = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, // TODO
        .rx_flow_ctrl_thresh = 0, // TODO
        .source_clk = UART_SCLK_APB,
    };

    ESP_ERROR_CHECK(uart_param_config(modem_uart, &modem_uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(modem_uart, /* tx */ 21, /* rx */ 20, /* rts */ 3, /* cts */ 10));

    ESP_ERROR_CHECK(uart_driver_install(modem_uart, modem_uart_bufsize, modem_uart_bufsize, 10, &modem_uart_queue, 0));
}

void init_gpio() {
    gpio_config_t woof_conf;
    woof_conf.intr_type = GPIO_INTR_DISABLE;
    woof_conf.mode = GPIO_MODE_OUTPUT;
    woof_conf.pin_bit_mask = 1ULL << 8; // IO8
    woof_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    woof_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&woof_conf);
}

static uint32_t n_wifi_retries = 0;
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (n_wifi_retries < WIFI_RETRIES) {
            esp_wifi_connect();
            n_wifi_retries++;
            ESP_LOGI(lt::sta, "retry %d for AP connection", n_wifi_retries);
        } else {
            xEventGroupSetBits(sig_wifi_event_grp, WIFI_FAIL_BIT);
        }
        ESP_LOGE(lt::sta, "AP connection failed");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(lt::sta, "sta got ip (from dhcp?):" IPSTR, IP2STR(&event->ip_info.ip));
        n_wifi_retries = 0;
        xEventGroupSetBits(sig_wifi_event_grp, WIFI_CONNECTED_BIT);
    }
}

void init_wifi_sta() {
    sig_wifi_event_grp = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config;
    memcpy(&wifi_config.sta.ssid, WIFI_SSID, sizeof(WIFI_SSID));
    memcpy(&wifi_config.sta.password, WIFI_PASS, sizeof(WIFI_PASS));
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(lt::sta, "finished station init");

    EventBits_t evt_bits = xEventGroupWaitBits(sig_wifi_event_grp,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        /* clear on exit */ pdFALSE,
        /* wait for all bits */ pdFALSE,
        portMAX_DELAY);

    if (evt_bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(lt::sta, "connected to ap { ssid = '%s', pass = '%s' }", WIFI_SSID, WIFI_PASS);
    } else if (evt_bits & WIFI_FAIL_BIT) {
        ESP_LOGW(lt::sta, "failed to connect to to ap { ssid = '%s', pass = '%s' }", WIFI_SSID, WIFI_PASS);
    } else {
        ESP_LOGE(lt::sta, "unexpected wifi event bits");
    }

    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(sig_wifi_event_grp);
}

extern "C" void app_main() {
    printf("w o o f\n");

    init_wifi_sta();
    init_gpio();
    init_uart();

    uint16_t lvl = 0;
    while (1) {
        const char* test_str = "woof bark\n";
        uart_write_bytes(modem_uart, test_str, strlen(test_str));

        gpio_set_level(GPIO_NUM_8, lvl);
        lvl ^= 1;

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
