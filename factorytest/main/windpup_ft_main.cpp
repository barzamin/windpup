#include <cstdio>
#include <cstring>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/uart.h"
#include "driver/gpio.h"

constexpr uart_port_t modem_uart = UART_NUM_0;
constexpr int modem_uart_bufsize = (1024 * 2); // 2KiB rx, tx buffers
static QueueHandle_t modem_uart_queue;

// TODO: store in flash
#define WIFI_SSID CONFIG_WIFI_SSID
#define WIFI_PASS CONFIG_WIFI_PASS
#define WIFI_RETRIES CONFIG_MAX_WIFI_RETRY

static EventGroupHandle_t sig_wifi_event_grp;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

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

extern "C" void app_main() {
    printf("w o o f\n");

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
