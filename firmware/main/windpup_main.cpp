#include <cstdio>
#include <cstring>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/uart.h"

constexpr uart_port_t modem_uart = UART_NUM_0;
constexpr int modem_uart_bufsize = (1024 * 2); // 2KiB rx, tx buffers
static QueueHandle_t modem_uart_queue;

void init_uart() {
    uart_config_t modem_uart_cfg = {
        .baud_rate = 300,
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

extern "C" void app_main() {
    printf("w o o f\n");

    init_uart();

    while (1) {
        const char* test_str = "fucky wucky woof bark bark\n";
        uart_write_bytes(modem_uart, test_str, strlen(test_str));
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
