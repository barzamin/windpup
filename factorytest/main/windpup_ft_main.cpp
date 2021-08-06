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
    ESP_ERROR_CHECK(uart_set_pin(modem_uart, /* tx */ 21, /* rx */ 20, /* rts */ 0, /* cts */ 0));

    ESP_ERROR_CHECK(uart_driver_install(modem_uart, modem_uart_bufsize, modem_uart_bufsize, 10, &modem_uart_queue, 0));
}

void init_gpio() {
    gpio_config_t outputs;
    outputs.intr_type = GPIO_INTR_DISABLE;
    outputs.mode = GPIO_MODE_OUTPUT;
    outputs.pin_bit_mask = /* WUF */ (1ULL << 8) | /* RING */ (1ULL << 0) | /* RTS */ (1ULL << 3) | /* DCE */ (1ULL << 2);
    outputs.pull_down_en = GPIO_PULLDOWN_DISABLE;
    outputs.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&outputs);
}

void wait_nl() {
    char c = 0;
    for(;;) {
        if (uart_read_bytes(modem_uart, &c, 1, 20/portTICK_RATE_MS) && (c == '\n'))
            break;
    }
}

extern "C" void app_main() {
    printf("w o o f\n");

    init_gpio();
    init_uart();

    // gpio_set_level(GPIO_NUM_8, 0); // WUF high
    // gpio_set_level(GPIO_NUM_0, 1); // RING low
    // gpio_set_level(GPIO_NUM_3, 1); // RTS low
    // gpio_set_level(GPIO_NUM_2, 1); // DCE low

    uint16_t lvl= 0;
    while(true) {
        gpio_set_level(GPIO_NUM_0, lvl); // DCE low
        gpio_set_level(GPIO_NUM_8, lvl); // DCE low
        gpio_set_level(GPIO_NUM_2, lvl); // DCE low
        gpio_set_level(GPIO_NUM_3, lvl); // DCE low
        lvl ^= 1;
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    char c = 0;
    while (true) {
        if (uart_read_bytes(modem_uart, &c, 1, 20 / portTICK_RATE_MS))
            uart_write_bytes(modem_uart, &c, 1);
        if (c == '\n') break;
    }

    wait_nl();
    printf("set rts\n");
    gpio_set_level(GPIO_NUM_3, 0);

    wait_nl();
    printf("set dce\n");
    gpio_set_level(GPIO_NUM_2, 0);

    wait_nl();
    printf("set ring\n");
    gpio_set_level(GPIO_NUM_0, 0);

    vTaskDelay(100 / portTICK_PERIOD_MS);
    printf("🔄 restart\n");
    esp_restart();
}
