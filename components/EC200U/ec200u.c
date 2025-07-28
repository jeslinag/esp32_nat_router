#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h>

#define EC200U_UART_NUM       UART_NUM_2
#define EC200U_TX_PIN         17
#define EC200U_RX_PIN         18
#define EC200U_PWRKEY_PIN     42
#define EC200U_RESET_PIN      41
#define EC200U_UART_BUF_SIZE  (2048)

static const char *TAG = "EC200U_MODEM";

// UART and GPIO initialization
void ec200u_uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    ESP_ERROR_CHECK(uart_param_config(EC200U_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(EC200U_UART_NUM, EC200U_TX_PIN, EC200U_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(EC200U_UART_NUM, EC200U_UART_BUF_SIZE, EC200U_UART_BUF_SIZE, 0, NULL, 0));
}

// PWRKEY and RESET GPIO initialization and control
void ec200u_gpio_init(void)
{
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << EC200U_PWRKEY_PIN) | (1ULL << EC200U_RESET_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    gpio_set_level(EC200U_PWRKEY_PIN, 0);
    gpio_set_level(EC200U_RESET_PIN, 1);
}

void ec200u_power_on(void)
{
    // Toggle PWRKEY: High for 1 sec, then Low (see EC200U hardware guide)
    gpio_set_level(EC200U_PWRKEY_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(1100));
    gpio_set_level(EC200U_PWRKEY_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(5000)); // Wait for module to boot up
}

void ec200u_reset(void)
{
    gpio_set_level(EC200U_RESET_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(200));
    gpio_set_level(EC200U_RESET_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(3000)); // Wait for reset
}

// Send an AT command and read response (basic synchronous example; consider making a separate task/queue for robust AT command handling)
esp_err_t ec200u_send_at_cmd(const char *cmd, char *response, size_t response_size, TickType_t timeout_ticks)
{
    // Send command with CRLF
    char cmd_buf[64];
    snprintf(cmd_buf, sizeof(cmd_buf), "%s\r\n", cmd);

    uart_flush_input(EC200U_UART_NUM);
    uart_write_bytes(EC200U_UART_NUM, cmd_buf, strlen(cmd_buf));

    // Read response into buffer
    int len = uart_read_bytes(EC200U_UART_NUM, (uint8_t *)response, response_size - 1, timeout_ticks);
    if (len > 0) {
        response[len] = 0; // Null-terminate
        ESP_LOGI(TAG, "AT CMD: %s RESPONSE: %s", cmd, response);
        return ESP_OK;
    }
    return ESP_FAIL;
}

// Example: Run once in a task after power on
void ec200u_modem_task(void *arg)
{
    char resp[256];
    ec200u_uart_init();
    ec200u_gpio_init();
    ec200u_power_on();

    // Test communication
    if (ec200u_send_at_cmd("AT", resp, sizeof(resp), pdMS_TO_TICKS(1000)) == ESP_OK) {
        if (strstr(resp, "OK")) {
            ESP_LOGI(TAG, "EC200U is responsive.");
        } else {
            ESP_LOGW(TAG, "EC200U invalid response.");
        }
    } else {
        ESP_LOGE(TAG, "No response from EC200U.");
    }

    // Continue with LTE setup: AT+CPIN?, AT+CSQ, PDP context, etc.

    vTaskDelete(NULL);
}

// In your main app/router startup, add:
void start_ec200u_modem()
{
    xTaskCreate(ec200u_modem_task, "ec200u_modem_task", 4096, NULL, 10, NULL);
}
