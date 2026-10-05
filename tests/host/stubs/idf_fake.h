/* GdoLib - A library for controlling garage door openers.
 * Copyright (C) 2024  Konnected Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Minimal host-side stand-in for the ESP-IDF / FreeRTOS APIs gdolib uses, so the real
 * gdo.c can be compiled and exercised on a development machine. Only the behaviour the
 * tests depend on is modelled: a byte-accurate UART RX ring buffer, FIFO queues, and a
 * clock that fires due esp_timers as it advances. Tasks are recorded but never run.
 */

#ifndef IDF_FAKE_H
#define IDF_FAKE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

/* ---- esp_err ---- */
typedef int esp_err_t;
#define ESP_OK                  0
#define ESP_FAIL                -1
#define ESP_ERR_NO_MEM          0x101
#define ESP_ERR_INVALID_ARG     0x102
#define ESP_ERR_INVALID_STATE   0x103
#define ESP_ERR_NOT_FOUND       0x105
#define ESP_ERR_NOT_SUPPORTED   0x106
#define ESP_ERR_TIMEOUT         0x107
#define ESP_ERR_NOT_FINISHED    0x10C
const char *esp_err_to_name(esp_err_t err);

/* ---- esp_log ---- */
extern bool fake_log_enabled;
#define FAKE_LOG(lvl, tag, fmt, ...) \
    do { if (fake_log_enabled) printf("%s (%s) " fmt "\n", lvl, tag, ##__VA_ARGS__); } while (0)
#define ESP_LOGE(tag, fmt, ...) FAKE_LOG("E", tag, fmt, ##__VA_ARGS__)
#define ESP_LOGW(tag, fmt, ...) FAKE_LOG("W", tag, fmt, ##__VA_ARGS__)
#define ESP_LOGI(tag, fmt, ...) FAKE_LOG("I", tag, fmt, ##__VA_ARGS__)
#define ESP_LOGD(tag, fmt, ...) FAKE_LOG("D", tag, fmt, ##__VA_ARGS__)
#define ESP_LOGV(tag, fmt, ...) FAKE_LOG("V", tag, fmt, ##__VA_ARGS__)

#define IRAM_ATTR

/* ---- FreeRTOS ---- */
typedef int BaseType_t;
typedef uint32_t TickType_t;
typedef void *TaskHandle_t;
typedef struct fake_queue *QueueHandle_t;
typedef int portMUX_TYPE;
#define pdTRUE  1
#define pdFALSE 0
#define pdPASS  1
#define portMAX_DELAY ((TickType_t)0xffffffffUL)
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(mux) ((void)(mux))
#define portEXIT_CRITICAL(mux) ((void)(mux))

QueueHandle_t xQueueCreate(uint32_t length, uint32_t item_size);
void vQueueDelete(QueueHandle_t q);
BaseType_t xQueueSend(QueueHandle_t q, const void *item, TickType_t ticks);
BaseType_t xQueueSendToBack(QueueHandle_t q, const void *item, TickType_t ticks);
/* Receiving from an empty queue with portMAX_DELAY would block forever on a device; here it
 * unwinds back to fake_run_until_idle() instead, which is how a test regains control from
 * the never-returning gdo_main_task(). */
BaseType_t xQueueReceive(QueueHandle_t q, void *item, TickType_t ticks);
BaseType_t xQueueReset(QueueHandle_t q);
BaseType_t xTaskCreate(void (*fn)(void *), const char *name, uint32_t stack, void *arg,
                       uint32_t prio, TaskHandle_t *handle);
void vTaskDelete(TaskHandle_t task);
void vTaskDelay(TickType_t ticks);
BaseType_t xTaskNotifyGive(TaskHandle_t task);
uint32_t ulTaskNotifyTake(BaseType_t clear, TickType_t ticks);

/* ---- esp_timer ---- */
typedef struct fake_timer *esp_timer_handle_t;
typedef enum { ESP_TIMER_TASK } esp_timer_dispatch_t;
typedef struct {
    void (*callback)(void *arg);
    void *arg;
    esp_timer_dispatch_t dispatch_method;
    const char *name;
    bool skip_unhandled_events;
} esp_timer_create_args_t;
int64_t esp_timer_get_time(void);
esp_err_t esp_timer_create(const esp_timer_create_args_t *args, esp_timer_handle_t *out);
esp_err_t esp_timer_start_once(esp_timer_handle_t t, uint64_t us);
esp_err_t esp_timer_start_periodic(esp_timer_handle_t t, uint64_t us);
esp_err_t esp_timer_stop(esp_timer_handle_t t);
esp_err_t esp_timer_delete(esp_timer_handle_t t);

/* ---- gpio ---- */
typedef int gpio_num_t;
#define GPIO_NUM_MAX 49
typedef enum { GPIO_MODE_INPUT = 1 } gpio_mode_t;
typedef enum { GPIO_PULLUP_DISABLE = 0, GPIO_PULLUP_ENABLE } gpio_pullup_t;
typedef enum { GPIO_PULLDOWN_DISABLE = 0, GPIO_PULLDOWN_ENABLE } gpio_pulldown_t;
typedef enum { GPIO_INTR_DISABLE = 0, GPIO_INTR_NEGEDGE = 2 } gpio_int_type_t;
typedef struct {
    uint64_t pin_bit_mask;
    gpio_mode_t mode;
    gpio_pullup_t pull_up_en;
    gpio_pulldown_t pull_down_en;
    gpio_int_type_t intr_type;
} gpio_config_t;
esp_err_t gpio_config(const gpio_config_t *cfg);
esp_err_t gpio_reset_pin(gpio_num_t pin);
int gpio_get_level(gpio_num_t pin);
esp_err_t gpio_install_isr_service(int flags);
esp_err_t gpio_isr_handler_add(gpio_num_t pin, void (*isr)(void *), void *arg);

/* ---- uart ---- */
typedef int uart_port_t;
#define UART_NUM_MAX 3
#define UART_PIN_NO_CHANGE (-1)
typedef enum {
    UART_DATA,
    UART_BREAK,
    UART_BUFFER_FULL,
    UART_FIFO_OVF,
    UART_FRAME_ERR,
    UART_PARITY_ERR,
    UART_DATA_BREAK,
    UART_PATTERN_DET,
    UART_EVENT_MAX,
} uart_event_type_t;
typedef struct {
    uart_event_type_t type;
    size_t size;
    bool timeout_flag;
} uart_event_t;
typedef enum { UART_DATA_8_BITS = 3 } uart_word_length_t;
typedef enum { UART_PARITY_DISABLE = 0, UART_PARITY_EVEN = 2 } uart_parity_t;
typedef enum { UART_STOP_BITS_1 = 1 } uart_stop_bits_t;
typedef enum { UART_HW_FLOWCTRL_DISABLE = 0 } uart_hw_flowcontrol_t;
typedef enum { UART_SCLK_DEFAULT = 0 } uart_sclk_t;
#define UART_SIGNAL_RXD_INV (1 << 2)
#define UART_SIGNAL_TXD_INV (1 << 5)
typedef struct {
    int baud_rate;
    uart_word_length_t data_bits;
    uart_parity_t parity;
    uart_stop_bits_t stop_bits;
    uart_hw_flowcontrol_t flow_ctrl;
    uint8_t rx_flow_ctrl_thresh;
    uart_sclk_t source_clk;
} uart_config_t;
esp_err_t uart_param_config(uart_port_t num, const uart_config_t *cfg);
esp_err_t uart_set_pin(uart_port_t num, int tx, int rx, int rts, int cts);
esp_err_t uart_set_line_inverse(uart_port_t num, uint32_t mask);
esp_err_t uart_driver_install(uart_port_t num, int rx_size, int tx_size, int queue_size,
                              QueueHandle_t *queue, int flags);
esp_err_t uart_driver_delete(uart_port_t num);
esp_err_t uart_set_baudrate(uart_port_t num, uint32_t baud);
esp_err_t uart_set_parity(uart_port_t num, uart_parity_t parity);
int uart_read_bytes(uart_port_t num, void *buf, uint32_t len, TickType_t ticks);
int uart_write_bytes(uart_port_t num, const void *buf, size_t len);
esp_err_t uart_wait_tx_done(uart_port_t num, TickType_t ticks);
esp_err_t uart_flush(uart_port_t num);
esp_err_t uart_flush_input(uart_port_t num);

/* ---- test controls ---- */
void fake_reset(void);
void fake_set_time_ms(uint32_t ms);
/* Moves the clock forward to ms, firing every armed esp_timer that falls due on the way, in
 * deadline order and at its own deadline. Callbacks run synchronously. */
void fake_advance_to(uint32_t ms);
/* Fires only the earliest armed timer due at or before until_ms, moving the clock to its
 * deadline. Returns false if none was due. */
bool fake_fire_next_timer(uint32_t until_ms);
bool fake_timer_armed(esp_timer_handle_t t);
/* Appends bytes to the UART RX ring buffer, as if the opener had sent them. */
void fake_uart_rx(const uint8_t *bytes, size_t len);
size_t fake_uart_rx_available(void);
uint32_t fake_queue_depth(QueueHandle_t q);
extern int fake_uart_flush_count;
/* Writes of a whole Sec+ v2 frame; transmit_packet() flushes RX once after each. */
extern int fake_uart_frames_written;
/* Every uart_write_bytes() call, in order. A Sec+ v2 transmit is two writes: a single 0x00
 * byte (sent at 6900 baud to fake a break), then the 19-byte frame. */
#define FAKE_TX_LOG_MAX 128
typedef struct {
    uint8_t bytes[19];
    size_t len;
    uint32_t at_ms;
} fake_tx_t;
extern fake_tx_t fake_tx_log[FAKE_TX_LOG_MAX];
extern int fake_tx_count;
/* Runs fn (a never-returning task loop) until it blocks on an empty queue and fake_on_idle
 * (if set) declines to supply more work by returning false. */
void fake_run_until_idle(void (*fn)(void *), void *arg);
/* Called for every item successfully queued, so tests can observe e.g. queued commands. */
extern bool (*fake_on_idle)(void);
extern void (*fake_on_queue_send)(QueueHandle_t q, const void *item);

#endif // IDF_FAKE_H
