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

#include "idf_fake.h"

#include <setjmp.h>
#include <string.h>

bool fake_log_enabled;
int fake_uart_flush_count;
int fake_uart_frames_written;
void (*fake_on_queue_send)(QueueHandle_t q, const void *item);
bool (*fake_on_idle)(void);

static uint32_t s_time_ms;
static jmp_buf s_idle_jmp;
static bool s_running;

#define FAKE_RX_CAP 4096
static uint8_t s_rx[FAKE_RX_CAP];
static size_t s_rx_head, s_rx_len;

const char *esp_err_to_name(esp_err_t err) {
    switch (err) {
    case ESP_OK: return "ESP_OK";
    case ESP_FAIL: return "ESP_FAIL";
    case ESP_ERR_NO_MEM: return "ESP_ERR_NO_MEM";
    case ESP_ERR_INVALID_ARG: return "ESP_ERR_INVALID_ARG";
    case ESP_ERR_INVALID_STATE: return "ESP_ERR_INVALID_STATE";
    case ESP_ERR_TIMEOUT: return "ESP_ERR_TIMEOUT";
    default: return "ESP_ERR_?";
    }
}

/* ---- queues ---- */
struct fake_queue {
    uint32_t length, item_size, head, count;
    uint8_t *items;
};

QueueHandle_t xQueueCreate(uint32_t length, uint32_t item_size) {
    QueueHandle_t q = calloc(1, sizeof(*q));
    q->length = length;
    q->item_size = item_size;
    q->items = calloc(length, item_size);
    return q;
}

void vQueueDelete(QueueHandle_t q) {
    if (q) {
        free(q->items);
        free(q);
    }
}

BaseType_t xQueueSendToBack(QueueHandle_t q, const void *item, TickType_t ticks) {
    (void)ticks;
    if (q->count == q->length) {
        return pdFALSE;
    }
    memcpy(q->items + ((q->head + q->count) % q->length) * q->item_size, item, q->item_size);
    q->count++;
    if (fake_on_queue_send) {
        fake_on_queue_send(q, item);
    }
    return pdTRUE;
}

BaseType_t xQueueSend(QueueHandle_t q, const void *item, TickType_t ticks) {
    return xQueueSendToBack(q, item, ticks);
}

BaseType_t xQueueReceive(QueueHandle_t q, void *item, TickType_t ticks) {
    while (q->count == 0) {
        if (ticks != portMAX_DELAY || !s_running) {
            return pdFALSE;
        }
        if (!fake_on_idle || !fake_on_idle()) {
            longjmp(s_idle_jmp, 1);
        }
    }
    memcpy(item, q->items + q->head * q->item_size, q->item_size);
    q->head = (q->head + 1) % q->length;
    q->count--;
    return pdTRUE;
}

BaseType_t xQueueReset(QueueHandle_t q) {
    q->head = q->count = 0;
    return pdPASS;
}

void fake_run_until_idle(void (*fn)(void *), void *arg) {
    if (setjmp(s_idle_jmp) == 0) {
        s_running = true;
        fn(arg);
    }
    s_running = false;
}

/* ---- tasks: never actually started ---- */
BaseType_t xTaskCreate(void (*fn)(void *), const char *name, uint32_t stack, void *arg,
                       uint32_t prio, TaskHandle_t *handle) {
    (void)fn; (void)name; (void)stack; (void)arg; (void)prio;
    if (handle) {
        *handle = (TaskHandle_t)1;
    }
    return pdPASS;
}
void vTaskDelete(TaskHandle_t task) { (void)task; }
void vTaskDelay(TickType_t ticks) { fake_advance_to(s_time_ms + ticks); }
BaseType_t xTaskNotifyGive(TaskHandle_t task) { (void)task; return pdPASS; }
uint32_t ulTaskNotifyTake(BaseType_t clear, TickType_t ticks) { (void)clear; (void)ticks; return 0; }

/* ---- timers: fired only by fake_advance_to() ---- */
#define FAKE_MAX_TIMERS 128
struct fake_timer {
    void (*callback)(void *arg);
    void *arg;
    bool armed;
    uint32_t deadline_ms;
    uint32_t period_ms; // 0 = one-shot
};
// Deleted timers are disarmed but never freed, so a callback that deletes its own timer
// (gdolib's scheduled commands do) can't leave fake_advance_to() holding a dangling pointer.
static struct fake_timer *s_timers[FAKE_MAX_TIMERS];
static int s_timer_count;

int64_t esp_timer_get_time(void) { return (int64_t)s_time_ms * 1000; }

esp_err_t esp_timer_create(const esp_timer_create_args_t *args, esp_timer_handle_t *out) {
    if (s_timer_count == FAKE_MAX_TIMERS) {
        return ESP_ERR_NO_MEM;
    }
    struct fake_timer *t = calloc(1, sizeof(*t));
    t->callback = args->callback;
    t->arg = args->arg;
    s_timers[s_timer_count++] = t;
    *out = t;
    return ESP_OK;
}

// Like ESP-IDF, a NULL handle is rejected rather than dereferenced.
static esp_err_t timer_start(esp_timer_handle_t t, uint64_t us, bool periodic) {
    if (!t) {
        return ESP_ERR_INVALID_ARG;
    }
    if (t->armed) {
        return ESP_ERR_INVALID_STATE;
    }
    t->armed = true;
    t->deadline_ms = s_time_ms + (uint32_t)(us / 1000);
    t->period_ms = periodic ? (uint32_t)(us / 1000) : 0;
    return ESP_OK;
}

esp_err_t esp_timer_start_once(esp_timer_handle_t t, uint64_t us) { return timer_start(t, us, false); }
esp_err_t esp_timer_start_periodic(esp_timer_handle_t t, uint64_t us) { return timer_start(t, us, true); }

esp_err_t esp_timer_stop(esp_timer_handle_t t) {
    if (!t) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!t->armed) {
        return ESP_ERR_INVALID_STATE;
    }
    t->armed = false;
    return ESP_OK;
}

esp_err_t esp_timer_delete(esp_timer_handle_t t) {
    if (!t) {
        return ESP_ERR_INVALID_ARG;
    }
    t->armed = false;
    t->callback = NULL;
    return ESP_OK;
}

bool fake_fire_next_timer(uint32_t until_ms) {
    struct fake_timer *next = NULL;
    for (int i = 0; i < s_timer_count; i++) {
        struct fake_timer *t = s_timers[i];
        if (t->armed && t->deadline_ms <= until_ms && (!next || t->deadline_ms < next->deadline_ms)) {
            next = t;
        }
    }
    if (!next) {
        return false;
    }
    if (next->deadline_ms > s_time_ms) {
        s_time_ms = next->deadline_ms;
    }
    if (next->period_ms) {
        next->deadline_ms += next->period_ms;
    } else {
        next->armed = false;
    }
    next->callback(next->arg);
    return true;
}

void fake_advance_to(uint32_t ms) {
    while (fake_fire_next_timer(ms)) {
    }
    if (ms > s_time_ms) {
        s_time_ms = ms;
    }
}

bool fake_timer_armed(esp_timer_handle_t t) { return t && t->armed; }

/* ---- gpio ---- */
esp_err_t gpio_config(const gpio_config_t *cfg) { (void)cfg; return ESP_OK; }
esp_err_t gpio_reset_pin(gpio_num_t pin) { (void)pin; return ESP_OK; }
int gpio_get_level(gpio_num_t pin) { (void)pin; return 0; }
esp_err_t gpio_install_isr_service(int flags) { (void)flags; return ESP_OK; }
esp_err_t gpio_isr_handler_add(gpio_num_t pin, void (*isr)(void *), void *arg) {
    (void)pin; (void)isr; (void)arg;
    return ESP_OK;
}

/* ---- uart ---- */
esp_err_t uart_param_config(uart_port_t num, const uart_config_t *cfg) { (void)num; (void)cfg; return ESP_OK; }
esp_err_t uart_set_pin(uart_port_t num, int tx, int rx, int rts, int cts) {
    (void)num; (void)tx; (void)rx; (void)rts; (void)cts;
    return ESP_OK;
}
esp_err_t uart_set_line_inverse(uart_port_t num, uint32_t mask) { (void)num; (void)mask; return ESP_OK; }
esp_err_t uart_driver_install(uart_port_t num, int rx_size, int tx_size, int queue_size,
                              QueueHandle_t *queue, int flags) {
    (void)num; (void)rx_size; (void)tx_size; (void)flags;
    if (queue) {
        *queue = xQueueCreate(queue_size, sizeof(uart_event_t));
    }
    return ESP_OK;
}
esp_err_t uart_driver_delete(uart_port_t num) { (void)num; return ESP_OK; }
esp_err_t uart_set_baudrate(uart_port_t num, uint32_t baud) { (void)num; (void)baud; return ESP_OK; }
esp_err_t uart_set_parity(uart_port_t num, uart_parity_t parity) { (void)num; (void)parity; return ESP_OK; }
esp_err_t uart_wait_tx_done(uart_port_t num, TickType_t ticks) { (void)num; (void)ticks; return ESP_OK; }
fake_tx_t fake_tx_log[FAKE_TX_LOG_MAX];
int fake_tx_count;

int uart_write_bytes(uart_port_t num, const void *buf, size_t len) {
    (void)num;
    if (len == 19) {
        fake_uart_frames_written++;
    }
    if (fake_tx_count < FAKE_TX_LOG_MAX && len <= sizeof(fake_tx_log[0].bytes)) {
        fake_tx_t *tx = &fake_tx_log[fake_tx_count++];
        memcpy(tx->bytes, buf, len);
        tx->len = len;
        tx->at_ms = s_time_ms;
    }
    return (int)len;
}

int uart_read_bytes(uart_port_t num, void *buf, uint32_t len, TickType_t ticks) {
    (void)num; (void)ticks;
    size_t n = len < s_rx_len ? len : s_rx_len;
    for (size_t i = 0; i < n; i++) {
        ((uint8_t *)buf)[i] = s_rx[(s_rx_head + i) % FAKE_RX_CAP];
    }
    s_rx_head = (s_rx_head + n) % FAKE_RX_CAP;
    s_rx_len -= n;
    return (int)n;
}

esp_err_t uart_flush_input(uart_port_t num) {
    (void)num;
    s_rx_head = s_rx_len = 0;
    fake_uart_flush_count++;
    return ESP_OK;
}

esp_err_t uart_flush(uart_port_t num) { return uart_flush_input(num); }

/* ---- test controls ---- */
void fake_reset(void) {
    s_rx_head = s_rx_len = 0;
    fake_uart_flush_count = 0;
    fake_uart_frames_written = 0;
    fake_tx_count = 0;
    fake_on_queue_send = NULL;
    fake_on_idle = NULL;
}

void fake_set_time_ms(uint32_t ms) { s_time_ms = ms; }

void fake_uart_rx(const uint8_t *bytes, size_t len) {
    for (size_t i = 0; i < len && s_rx_len < FAKE_RX_CAP; i++) {
        s_rx[(s_rx_head + s_rx_len) % FAKE_RX_CAP] = bytes[i];
        s_rx_len++;
    }
}

size_t fake_uart_rx_available(void) { return s_rx_len; }

uint32_t fake_queue_depth(QueueHandle_t q) { return q->count; }
