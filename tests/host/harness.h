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
 * Shared harness for the host tests. Each test binary includes this once.
 *
 * gdo.c is included directly so its statics are reachable and the real, unmodified
 * gdo_main_task() runs against the fake UART in idf_fake.c. A test starts the driver
 * through the public API, scripts what the opener puts on the wire (and when), then
 * calls run(). The script is fed to the task one step at a time whenever it would
 * otherwise block, advancing the fake clock (and firing due timers) as it goes, so
 * task-local state carries across the whole scenario.
 *
 * Every test runs in its own forked process, so function-static state inside gdo.c
 * (debounce anchors, duration measurement, the v1 poll index) starts fresh each time.
 */

#ifndef GDO_TEST_HARNESS_H
#define GDO_TEST_HARNESS_H

#include "../../gdo.c"

#include <sys/wait.h>
#include <unistd.h>

/* ---------------------------------------------------------------- checks */

static int g_failures;

#define CHECK(cond) do { \
    if (!(cond)) { \
        g_failures++; \
        printf("    FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    } \
} while (0)

#define CHECK_EQ(actual, expected) do { \
    long long a_ = (long long)(actual), e_ = (long long)(expected); \
    if (a_ != e_) { \
        g_failures++; \
        printf("    FAIL %s:%d: %s == %lld, expected %lld\n", __FILE__, __LINE__, #actual, a_, e_); \
    } \
} while (0)

/* ---------------------------------------------------------------- driver under test */

#define OPENER_ID 0x00ABCDEF12ULL
#define FRAME_SIZE 19

typedef struct {
    gdo_protocol_type_t protocol;
    bool synced;
    bool obst_from_status;
} gdo_test_opts_t;

static int s_cb_events[GDO_CB_EVENT_MAX];

static void record_cb(const gdo_status_t *status, gdo_cb_event_t event, void *arg) {
    (void)status; (void)arg;
    s_cb_events[event]++;
}

/* A command the driver queued for transmission, decoded back into its fields. */
typedef struct {
    uint32_t cmd;      // gdo_command_t for v2, gdo_v1_command_t for v1
    uint8_t nibble, byte1, byte2;
    uint32_t rolling;
    uint64_t fixed;
} queued_cmd_t;

#define MAX_QUEUED 64
static queued_cmd_t s_queued[MAX_QUEUED];
static int s_queued_count;

static void on_queue_send(QueueHandle_t q, const void *item) {
    if (q != gdo_tx_queue || s_queued_count == MAX_QUEUED) {
        return;
    }
    const gdo_tx_message_t *m = (const gdo_tx_message_t *)item;
    queued_cmd_t *c = &s_queued[s_queued_count++];
    memset(c, 0, sizeof(*c));
    c->cmd = m->cmd;
    if (g_status.protocol == GDO_PROTOCOL_SEC_PLUS_V2) {
        uint32_t data = 0;
        if (decode_wireline(m->packet, &c->rolling, &c->fixed, &data) == 0) {
            c->nibble = (data >> 8) & 0xf;
            c->byte1 = (data >> 16) & 0xff;
            c->byte2 = (data >> 24) & 0xff;
        }
    }
}

static int queued(uint32_t cmd) {
    int n = 0;
    for (int i = 0; i < s_queued_count; i++) {
        n += s_queued[i].cmd == cmd;
    }
    return n;
}

static const queued_cmd_t *last_queued(uint32_t cmd) {
    for (int i = s_queued_count - 1; i >= 0; i--) {
        if (s_queued[i].cmd == cmd) {
            return &s_queued[i];
        }
    }
    return NULL;
}

/* Bytes actually written to the UART, skipping the 0x00 byte v2 sends as a break. */
static int transmitted(int index, const fake_tx_t **out) {
    int n = 0;
    for (int i = 0; i < fake_tx_count; i++) {
        const fake_tx_t *tx = &fake_tx_log[i];
        if (tx->len == 1 && tx->bytes[0] == 0x00) {
            continue;
        }
        if (n++ == index) {
            *out = tx;
            return 1;
        }
    }
    return 0;
}

static int transmitted_count(void) {
    const fake_tx_t *unused;
    int n = 0;
    while (transmitted(n, &unused)) {
        n++;
    }
    return n;
}

/* RX flushes other than the one transmit_packet() does after every frame it sends. */
static int rx_resyncs(void) {
    return fake_uart_flush_count - fake_uart_frames_written;
}

/* Brings the driver up through the public API, as a host project would. The sync task is
 * created but never runs on the host, so the synced flag is set directly. */
static void start_gdo(gdo_test_opts_t opts) {
    gdo_config_t config = {
        .uart_num = 1,
        .obst_from_status = opts.obst_from_status,
        .uart_tx_pin = 17,
        .uart_rx_pin = 21,
        .obst_in_pin = -1,
    };
    if (opts.protocol) {
        CHECK_EQ(gdo_set_protocol(opts.protocol), ESP_OK);
    }
    CHECK_EQ(gdo_init(&config), ESP_OK);
    CHECK_EQ(gdo_start(record_cb, NULL), ESP_OK);
    g_status.synced = opts.synced;

    fake_on_queue_send = on_queue_send;
    fake_uart_flush_count = 0;
    fake_uart_frames_written = 0;
    fake_tx_count = 0;
    fake_set_time_ms(1000000);
}

static void start_v2(void) {
    start_gdo((gdo_test_opts_t){ .protocol = GDO_PROTOCOL_SEC_PLUS_V2, .synced = true });
}

static void start_v1(void) {
    start_gdo((gdo_test_opts_t){ .protocol = GDO_PROTOCOL_SEC_PLUS_V1, .synced = true });
}

/* ---------------------------------------------------------------- opener script */

#define MAX_STEPS 32

typedef struct {
    uint32_t at_ms;        // ms after the test started when this step happens
    uint8_t bytes[64];     // bytes appended to the UART RX buffer
    size_t len;
    uint8_t breaks;        // UART_BREAK events posted
    uint16_t data_size;    // size of the UART_DATA event posted (0 = none)
    void (*before)(void);  // optional state change applied first
} step_t;

static step_t s_steps[MAX_STEPS];
static int s_step_count, s_next_step;
static uint32_t s_opener_rolling = 0x1000;

static uint32_t t0(void) {
    return 1000000;
}

static void post_uart_event(uart_event_type_t type, size_t size) {
    gdo_event_t ev = {0};
    ev.uart_event.type = type;
    ev.uart_event.size = size;
    xQueueSend(gdo_event_queue, &ev, 0);
}

static bool feed_next_step(void) {
    if (s_next_step >= s_step_count) {
        return false;
    }
    step_t *s = &s_steps[s_next_step];
    // Fire timers due before this step one at a time, so the task handles whatever each
    // one queued (e.g. a paced TX) at its own time, before the step's bytes arrive.
    if (fake_fire_next_timer(t0() + s->at_ms)) {
        return true;
    }
    s_next_step++;
    fake_set_time_ms(t0() + s->at_ms);
    if (s->before) {
        s->before();
    }
    fake_uart_rx(s->bytes, s->len);
    for (int i = 0; i < s->breaks; i++) {
        post_uart_event(UART_BREAK, 0);
    }
    if (s->data_size) {
        post_uart_event(UART_DATA, s->data_size);
    }
    return true;
}

static step_t *add_step(uint32_t at_ms) {
    if (s_step_count == MAX_STEPS) {
        printf("too many script steps\n");
        exit(2);
    }
    step_t *s = &s_steps[s_step_count++];
    memset(s, 0, sizeof(*s));
    s->at_ms = at_ms;
    return s;
}

static void add_bytes(step_t *s, const uint8_t *bytes, size_t len) {
    memcpy(s->bytes + s->len, bytes, len);
    s->len += len;
}

/* Builds the 19-byte wireline frame an opener sends. */
static void v2_frame(uint8_t out[FRAME_SIZE], gdo_command_t cmd, uint8_t nibble, uint8_t byte1, uint8_t byte2) {
    uint32_t data = ((uint32_t)byte2 << 24) | ((uint32_t)byte1 << 16) | ((uint32_t)nibble << 8) | (cmd & 0xff);
    uint64_t fixed = ((uint64_t)(cmd & ~0xff) << 24) | OPENER_ID;
    if (encode_wireline(s_opener_rolling++, fixed, data, out) != 0) {
        printf("encode_wireline failed\n");
        exit(2);
    }
}

/* STATUS frame bits: nibble = door state, byte1 bit 6 = obstruction (active-low),
 * byte2 bit 0 = lock, bit 1 = light, bit 5 = learn. */
static void status_frame(uint8_t out[FRAME_SIZE], gdo_door_state_t door) {
    v2_frame(out, GDO_CMD_STATUS, door, 0x40, 0x00);
}

/* A well-formed Sec+ v2 frame on the wire: break, then 19 bytes. */
static void opener_v2(uint32_t at_ms, gdo_command_t cmd, uint8_t nibble, uint8_t byte1, uint8_t byte2) {
    uint8_t frame[FRAME_SIZE];
    v2_frame(frame, cmd, nibble, byte1, byte2);
    step_t *s = add_step(at_ms);
    add_bytes(s, frame, FRAME_SIZE);
    s->breaks = 1;
    s->data_size = FRAME_SIZE;
}

static void opener_sends(uint32_t at_ms, gdo_door_state_t door) {
    opener_v2(at_ms, GDO_CMD_STATUS, door, 0x40, 0x00);
}

/* A Sec+ v1 exchange as seen on the bus: the command byte and the opener's response. */
static void opener_v1(uint32_t at_ms, uint8_t cmd, uint8_t resp) {
    step_t *s = add_step(at_ms);
    uint8_t pkt[2] = {cmd, resp};
    add_bytes(s, pkt, 2);
    s->data_size = 2;
}

/* Nothing on the wire; just lets the clock (and any timers) run up to at_ms. */
static void wait_until(uint32_t at_ms) {
    add_step(at_ms);
}

static void run(void) {
    fake_on_idle = feed_next_step;
    fake_run_until_idle(gdo_main_task, NULL);
}

static uint32_t tx_queue_depth(void) {
    return fake_queue_depth(gdo_tx_queue);
}

/* ---------------------------------------------------------------- runner */

typedef struct {
    const char *name;
    void (*fn)(void);
} test_case_t;

#define TEST(fn) { #fn, fn }

static int run_tests(const test_case_t *tests, size_t count) {
    fake_log_enabled = getenv("GDO_TEST_LOG") != NULL;
    int failed = 0;
    for (size_t i = 0; i < count; i++) {
        fflush(stdout);
        pid_t pid = fork();
        if (pid == 0) {
            tests[i].fn();
            fflush(stdout);
            _exit(g_failures ? 1 : 0);
        }
        int status = 0;
        waitpid(pid, &status, 0);
        bool ok = WIFEXITED(status) && WEXITSTATUS(status) == 0;
        if (WIFSIGNALED(status)) {
            printf("    crashed with signal %d\n", WTERMSIG(status));
        }
        failed += !ok;
        printf("%s %s\n", ok ? "PASS" : "FAIL", tests[i].name);
    }
    printf("%zu tests, %d failed\n\n", count, failed);
    return failed ? 1 : 0;
}

#define RUN_TESTS(...) \
    int main(void) { \
        static const test_case_t tests_[] = { __VA_ARGS__ }; \
        return run_tests(tests_, sizeof(tests_) / sizeof(tests_[0])); \
    }

#endif // GDO_TEST_HARNESS_H
