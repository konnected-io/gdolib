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
 * Sec+ v2 RX framing tests (PR #36). gdo.c is included directly so the real, unmodified
 * gdo_main_task() runs against the fake UART in idf_fake.c. Each test scripts what the
 * opener puts on the wire; the script is fed to the task one step at a time whenever it
 * would otherwise block, so task-local state (rx_pending) carries across the whole scenario.
 */

#include "../../gdo.c"

static int g_failures;
static int g_checks;

#define CHECK(cond) do { \
    g_checks++; \
    if (!(cond)) { \
        g_failures++; \
        printf("    FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    } \
} while (0)

#define CHECK_EQ(actual, expected) do { \
    long long a_ = (long long)(actual), e_ = (long long)(expected); \
    g_checks++; \
    if (a_ != e_) { \
        g_failures++; \
        printf("    FAIL %s:%d: %s == %lld, expected %lld\n", __FILE__, __LINE__, #actual, a_, e_); \
    } \
} while (0)

/* ---------------------------------------------------------------- opener script */

#define OPENER_ID 0x00ABCDEF12ULL
#define FRAME_SIZE 19
#define MAX_STEPS 32

typedef struct {
    uint32_t at_ms;        // clock value when this step happens
    uint8_t bytes[64];     // bytes appended to the UART RX buffer
    size_t len;
    uint8_t breaks;        // UART_BREAK events posted
    uint16_t data_size;    // size of the UART_DATA event posted (0 = none)
    void (*before)(void);  // optional state change applied first
} step_t;

static step_t s_steps[MAX_STEPS];
static int s_step_count, s_next_step;
static uint32_t s_base_ms;
static uint32_t s_opener_rolling = 0x1000;

static int s_get_status_queued;

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
    step_t *s = &s_steps[s_next_step++];
    fake_set_time_ms(s_base_ms + s->at_ms);
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

static void on_queue_send(QueueHandle_t q, const void *item) {
    if (q == gdo_tx_queue && ((const gdo_tx_message_t *)item)->cmd == GDO_CMD_GET_STATUS) {
        s_get_status_queued++;
    }
}

/* Builds the 19-byte wireline STATUS frame an opener sends for a door state. */
static void status_frame(uint8_t out[FRAME_SIZE], gdo_door_state_t door) {
    uint8_t byte1 = 0x40; // obstruction bit is active-low: set = clear
    uint32_t data = (byte1 << 16) | ((uint32_t)door << 8) | (GDO_CMD_STATUS & 0xff);
    uint64_t fixed = ((uint64_t)(GDO_CMD_STATUS & ~0xff) << 24) | OPENER_ID;
    if (encode_wireline(s_opener_rolling++, fixed, data, out) != 0) {
        printf("encode_wireline failed\n");
        exit(2);
    }
}

static step_t *add_step(uint32_t at_ms) {
    step_t *s = &s_steps[s_step_count++];
    memset(s, 0, sizeof(*s));
    s->at_ms = at_ms;
    return s;
}

static void add_bytes(step_t *s, const uint8_t *bytes, size_t len) {
    memcpy(s->bytes + s->len, bytes, len);
    s->len += len;
}

/* A well-formed frame on the wire: break, then 19 bytes. */
static void opener_sends(uint32_t at_ms, gdo_door_state_t door) {
    uint8_t frame[FRAME_SIZE];
    status_frame(frame, door);
    step_t *s = add_step(at_ms);
    add_bytes(s, frame, FRAME_SIZE);
    s->breaks = 1;
    s->data_size = FRAME_SIZE;
}

/* Stray bytes left in the ring buffer with no event of their own (noise fragment, or the
 * tail of a frame whose events were lost). This is the field-reported stuck state. */
static void stray_bytes(uint32_t at_ms, size_t len) {
    static const uint8_t junk[] = {0xde, 0xad, 0xbe, 0xef, 0x13, 0x37, 0x42, 0x99, 0x01, 0x02};
    step_t *s = add_step(at_ms);
    add_bytes(s, junk, len);
}

/* ---------------------------------------------------------------- harness */

static int s_cb_door_events;

static void record_cb(const gdo_status_t *status, gdo_cb_event_t event, void *arg) {
    (void)status; (void)arg;
    if (event == GDO_CB_EVENT_DOOR_POSITION) {
        s_cb_door_events++;
    }
}

static void setup(bool synced) {
    fake_reset();
    fake_on_idle = feed_next_step;
    fake_on_queue_send = on_queue_send;
    if (gdo_event_queue) {
        vQueueDelete(gdo_event_queue);
    }
    if (gdo_tx_queue) {
        gdo_tx_message_t m;
        while (xQueueReceive(gdo_tx_queue, &m, 0) == pdTRUE) {
            free(m.packet);
        }
        vQueueDelete(gdo_tx_queue);
    }
    gdo_event_queue = xQueueCreate(64, sizeof(gdo_event_t));
    gdo_tx_queue = xQueueCreate(16, sizeof(gdo_tx_message_t));
    g_event_callback = record_cb;
    g_status.protocol = GDO_PROTOCOL_SEC_PLUS_V2;
    g_status.synced = synced;
    g_status.door = GDO_DOOR_STATE_UNKNOWN;

    s_step_count = s_next_step = 0;
    s_get_status_queued = 0;
    s_cb_door_events = 0;
    // Start every test an hour after the last so no rate limiter carries over.
    s_base_ms += 3600 * 1000;
    fake_set_time_ms(s_base_ms);
}

static void run(void) {
    fake_run_until_idle(gdo_main_task, NULL);
}

/* RX flushes other than the one transmit_packet() does after every frame it sends. */
static int rx_resyncs(void) {
    return fake_uart_flush_count - fake_uart_frames_written;
}

static uint32_t tx_queue_depth(void) {
    return fake_queue_depth(gdo_tx_queue);
}

/* ---------------------------------------------------------------- tests */

static void test_aligned_frames_decode_without_resync(void) {
    setup(true);
    opener_sends(0, GDO_DOOR_STATE_OPENING);
    opener_sends(74, GDO_DOOR_STATE_OPENING);
    opener_sends(5000, GDO_DOOR_STATE_OPEN);
    run();

    CHECK_EQ(g_status.door, GDO_DOOR_STATE_OPEN);
    CHECK_EQ(rx_resyncs(), 0);
    CHECK_EQ(s_get_status_queued, 0);
}

static void test_stray_bytes_realign_and_request_status(void) {
    setup(true);
    // 7 stray bytes: every following 19-byte read straddles two frames. On master this
    // never recovers until the device transmits.
    stray_bytes(0, 7);
    opener_sends(1000, GDO_DOOR_STATE_CLOSING);
    opener_sends(1074, GDO_DOOR_STATE_CLOSING);  // opener's retransmit
    opener_sends(15000, GDO_DOOR_STATE_CLOSED);
    run();

    CHECK_EQ(rx_resyncs(), 1);
    CHECK_EQ(s_get_status_queued, 1);
    // rx_pending was cleared by the resync, so the GET_STATUS was not held back as a collision.
    CHECK_EQ(tx_queue_depth(), 0);
    CHECK_EQ(g_status.door, GDO_DOOR_STATE_CLOSED);
    CHECK_EQ(fake_uart_rx_available(), 0);
}

static void test_retransmit_after_resync_is_decoded(void) {
    setup(true);
    stray_bytes(0, 7);
    opener_sends(1000, GDO_DOOR_STATE_CLOSING); // lost to the misalignment
    opener_sends(1074, GDO_DOOR_STATE_CLOSING); // the retransmit carries it
    run();

    CHECK_EQ(g_status.door, GDO_DOOR_STATE_CLOSING);
}

static void test_status_request_is_rate_limited(void) {
    setup(true);
    stray_bytes(0, 7);
    opener_sends(100, GDO_DOOR_STATE_OPENING);
    stray_bytes(1000, 5);
    opener_sends(1100, GDO_DOOR_STATE_OPENING); // within 3s of the first request
    stray_bytes(2900, 3);
    opener_sends(2950, GDO_DOOR_STATE_OPENING); // still within 3s
    run();

    CHECK_EQ(s_get_status_queued, 1);
    CHECK_EQ(rx_resyncs(), 3); // every error still realigns

    // Past the interval another request is allowed.
    stray_bytes(3200, 7);
    opener_sends(3300, GDO_DOOR_STATE_OPEN);
    run();
    CHECK_EQ(s_get_status_queued, 2);
}

static void test_unsynced_resync_flushes_without_status(void) {
    setup(false);
    stray_bytes(0, 7);
    opener_sends(1000, GDO_DOOR_STATE_CLOSING);
    opener_sends(1074, GDO_DOOR_STATE_CLOSING);
    run();

    CHECK_EQ(rx_resyncs(), 1);
    CHECK_EQ(s_get_status_queued, 0); // the sync task owns status queries until synced
    CHECK_EQ(g_status.door, GDO_DOOR_STATE_CLOSING);
}

static void become_synced(void) {
    g_status.synced = true;
}

static void test_unsynced_resync_does_not_request_status_once_synced(void) {
    setup(false);
    stray_bytes(0, 7);
    opener_sends(1000, GDO_DOOR_STATE_CLOSING);
    // Sync completes later; the error seen before it must not trigger a late GET_STATUS.
    add_step(4000)->before = become_synced;
    opener_sends(5000, GDO_DOOR_STATE_CLOSED);
    run();

    CHECK_EQ(s_get_status_queued, 0);
    CHECK_EQ(g_status.door, GDO_DOOR_STATE_CLOSED);
}

static void test_short_read_realigns(void) {
    setup(true);
    // Two breaks are counted but only one whole frame plus a fragment is buffered, so the
    // second read comes up short and would leave the stream mid-frame.
    uint8_t frame[FRAME_SIZE], next[FRAME_SIZE];
    status_frame(frame, GDO_DOOR_STATE_OPENING);
    status_frame(next, GDO_DOOR_STATE_OPENING);
    step_t *s = add_step(0);
    add_bytes(s, frame, FRAME_SIZE);
    add_bytes(s, next, 7);
    s->breaks = 2;
    s->data_size = FRAME_SIZE;
    opener_sends(3000, GDO_DOOR_STATE_OPEN);
    run();

    CHECK_EQ(rx_resyncs(), 1);
    CHECK_EQ(s_get_status_queued, 1);
    CHECK_EQ(tx_queue_depth(), 0);
    CHECK_EQ(g_status.door, GDO_DOOR_STATE_OPEN);
}

static void test_empty_read_does_not_resync(void) {
    setup(true);
    // Noise breaks over-count rx_pending; the buffer holds exactly one aligned frame.
    uint8_t frame[FRAME_SIZE];
    status_frame(frame, GDO_DOOR_STATE_CLOSED);
    step_t *s = add_step(0);
    add_bytes(s, frame, FRAME_SIZE);
    s->breaks = 3;
    s->data_size = FRAME_SIZE;
    run();

    CHECK_EQ(g_status.door, GDO_DOOR_STATE_CLOSED);
    CHECK_EQ(rx_resyncs(), 0);
    CHECK_EQ(s_get_status_queued, 0);

    // The over-count drained to zero, so a command is transmitted, not held as a collision.
    queue_command(GDO_CMD_LIGHT, 1, 0, 0);
    run();
    CHECK_EQ(tx_queue_depth(), 0);
}

static void test_short_packet_event_is_ignored(void) {
    setup(true);
    // A UART_DATA event smaller than a frame is a fragment; it is consumed and dropped,
    // and the next good frame decodes.
    step_t *s = add_step(0);
    static const uint8_t frag[] = {0x00, 0x00, 0x00};
    add_bytes(s, frag, sizeof(frag));
    s->breaks = 1;
    s->data_size = sizeof(frag);
    opener_sends(500, GDO_DOOR_STATE_STOPPED);
    run();

    CHECK_EQ(g_status.door, GDO_DOOR_STATE_STOPPED);
    CHECK_EQ(s_get_status_queued, 0);
}

static void test_oversized_packet_drops_leading_bytes(void) {
    setup(true);
    // A break read as a leading 0x00 makes a 20-byte event; the extra byte is discarded.
    uint8_t frame[FRAME_SIZE];
    status_frame(frame, GDO_DOOR_STATE_OPEN);
    step_t *s = add_step(0);
    static const uint8_t zero = 0x00;
    add_bytes(s, &zero, 1);
    add_bytes(s, frame, FRAME_SIZE);
    s->breaks = 1;
    s->data_size = FRAME_SIZE + 1;
    run();

    CHECK_EQ(g_status.door, GDO_DOOR_STATE_OPEN);
    CHECK_EQ(rx_resyncs(), 0);
    CHECK_EQ(s_get_status_queued, 0);
}

/* ---------------------------------------------------------------- main */

typedef struct {
    const char *name;
    void (*fn)(void);
} test_case_t;

#define TEST(fn) { #fn, fn }

int main(void) {
    fake_log_enabled = getenv("GDO_TEST_LOG") != NULL;

    static const test_case_t tests[] = {
        TEST(test_aligned_frames_decode_without_resync),
        TEST(test_stray_bytes_realign_and_request_status),
        TEST(test_retransmit_after_resync_is_decoded),
        TEST(test_status_request_is_rate_limited),
        TEST(test_unsynced_resync_flushes_without_status),
        TEST(test_unsynced_resync_does_not_request_status_once_synced),
        TEST(test_short_read_realigns),
        TEST(test_empty_read_does_not_resync),
        TEST(test_short_packet_event_is_ignored),
        TEST(test_oversized_packet_drops_leading_bytes),
    };

    int failed_tests = 0;
    for (size_t i = 0; i < sizeof(tests) / sizeof(tests[0]); i++) {
        int before = g_failures;
        tests[i].fn();
        bool ok = g_failures == before;
        failed_tests += !ok;
        printf("%s %s\n", ok ? "PASS" : "FAIL", tests[i].name);
    }

    printf("\n%zu tests, %d failed (%d checks)\n", sizeof(tests) / sizeof(tests[0]), failed_tests, g_checks);
    return failed_tests ? 1 : 0;
}
