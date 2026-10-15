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

/* Security+ 2.0 heartbeat: detecting an opener that stops responding (issue #47). */

#include "harness.h"

#define HB HEARTBEAT_INTERVAL_MS
#define REPLY HEARTBEAT_REPLY_TIMEOUT_MS

/* The sync task never runs on the host, so post its completion. */
static void sync_completes(void) {
    queue_event((gdo_event_t){GDO_EVENT_SYNC_COMPLETE});
}

static void start_synced_v2(void) {
    start_v2();
    sync_completes();
    opener_sends(0, GDO_DOOR_STATE_CLOSED);
    run();
    CHECK_EQ(s_cb_events[GDO_CB_EVENT_SYNCED], 1);
}

/* Probes at HB, HB+REPLY, HB+2*REPLY; the check at HB+3*REPLY marks it lost. */
static void opener_goes_silent(void) {
    wait_until(HB + HEARTBEAT_MAX_PROBES * REPLY);
    run();
}

static void test_busy_line_sends_no_probes(void) {
    start_synced_v2();
    for (uint32_t t = 4 * 60 * 1000; t <= 30 * 60 * 1000; t += 4 * 60 * 1000) {
        opener_sends(t, GDO_DOOR_STATE_CLOSED);
    }
    run();
    CHECK_EQ(queued(GDO_CMD_GET_STATUS), 0);
    CHECK(g_status.synced);
}

static void test_idle_opener_answers_probe(void) {
    start_synced_v2();
    wait_until(HB - 1);
    run();
    CHECK_EQ(queued(GDO_CMD_GET_STATUS), 0);

    opener_sends(HB + 100, GDO_DOOR_STATE_CLOSED);
    wait_until(HB + 10 * REPLY);
    run();
    CHECK_EQ(queued(GDO_CMD_GET_STATUS), 1);
    const fake_tx_t *tx = NULL;
    // (Earlier transmissions are the GET_OPENINGS a CLOSED status triggers.)
    CHECK(transmitted(transmitted_count() - 1, &tx) && tx->at_ms == t0() + HB);
    CHECK(g_status.synced);
    CHECK_EQ(s_cb_events[GDO_CB_EVENT_SYNCED], 1);

    wait_until(2 * HB + 99);
    run();
    CHECK_EQ(queued(GDO_CMD_GET_STATUS), 1);
    wait_until(2 * HB + 100);
    run();
    CHECK_EQ(queued(GDO_CMD_GET_STATUS), 2);
}

static void test_late_reply_within_probe_limit_stays_synced(void) {
    start_synced_v2();
    opener_sends(HB + 2 * REPLY + 100, GDO_DOOR_STATE_CLOSED);
    wait_until(HB + 10 * REPLY);
    run();
    CHECK_EQ(queued(GDO_CMD_GET_STATUS), 3);
    CHECK(g_status.synced);
    CHECK_EQ(s_cb_events[GDO_CB_EVENT_SYNCED], 1);
}

static void test_silent_opener_marked_unsynced(void) {
    start_synced_v2();
    wait_until(HB + HEARTBEAT_MAX_PROBES * REPLY - 1);
    run();
    CHECK_EQ(queued(GDO_CMD_GET_STATUS), HEARTBEAT_MAX_PROBES);
    CHECK(g_status.synced);

    opener_goes_silent();
    CHECK(!g_status.synced);
    CHECK(g_link_lost);
    CHECK_EQ(s_cb_events[GDO_CB_EVENT_SYNCED], 2);
    CHECK_EQ(queued(GDO_CMD_GET_STATUS), HEARTBEAT_MAX_PROBES + 1);

    wait_until(2 * HB + HEARTBEAT_MAX_PROBES * REPLY - 1);
    run();
    CHECK_EQ(queued(GDO_CMD_GET_STATUS), HEARTBEAT_MAX_PROBES + 1);
    wait_until(4 * HB + HEARTBEAT_MAX_PROBES * REPLY);
    run();
    CHECK_EQ(queued(GDO_CMD_GET_STATUS), HEARTBEAT_MAX_PROBES + 4);
    CHECK_EQ(s_cb_events[GDO_CB_EVENT_SYNCED], 2);
}

static void test_opener_reply_restores_sync(void) {
    start_synced_v2();
    opener_goes_silent();
    CHECK(!g_status.synced);

    uint32_t back = 2 * HB + HEARTBEAT_MAX_PROBES * REPLY + 100;
    opener_sends(back, GDO_DOOR_STATE_OPEN);
    run();
    CHECK(g_status.synced);
    CHECK(!g_link_lost);
    CHECK_EQ(g_status.door, GDO_DOOR_STATE_OPEN);
    CHECK_EQ(s_cb_events[GDO_CB_EVENT_SYNCED], 3);

    int probes = queued(GDO_CMD_GET_STATUS);
    wait_until(back + HB - 1);
    run();
    CHECK_EQ(queued(GDO_CMD_GET_STATUS), probes);
    wait_until(back + HB);
    run();
    CHECK_EQ(queued(GDO_CMD_GET_STATUS), probes + 1);
}

static void test_unprompted_frame_restores_sync(void) {
    start_synced_v2();
    opener_goes_silent();
    CHECK(!g_status.synced);

    opener_v2(HB + 60 * 1000, GDO_CMD_PAIR_3_RESP, 0, 0x09, 0);
    run();
    CHECK(g_status.synced);
    CHECK_EQ(s_cb_events[GDO_CB_EVENT_SYNCED], 3);
}

static void test_own_echo_does_not_count_as_reply(void) {
    start_synced_v2();
    for (int i = 0; i < HEARTBEAT_MAX_PROBES; i++) {
        uint8_t frame[FRAME_SIZE];
        uint32_t data = GDO_CMD_GET_STATUS & 0xff;
        uint64_t fixed = ((uint64_t)(GDO_CMD_GET_STATUS & ~0xff) << 24) | g_status.client_id;
        CHECK_EQ(encode_wireline(i, fixed, data, frame), 0);
        step_t *s = add_step(HB + i * REPLY + 50);
        add_bytes(s, frame, FRAME_SIZE);
        s->breaks = 1;
        s->data_size = FRAME_SIZE;
    }
    opener_goes_silent();
    CHECK(!g_status.synced);
}

static void test_setters_and_sync_rejected_while_lost(void) {
    start_synced_v2();
    opener_goes_silent();
    CHECK(!g_status.synced);
    uint32_t client_id = g_status.client_id;
    uint32_t rolling = g_status.rolling_code;

    CHECK_EQ(gdo_sync(), ESP_ERR_INVALID_STATE);
    CHECK_EQ(gdo_set_client_id(client_id + 1), ESP_ERR_INVALID_STATE);
    CHECK_EQ(gdo_set_rolling_code(0), ESP_ERR_INVALID_STATE);
    CHECK_EQ(g_status.client_id, client_id);
    CHECK_EQ(g_status.rolling_code, rolling);
}

static void test_no_probes_until_synced(void) {
    start_gdo((gdo_test_opts_t){ .protocol = GDO_PROTOCOL_SEC_PLUS_V2, .synced = false });
    wait_until(5 * HB);
    run();
    CHECK_EQ(queued(GDO_CMD_GET_STATUS), 0);
}

static void test_no_heartbeat_on_sec_plus_v1(void) {
    start_v1();
    sync_completes();
    wait_until(5 * HB);
    run();
    CHECK_EQ(queued(GDO_CMD_GET_STATUS), 0);
    CHECK(g_status.synced);
    CHECK_EQ(s_cb_events[GDO_CB_EVENT_SYNCED], 1);
}

static void test_deinit_resets(void) {
    start_synced_v2();
    opener_goes_silent();

    CHECK_EQ(gdo_deinit(), ESP_OK);
    CHECK(!g_link_lost);
    CHECK_EQ(g_last_rx_ms, 0);
}

RUN_TESTS(
    TEST(test_busy_line_sends_no_probes),
    TEST(test_idle_opener_answers_probe),
    TEST(test_late_reply_within_probe_limit_stays_synced),
    TEST(test_silent_opener_marked_unsynced),
    TEST(test_opener_reply_restores_sync),
    TEST(test_unprompted_frame_restores_sync),
    TEST(test_own_echo_does_not_count_as_reply),
    TEST(test_setters_and_sync_rejected_while_lost),
    TEST(test_no_probes_until_synced),
    TEST(test_no_heartbeat_on_sec_plus_v1),
    TEST(test_deinit_resets)
)
