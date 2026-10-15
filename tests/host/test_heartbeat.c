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

/* Security+ 2.0 heartbeat: detecting an opener that stops responding (issue #47).
 * The idle wait lives in gdo_main_task's locals, which restart on every run(), so each
 * test scripts the whole scenario and runs it once, sampling state mid-run with snapshot(). */

#include "harness.h"

#define HB HEARTBEAT_INTERVAL_MS
#define REPLY HEARTBEAT_REPLY_TIMEOUT_MS

typedef struct {
    int probes;
    bool synced;
} snap_t;

static snap_t s_snaps[8];
static int s_snap_count;

static int probes(void) {
    return queued(GDO_CMD_GET_STATUS);
}

static void take_snapshot(void) {
    s_snaps[s_snap_count++] = (snap_t){ probes(), g_status.synced };
}

static void snapshot(uint32_t at_ms) {
    add_step(at_ms)->before = take_snapshot;
}

/* Synced on Sec+ v2 with the opener's last frame at t=0 (OPEN, so nothing is sent back). */
static void start_synced_v2(void) {
    start_v2();
    // The real sync task clears its handle on exit; on the host it never runs.
    gdo_sync_task_handle = NULL;
    opener_sends(0, GDO_DOOR_STATE_OPEN);
}

static void test_busy_line_sends_no_probes(void) {
    start_synced_v2();
    for (uint32_t t = 4 * 60 * 1000; t <= 30 * 60 * 1000; t += 4 * 60 * 1000) {
        opener_sends(t, GDO_DOOR_STATE_OPEN);
    }
    run();
    CHECK_EQ(probes(), 0);
    CHECK(g_status.synced);
}

static void test_idle_opener_answers_probe(void) {
    start_synced_v2();
    snapshot(HB - 1);
    opener_sends(HB + 500, GDO_DOOR_STATE_OPEN);
    snapshot(HB + 10 * REPLY);
    snapshot(2 * HB + 499);
    wait_until(2 * HB + 501);
    run();

    CHECK_EQ(s_snaps[0].probes, 0);
    CHECK_EQ(s_snaps[1].probes, 1);
    CHECK(s_snaps[1].synced);
    CHECK_EQ(s_snaps[2].probes, 1);
    CHECK_EQ(probes(), 2);
    CHECK(g_status.synced);
}

static void test_silent_opener_marked_unsynced(void) {
    start_synced_v2();
    snapshot(HB + REPLY - 1);
    wait_until(HB + REPLY + 100);
    run();

    CHECK_EQ(s_snaps[0].probes, 1);
    CHECK(s_snaps[0].synced);
    CHECK(!g_status.synced);
    CHECK(gdo_sync_task_handle != NULL); // resync started; its SYNC_COMPLETE notifies the app
}

static void test_silent_opener_eventually_unsynced(void) {
    start_synced_v2();
    wait_until(6 * HB);
    run();
    CHECK(!g_status.synced);
    CHECK_EQ(probes(), 1); // one unanswered probe is enough; no further rolling codes spent
}

static void test_lost_state_reads_as_unknown(void) {
    start_synced_v2();
    wait_until(HB + 2 * REPLY);
    run();
    CHECK(!g_status.synced);
    // The sync task requests status only while door is UNKNOWN, and paired devices only while
    // their counts are GDO_PAIRED_DEVICE_COUNT_UNKNOWN.
    CHECK_EQ(g_status.door, GDO_DOOR_STATE_UNKNOWN);
    CHECK_EQ(g_status.paired_devices.total_all, GDO_PAIRED_DEVICE_COUNT_UNKNOWN);
}

static void test_own_echo_does_not_count_as_reply(void) {
    start_synced_v2();
    // The probe's own bytes come back on the shared line.
    uint8_t frame[FRAME_SIZE];
    uint32_t data = GDO_CMD_GET_STATUS & 0xff;
    uint64_t fixed = ((uint64_t)(GDO_CMD_GET_STATUS & ~0xff) << 24) | g_status.client_id;
    CHECK_EQ(encode_wireline(g_status.rolling_code, fixed, data, frame), 0);
    step_t *s = add_step(HB + 50);
    add_bytes(s, frame, FRAME_SIZE);
    s->breaks = 1;
    s->data_size = FRAME_SIZE;
    wait_until(HB + 2 * REPLY);
    run();
    CHECK_EQ(probes(), 1);
    CHECK(!g_status.synced);
}

static void test_no_probes_until_synced(void) {
    start_gdo((gdo_test_opts_t){ .protocol = GDO_PROTOCOL_SEC_PLUS_V2, .synced = false });
    opener_sends(0, GDO_DOOR_STATE_OPEN);
    wait_until(5 * HB);
    run();
    CHECK_EQ(probes(), 0);
}

static void test_no_heartbeat_on_sec_plus_v1(void) {
    start_v1();
    opener_v1(0, 0x38, 0x02);
    wait_until(5 * HB);
    run();
    CHECK_EQ(probes(), 0);
    CHECK(g_status.synced);
}

RUN_TESTS(
    TEST(test_busy_line_sends_no_probes),
    TEST(test_idle_opener_answers_probe),
    TEST(test_silent_opener_marked_unsynced),
    TEST(test_silent_opener_eventually_unsynced),
    TEST(test_lost_state_reads_as_unknown),
    TEST(test_own_echo_does_not_count_as_reply),
    TEST(test_no_probes_until_synced),
    TEST(test_no_heartbeat_on_sec_plus_v1)
)
