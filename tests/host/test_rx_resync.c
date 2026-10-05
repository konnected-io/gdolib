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
 * Sec+ v2 RX framing tests (PR #36): realigning the stream after stray bytes or a short
 * read, and the rate-limited GET_STATUS that covers a frame dropped by the flush.
 */

#include "harness.h"

/* Stray bytes left in the ring buffer with no event of their own (noise fragment, or the
 * tail of a frame whose events were lost). This is the field-reported stuck state. */
static void stray_bytes(uint32_t at_ms, size_t len) {
    static const uint8_t junk[] = {0xde, 0xad, 0xbe, 0xef, 0x13, 0x37, 0x42, 0x99, 0x01, 0x02};
    step_t *s = add_step(at_ms);
    add_bytes(s, junk, len);
}

static void setup(bool synced) {
    start_gdo((gdo_test_opts_t){ .protocol = GDO_PROTOCOL_SEC_PLUS_V2, .synced = synced });
}

static int get_status_queued(void) {
    return queued(GDO_CMD_GET_STATUS);
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
    CHECK_EQ(get_status_queued(), 0);
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
    CHECK_EQ(get_status_queued(), 1);
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

    CHECK_EQ(get_status_queued(), 1);
    CHECK_EQ(rx_resyncs(), 3); // every error still realigns

    // Past the interval another request is allowed.
    stray_bytes(3200, 7);
    opener_sends(3300, GDO_DOOR_STATE_OPEN);
    run();
    CHECK_EQ(get_status_queued(), 2);
}

static void test_unsynced_resync_flushes_without_status(void) {
    setup(false);
    stray_bytes(0, 7);
    opener_sends(1000, GDO_DOOR_STATE_CLOSING);
    opener_sends(1074, GDO_DOOR_STATE_CLOSING);
    run();

    CHECK_EQ(rx_resyncs(), 1);
    CHECK_EQ(get_status_queued(), 0); // the sync task owns status queries until synced
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

    CHECK_EQ(get_status_queued(), 0);
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
    CHECK_EQ(get_status_queued(), 1);
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
    CHECK_EQ(get_status_queued(), 0);

    // The over-count drained to zero, so a command is transmitted, not held as a collision.
    queue_command(GDO_CMD_LIGHT, 1, 0, 0);
    wait_until(500); // past the min TX interval behind the GET_OPENINGS the close queued
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
    CHECK_EQ(get_status_queued(), 0);
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
    CHECK_EQ(get_status_queued(), 0);
}

RUN_TESTS(
    TEST(test_aligned_frames_decode_without_resync),
    TEST(test_stray_bytes_realign_and_request_status),
    TEST(test_retransmit_after_resync_is_decoded),
    TEST(test_status_request_is_rate_limited),
    TEST(test_unsynced_resync_flushes_without_status),
    TEST(test_unsynced_resync_does_not_request_status_once_synced),
    TEST(test_short_read_realigns),
    TEST(test_empty_read_does_not_resync),
    TEST(test_short_packet_event_is_ignored),
    TEST(test_oversized_packet_drops_leading_bytes)
)
