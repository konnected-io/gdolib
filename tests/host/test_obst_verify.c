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
 * Security+ 2.0 obstruction-from-status: the verify poll. Once "obstructed", the driver
 * asks for STATUS every OBST_VERIFY_INTERVAL_MS until it reads clear, so a desynced
 * OBST_1 toggle cannot stick while the opener sits idle and sends nothing.
 */

#include "harness.h"

static void start_obst_from_status(void) {
    start_gdo((gdo_test_opts_t){ .protocol = GDO_PROTOCOL_SEC_PLUS_V2, .synced = true,
                                 .obst_from_status = true });
}

/* One OBST_1 edge as the opener sends it: the frame plus its ~74ms retransmit. */
static void opener_obst_edge(uint32_t at_ms) {
    opener_v2(at_ms, GDO_CMD_OBST_1, 0, 0, 0);
    opener_v2(at_ms + 74, GDO_CMD_OBST_1, 0, 0, 0);
}

/* A STATUS reply carrying the obstruction bit (byte1 bit 6, active-low). */
static void opener_status_obst(uint32_t at_ms, bool obstructed) {
    opener_v2(at_ms, GDO_CMD_STATUS, GDO_DOOR_STATE_OPEN, obstructed ? 0x00 : 0x40, 0);
}

static void test_dropped_clear_edge_repaired_by_poll(void) {
    start_obst_from_status();
    opener_obst_edge(0);
    // The clear edge never arrives and the opener goes quiet.
    wait_until(OBST_VERIFY_INTERVAL_MS - 1);
    run();
    CHECK_EQ(g_status.obstruction, GDO_OBSTRUCTION_STATE_OBSTRUCTED);
    CHECK_EQ(queued(GDO_CMD_GET_STATUS), 0);

    // The poll lands past the STATUS lag and the clear-guard, so the reply is accepted.
    opener_status_obst(OBST_VERIFY_INTERVAL_MS + 100, false);
    run();
    CHECK_EQ(queued(GDO_CMD_GET_STATUS), 1);
    const fake_tx_t *tx = NULL;
    CHECK(transmitted(0, &tx) && tx->at_ms == t0() + OBST_VERIFY_INTERVAL_MS);
    CHECK_EQ(g_status.obstruction, GDO_OBSTRUCTION_STATE_CLEAR);
    CHECK_EQ(s_cb_events[GDO_CB_EVENT_OBSTRUCTION], 2);

    // Once clear, the poll stops.
    wait_until(5 * OBST_VERIFY_INTERVAL_MS);
    run();
    CHECK_EQ(queued(GDO_CMD_GET_STATUS), 1);
    CHECK(!fake_timer_armed(obst_verify_timer));
}

static void test_sub_debounce_wave_repaired_by_poll(void) {
    start_obst_from_status();
    // Break and clear closer than OBST_EDGE_DEBOUNCE_MS: the clear edge is swallowed.
    opener_obst_edge(0);
    opener_obst_edge(500);
    run();
    CHECK_EQ(g_status.obstruction, GDO_OBSTRUCTION_STATE_OBSTRUCTED);

    opener_status_obst(OBST_VERIFY_INTERVAL_MS + 100, false);
    run();
    CHECK_EQ(queued(GDO_CMD_GET_STATUS), 1);
    CHECK_EQ(g_status.obstruction, GDO_OBSTRUCTION_STATE_CLEAR);
}

static void test_sustained_obstruction_keeps_polling_until_clear(void) {
    start_obst_from_status();
    opener_obst_edge(0);
    opener_v2(3700, GDO_CMD_PAIR_3_RESP, 0, 0x0e, 0);
    opener_status_obst(6100, true);
    opener_status_obst(12100, true);
    run();
    CHECK_EQ(queued(GDO_CMD_GET_STATUS), 2);
    CHECK_EQ(g_status.obstruction, GDO_OBSTRUCTION_STATE_OBSTRUCTED);
    CHECK(fake_timer_armed(obst_verify_timer));

    // The beam clears; the trailing OBST_1 clear edge must not re-trip.
    opener_v2(15000, GDO_CMD_PAIR_3_RESP, 0, 0x09, 0);
    opener_obst_edge(15471);
    wait_until(40000);
    run();
    CHECK_EQ(g_status.obstruction, GDO_OBSTRUCTION_STATE_CLEAR);
    // The poll due at 18000 sees "clear" and stops without asking.
    CHECK_EQ(queued(GDO_CMD_GET_STATUS), 2);
    CHECK(!fake_timer_armed(obst_verify_timer));
    CHECK_EQ(s_cb_events[GDO_CB_EVENT_OBSTRUCTION], 2);
}

static void test_retrip_restarts_poll(void) {
    start_obst_from_status();
    opener_obst_edge(0);     // trip: poll due at 6000
    opener_obst_edge(2000);  // clear
    opener_obst_edge(4000);  // trip again: poll pushed to 10000
    wait_until(4000 + OBST_VERIFY_INTERVAL_MS - 1);
    run();
    CHECK_EQ(g_status.obstruction, GDO_OBSTRUCTION_STATE_OBSTRUCTED);
    CHECK_EQ(queued(GDO_CMD_GET_STATUS), 0);

    wait_until(4000 + OBST_VERIFY_INTERVAL_MS);
    run();
    CHECK_EQ(queued(GDO_CMD_GET_STATUS), 1);
}

static void test_normal_clear_sends_nothing(void) {
    start_obst_from_status();
    opener_obst_edge(0);
    opener_obst_edge(2000);
    wait_until(5 * OBST_VERIFY_INTERVAL_MS);
    run();
    CHECK_EQ(g_status.obstruction, GDO_OBSTRUCTION_STATE_CLEAR);
    CHECK_EQ(queued(GDO_CMD_GET_STATUS), 0);
    CHECK_EQ(transmitted_count(), 0);
    CHECK(!fake_timer_armed(obst_verify_timer));
}

static void test_stale_status_after_clear_marker_repaired(void) {
    start_obst_from_status();
    opener_v2(0, GDO_CMD_PAIR_3_RESP, 0, 0x0e, 0);
    opener_v2(8000, GDO_CMD_PAIR_3_RESP, 0, 0x09, 0);
    // STATUS still carries the obstructed bit just after 0x09 and re-asserts it.
    opener_status_obst(8500, true);
    run();
    CHECK_EQ(g_status.obstruction, GDO_OBSTRUCTION_STATE_OBSTRUCTED);
    int polls = queued(GDO_CMD_GET_STATUS);

    // The poll restarted at 8500 asks again once the bit has caught up.
    wait_until(8500 + OBST_VERIFY_INTERVAL_MS - 1);
    run();
    CHECK_EQ(queued(GDO_CMD_GET_STATUS), polls);
    opener_status_obst(8500 + OBST_VERIFY_INTERVAL_MS + 100, false);
    run();
    CHECK_EQ(queued(GDO_CMD_GET_STATUS), polls + 1);
    CHECK_EQ(g_status.obstruction, GDO_OBSTRUCTION_STATE_CLEAR);
}

static void test_no_verify_timer_without_option(void) {
    start_v2();
    CHECK(obst_verify_timer == NULL);
    opener_status_obst(0, true); // ignored without obst_from_status
    wait_until(5 * OBST_VERIFY_INTERVAL_MS);
    run();
    CHECK_EQ(queued(GDO_CMD_GET_STATUS), 0);
}

static void test_no_poll_on_sec_plus_v1(void) {
    start_gdo((gdo_test_opts_t){ .protocol = GDO_PROTOCOL_SEC_PLUS_V1, .synced = true,
                                 .obst_from_status = true });
    opener_v1(0, V1_CMD_OBSTRUCTION, 0x01);
    run();
    CHECK_EQ(g_status.obstruction, GDO_OBSTRUCTION_STATE_OBSTRUCTED);
    s_queued_count = 0;

    wait_until(5 * OBST_VERIFY_INTERVAL_MS);
    run();
    CHECK_EQ(s_queued_count, 0);
    CHECK(!fake_timer_armed(obst_verify_timer));
}

static void test_deinit_stops_and_deletes_verify_timer(void) {
    start_obst_from_status();
    opener_obst_edge(0);
    run();
    esp_timer_handle_t timer = obst_verify_timer;
    CHECK(fake_timer_armed(timer));

    CHECK_EQ(gdo_deinit(), ESP_OK);
    CHECK(obst_verify_timer == NULL);
    CHECK(!fake_timer_armed(timer));
}

RUN_TESTS(
    TEST(test_dropped_clear_edge_repaired_by_poll),
    TEST(test_sub_debounce_wave_repaired_by_poll),
    TEST(test_sustained_obstruction_keeps_polling_until_clear),
    TEST(test_retrip_restarts_poll),
    TEST(test_normal_clear_sends_nothing),
    TEST(test_stale_status_after_clear_marker_repaired),
    TEST(test_no_verify_timer_without_option),
    TEST(test_no_poll_on_sec_plus_v1),
    TEST(test_deinit_stops_and_deletes_verify_timer)
)
