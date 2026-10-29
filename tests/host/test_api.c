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
 * Public API contract (lifecycle, setters, argument validation) and the door position
 * model: duration measurement, position tracking while moving, and move-to-target.
 */

#include "harness.h"

static const gdo_config_t k_config = {
    .uart_num = 1,
    .uart_tx_pin = 17,
    .uart_rx_pin = 21,
    .obst_in_pin = -1,
};

/* ---------------------------------------------------------------- lifecycle */

static void test_init_validates_and_rejects_double_init(void) {
    gdo_config_t bad = k_config;
    CHECK_EQ(gdo_init(NULL), ESP_ERR_INVALID_ARG);
    bad.uart_num = UART_NUM_MAX;
    CHECK_EQ(gdo_init(&bad), ESP_ERR_INVALID_ARG);
    bad = k_config;
    bad.uart_rx_pin = GPIO_NUM_MAX;
    CHECK_EQ(gdo_init(&bad), ESP_ERR_INVALID_ARG);

    CHECK_EQ(gdo_init(&k_config), ESP_OK);
    CHECK_EQ(gdo_init(&k_config), ESP_ERR_INVALID_STATE);
}

static void test_calls_before_init_or_start_are_rejected(void) {
    CHECK_EQ(gdo_start(record_cb, NULL), ESP_ERR_INVALID_STATE);
    CHECK_EQ(gdo_deinit(), ESP_ERR_INVALID_STATE);
    CHECK_EQ(gdo_sync(), ESP_ERR_INVALID_STATE);
    CHECK_EQ(gdo_light_on(), ESP_ERR_INVALID_STATE); // no queue to put it on

    CHECK_EQ(gdo_init(&k_config), ESP_OK);
    CHECK_EQ(gdo_sync(), ESP_ERR_INVALID_STATE); // initialized but not started
    CHECK_EQ(gdo_start(record_cb, NULL), ESP_OK);
    CHECK_EQ(gdo_sync(), ESP_ERR_NOT_FINISHED);  // start already launched the sync task
}

static void test_deinit_resets_state_and_allows_reinit(void) {
    start_v2();
    opener_v2(0, GDO_CMD_STATUS, GDO_DOOR_STATE_OPEN, 0x40, 0x02);
    run();
    CHECK_EQ(g_status.door, GDO_DOOR_STATE_OPEN);

    CHECK_EQ(gdo_deinit(), ESP_OK);
    gdo_status_t status;
    CHECK_EQ(gdo_get_status(&status), ESP_OK);
    CHECK_EQ(status.door, GDO_DOOR_STATE_UNKNOWN);
    CHECK_EQ(status.light, GDO_LIGHT_STATE_MAX);
    CHECK_EQ(status.protocol, 0);
    CHECK_EQ(status.synced, false);
    CHECK_EQ(status.door_position, -1);

    CHECK_EQ(gdo_init(&k_config), ESP_OK);
}

static void test_get_status_snapshots(void) {
    CHECK_EQ(gdo_get_status(NULL), ESP_ERR_INVALID_ARG);
    start_v2();
    opener_sends(0, GDO_DOOR_STATE_CLOSED);
    run();

    gdo_status_t status;
    CHECK_EQ(gdo_get_status(&status), ESP_OK);
    CHECK_EQ(status.door, GDO_DOOR_STATE_CLOSED);
    CHECK_EQ(status.protocol, GDO_PROTOCOL_SEC_PLUS_V2);
}

/* ---------------------------------------------------------------- setters */

static void test_set_protocol(void) {
    CHECK_EQ(gdo_set_protocol(GDO_PROTOCOL_MAX), ESP_ERR_INVALID_ARG);
    CHECK_EQ(gdo_set_protocol(GDO_PROTOCOL_SEC_PLUS_V2), ESP_OK);
    CHECK(g_protocol_forced);
    CHECK_EQ(gdo_set_protocol(GDO_PROTOCOL_SEC_PLUS_V1), ESP_ERR_INVALID_STATE);
}

static void test_reset_protocol_forgets_detection(void) {
    CHECK_EQ(gdo_set_protocol(GDO_PROTOCOL_SEC_PLUS_V2), ESP_OK);
    CHECK(g_protocol_forced);
    CHECK_EQ(gdo_reset_protocol(), ESP_OK);
    CHECK_EQ(g_status.protocol, 0);
    CHECK(!g_protocol_forced);
    CHECK_EQ(gdo_set_protocol(GDO_PROTOCOL_SEC_PLUS_V1), ESP_OK); // detection can start over

    g_status.synced = true;
    CHECK_EQ(gdo_reset_protocol(), ESP_ERR_INVALID_STATE);
    g_status.synced = false;
}

static void test_persisted_rolling_code_and_client_id_are_used(void) {
    // Hosts restore these from storage before start; the opener rejects stale codes.
    CHECK_EQ(gdo_set_client_id(0x1234), ESP_OK);
    CHECK_EQ(gdo_set_rolling_code(0x5000), ESP_OK);
    start_v2();

    CHECK_EQ(gdo_light_toggle(), ESP_OK);
    CHECK_EQ(s_queued[0].rolling, 0x5000);
    CHECK_EQ(s_queued[0].fixed & 0xffffffff, 0x1234);
    CHECK_EQ(s_queued[1].rolling, 0x5001);
    CHECK_EQ(g_status.rolling_code, 0x5002);

    // Once synced they can no longer be changed underneath the session.
    CHECK_EQ(gdo_set_client_id(1), ESP_ERR_INVALID_STATE);
    CHECK_EQ(gdo_set_rolling_code(1), ESP_ERR_INVALID_STATE);
}

static void test_duration_and_interval_validation(void) {
    CHECK_EQ(gdo_set_open_duration(999), ESP_ERR_INVALID_ARG);
    CHECK_EQ(gdo_set_open_duration(65001), ESP_ERR_INVALID_ARG);
    CHECK_EQ(gdo_set_open_duration(12000), ESP_OK);
    CHECK_EQ(gdo_set_close_duration(999), ESP_ERR_INVALID_ARG);
    CHECK_EQ(gdo_set_close_duration(14000), ESP_OK);
    CHECK_EQ(g_status.open_ms, 12000);
    CHECK_EQ(g_status.close_ms, 14000);

    CHECK_EQ(gdo_set_min_command_interval(49), ESP_ERR_INVALID_ARG);
    CHECK_EQ(gdo_set_min_command_interval(100), ESP_OK);
}

/* ---------------------------------------------------------------- door position model */

static void test_open_and_close_durations_are_measured(void) {
    start_v2();
    opener_sends(0, GDO_DOOR_STATE_CLOSED);
    opener_sends(1000, GDO_DOOR_STATE_OPENING);
    opener_sends(13000, GDO_DOOR_STATE_OPEN);
    opener_sends(20000, GDO_DOOR_STATE_CLOSING);
    opener_sends(34000, GDO_DOOR_STATE_CLOSED);
    run();

    CHECK_EQ(g_status.open_ms, 12000);
    CHECK_EQ(g_status.close_ms, 14000);
    CHECK_EQ(s_cb_events[GDO_CB_EVENT_OPEN_DURATION_MEASUREMENT], 1);
    CHECK_EQ(s_cb_events[GDO_CB_EVENT_CLOSE_DURATION_MEASUREMENT], 1);
}

static void test_position_tracks_while_moving(void) {
    start_v2();
    CHECK_EQ(gdo_set_open_duration(10000), ESP_OK);
    CHECK_EQ(gdo_set_close_duration(10000), ESP_OK);
    opener_sends(0, GDO_DOOR_STATE_CLOSED);
    opener_sends(1000, GDO_DOOR_STATE_OPENING);
    wait_until(6000); // halfway
    run();

    CHECK(g_status.door_position >= 4500 && g_status.door_position <= 5500);
    CHECK(s_cb_events[GDO_CB_EVENT_DOOR_POSITION] >= 10); // ~every 500ms while moving

    opener_sends(11000, GDO_DOOR_STATE_OPEN);
    run();
    CHECK_EQ(g_status.door_position, 0);
    CHECK(!fake_timer_armed(door_position_sync_timer));
}

static void test_move_to_target_validation(void) {
    start_v2();
    CHECK_EQ(gdo_door_move_to_target(10001), ESP_ERR_INVALID_ARG);
    // Position and durations unknown.
    CHECK_EQ(gdo_door_move_to_target(5000), ESP_ERR_INVALID_STATE);

    // 0 and 10000 are plain open/close and need neither.
    s_queued_count = 0;
    CHECK_EQ(gdo_door_move_to_target(0), ESP_OK);
    CHECK_EQ(last_queued(GDO_CMD_DOOR_ACTION)->nibble, GDO_DOOR_ACTION_OPEN);
}

static void test_move_to_target_opens_then_stops(void) {
    start_v2();
    CHECK_EQ(gdo_set_open_duration(10000), ESP_OK);
    CHECK_EQ(gdo_set_close_duration(10000), ESP_OK);
    opener_sends(0, GDO_DOOR_STATE_CLOSED);
    run();
    s_queued_count = 0;

    CHECK_EQ(gdo_door_move_to_target(7200), ESP_OK); // 28% of travel: OPEN, STOP at +2800ms
    CHECK_EQ(last_queued(GDO_CMD_DOOR_ACTION)->nibble, GDO_DOOR_ACTION_OPEN);
    CHECK_EQ(g_status.door_target, 7200);

    opener_sends(1100, GDO_DOOR_STATE_OPENING);
    wait_until(2700);
    run();
    CHECK_EQ(last_queued(GDO_CMD_DOOR_ACTION)->nibble, GDO_DOOR_ACTION_OPEN); // not yet

    wait_until(4200); // past the STOP and the 500ms position tick that overshoots to 7000
    run();
    CHECK_EQ(last_queued(GDO_CMD_DOOR_ACTION)->nibble, GDO_DOOR_ACTION_STOP);
    CHECK_EQ(g_status.door_position, 7200); // estimate clamps at the target
    CHECK(!fake_timer_armed(door_position_sync_timer));
}

/* A stop landing between 500ms ticks still counts the travel since the last one. */
static void test_position_counts_travel_since_last_tick_on_stop(void) {
    start_v2();
    CHECK_EQ(gdo_set_open_duration(10000), ESP_OK);
    CHECK_EQ(gdo_set_close_duration(10000), ESP_OK);
    opener_sends(0, GDO_DOOR_STATE_CLOSED);
    opener_sends(1000, GDO_DOOR_STATE_OPENING);
    opener_sends(3250, GDO_DOOR_STATE_STOPPED);
    run();
    CHECK_EQ(g_status.door_position, 7750);
    CHECK_EQ(g_status.door_target, 7750);
}

/* Door open (0), both durations 10s, partial close to 5000: CLOSE now, STOP at +5000ms. */
static void start_partial_close(void) {
    start_v2();
    CHECK_EQ(gdo_set_open_duration(10000), ESP_OK);
    CHECK_EQ(gdo_set_close_duration(10000), ESP_OK);
    opener_sends(0, GDO_DOOR_STATE_OPEN);
    run();
    s_queued_count = 0;

    CHECK_EQ(gdo_door_move_to_target(5000), ESP_OK);
    CHECK_EQ(last_queued(GDO_CMD_DOOR_ACTION)->nibble, GDO_DOOR_ACTION_CLOSE);
    opener_sends(1000, GDO_DOOR_STATE_CLOSING);
}

/* Some openers (a jackshaft on the bench) ignore STOP while closing and only reverse. */
static void test_move_to_target_reverses_when_close_stop_is_ignored(void) {
    start_partial_close();
    wait_until(6400);
    run();
    CHECK_EQ(last_queued(GDO_CMD_DOOR_ACTION)->nibble, GDO_DOOR_ACTION_STOP);
    CHECK_EQ(queued(GDO_CMD_GET_STATUS), 0);

    wait_until(6600); // verify timer at +6500 asks the opener rather than trusting a late report
    run();
    CHECK_EQ(queued(GDO_CMD_GET_STATUS), 1);
    CHECK_EQ(last_queued(GDO_CMD_DOOR_ACTION)->nibble, GDO_DOOR_ACTION_STOP);

    opener_sends(6700, GDO_DOOR_STATE_CLOSING); // still closing: reverse
    run();
    CHECK_EQ(last_queued(GDO_CMD_DOOR_ACTION)->nibble, GDO_DOOR_ACTION_OPEN);
    CHECK_EQ(g_status.door_target, 5000);

    // Closed for 7250ms, so the door is 2250 past the target: 2250ms back up, less the
    // STOP latency, puts the STOP at +1200ms.
    opener_sends(8250, GDO_DOOR_STATE_OPENING);
    run();
    CHECK(g_status.door_position >= 7200 && g_status.door_position <= 7300);
    CHECK_EQ(g_status.door_target, 5000);

    wait_until(9400);
    run();
    CHECK_EQ(last_queued(GDO_CMD_DOOR_ACTION)->nibble, GDO_DOOR_ACTION_OPEN);
    wait_until(9500);
    run();
    CHECK_EQ(last_queued(GDO_CMD_DOOR_ACTION)->nibble, GDO_DOOR_ACTION_STOP);

    opener_sends(10500, GDO_DOOR_STATE_STOPPED);
    run();
    CHECK_EQ(g_status.door_position, 5000);
    CHECK_EQ(queued(GDO_CMD_DOOR_ACTION), 8); // CLOSE, STOP, OPEN, STOP (press + release each)
}

static void test_move_to_target_close_stop_honored(void) {
    start_partial_close();
    opener_sends(6100, GDO_DOOR_STATE_STOPPED);
    wait_until(9000);
    run();
    CHECK_EQ(last_queued(GDO_CMD_DOOR_ACTION)->nibble, GDO_DOOR_ACTION_STOP);
    CHECK_EQ(queued(GDO_CMD_GET_STATUS), 0);
    CHECK_EQ(g_status.door_position, 5000);
}

/* The verify STATUS, not the stale local state, decides: a late STOPPED means no reverse. */
static void test_move_to_target_late_close_stop_report_does_not_reverse(void) {
    start_partial_close();
    wait_until(6600);
    run();
    CHECK_EQ(queued(GDO_CMD_GET_STATUS), 1);

    opener_sends(6700, GDO_DOOR_STATE_STOPPED);
    wait_until(9000);
    run();
    CHECK_EQ(last_queued(GDO_CMD_DOOR_ACTION)->nibble, GDO_DOOR_ACTION_STOP);
    CHECK_EQ(queued(GDO_CMD_DOOR_ACTION), 4); // CLOSE, STOP only
}

/* A STATUS that happens to say CLOSING later, for a close the user asked for, must not
 * be mistaken for the verify reply. */
static void test_move_to_target_full_close_is_never_reversed(void) {
    start_partial_close();
    wait_until(6600);
    run();
    CHECK_EQ(queued(GDO_CMD_GET_STATUS), 1);

    CHECK_EQ(gdo_door_close(), ESP_OK); // already closing: just retargets to fully closed
    opener_sends(6700, GDO_DOOR_STATE_CLOSING);
    run();
    CHECK_EQ(last_queued(GDO_CMD_DOOR_ACTION)->nibble, GDO_DOOR_ACTION_STOP);
    CHECK_EQ(g_status.door_target, 10000);
}

/* Reversing without a STOPPED in between keeps the travel since the last position tick. */
static void test_position_tracks_through_direct_reversal(void) {
    start_v2();
    CHECK_EQ(gdo_set_open_duration(10000), ESP_OK);
    CHECK_EQ(gdo_set_close_duration(10000), ESP_OK);
    opener_sends(0, GDO_DOOR_STATE_OPEN);
    opener_sends(1000, GDO_DOOR_STATE_CLOSING);
    opener_sends(4250, GDO_DOOR_STATE_OPENING); // 3250 closed, 250 of it since the last tick
    run();
    CHECK_EQ(g_status.door_position, 3250);

    wait_until(4800); // ticks restart from the reversal
    run();
    CHECK_EQ(g_status.door_position, 2750);
}

static void test_move_to_target_rejects_tiny_moves(void) {
    start_v2();
    CHECK_EQ(gdo_set_open_duration(10000), ESP_OK);
    CHECK_EQ(gdo_set_close_duration(10000), ESP_OK);
    opener_sends(0, GDO_DOOR_STATE_CLOSED);
    run();
    s_queued_count = 0;

    CHECK_EQ(gdo_door_move_to_target(9900), ESP_OK);          // within 2%: no-op
    CHECK_EQ(gdo_door_move_to_target(9500), ESP_ERR_INVALID_ARG); // 500ms < 800ms minimum
    CHECK_EQ(queued(GDO_CMD_DOOR_ACTION), 0);
    CHECK_EQ(g_status.door_target, 10000);
}

static void test_retry_door_command_repeats_the_pending_request(void) {
    start_v2();
    opener_sends(0, GDO_DOOR_STATE_CLOSED);
    run();
    uint32_t before = g_status.door_command_id;

    CHECK_EQ(gdo_retry_door_command(before + 1), ESP_ERR_INVALID_STATE); // nothing requested yet
    CHECK_EQ(gdo_door_open(), ESP_OK);
    CHECK_EQ(g_status.door_command_id, before + 1);
    s_queued_count = 0;

    CHECK_EQ(gdo_retry_door_command(0), ESP_ERR_INVALID_STATE);
    CHECK_EQ(gdo_retry_door_command(before), ESP_ERR_INVALID_STATE); // stale
    CHECK_EQ(queued(GDO_CMD_DOOR_ACTION), 0);

    CHECK_EQ(gdo_retry_door_command(before + 1), ESP_OK);
    CHECK_EQ(g_status.door_command_id, before + 1); // a retry is not a new request
    CHECK_EQ(queued(GDO_CMD_DOOR_ACTION), 2);
    CHECK_EQ(last_queued(GDO_CMD_DOOR_ACTION)->nibble, GDO_DOOR_ACTION_OPEN);

    // A newer request supersedes the old id.
    opener_sends(0, GDO_DOOR_STATE_OPEN);
    run();
    CHECK_EQ(gdo_door_close(), ESP_OK);
    CHECK_EQ(gdo_retry_door_command(before + 1), ESP_ERR_INVALID_STATE);
    s_queued_count = 0;
    CHECK_EQ(gdo_retry_door_command(before + 2), ESP_OK);
    CHECK_EQ(last_queued(GDO_CMD_DOOR_ACTION)->nibble, GDO_DOOR_ACTION_CLOSE);
}

static void test_retry_door_command_refuses_toggle_only_and_unsynced(void) {
    start_v2();
    opener_sends(0, GDO_DOOR_STATE_CLOSED);
    run();
    CHECK_EQ(gdo_door_open(), ESP_OK);
    uint32_t id = g_status.door_command_id;

    gdo_set_toggle_only(true);
    s_queued_count = 0;
    CHECK_EQ(gdo_retry_door_command(id), ESP_ERR_NOT_SUPPORTED);
    CHECK_EQ(s_queued_count, 0);

    gdo_set_toggle_only(false);
    g_status.synced = false;
    CHECK_EQ(gdo_retry_door_command(id), ESP_ERR_INVALID_STATE);
}

static void test_last_rx_advances_on_any_decoded_packet(void) {
    start_v2();
    CHECK_EQ(gdo_get_last_rx_ms(), 0);
    opener_sends(0, GDO_DOOR_STATE_CLOSED);
    run();
    int64_t first = gdo_get_last_rx_ms();
    CHECK(first > 0);

    // A repeat of the same state changes nothing visible, but the link is alive.
    opener_sends(0, GDO_DOOR_STATE_CLOSED);
    run();
    CHECK(gdo_get_last_rx_ms() >= first);

    CHECK_EQ(gdo_deinit(), ESP_OK);
    CHECK_EQ(gdo_get_last_rx_ms(), 0);
}

static void test_refresh_status_requests_status_on_v2_only(void) {
    start_v2();
    opener_sends(0, GDO_DOOR_STATE_CLOSED);
    run();
    s_queued_count = 0;
    CHECK_EQ(gdo_refresh_status(), ESP_OK);
    CHECK_EQ(queued(GDO_CMD_GET_STATUS), 1);

    g_status.protocol = GDO_PROTOCOL_SEC_PLUS_V1;
    s_queued_count = 0;
    CHECK_EQ(gdo_refresh_status(), ESP_ERR_NOT_SUPPORTED);
    CHECK_EQ(s_queued_count, 0);
}

RUN_TESTS(
    TEST(test_init_validates_and_rejects_double_init),
    TEST(test_calls_before_init_or_start_are_rejected),
    TEST(test_deinit_resets_state_and_allows_reinit),
    TEST(test_get_status_snapshots),
    TEST(test_set_protocol),
    TEST(test_reset_protocol_forgets_detection),
    TEST(test_persisted_rolling_code_and_client_id_are_used),
    TEST(test_duration_and_interval_validation),
    TEST(test_open_and_close_durations_are_measured),
    TEST(test_position_tracks_while_moving),
    TEST(test_move_to_target_validation),
    TEST(test_move_to_target_opens_then_stops),
    TEST(test_position_counts_travel_since_last_tick_on_stop),
    TEST(test_move_to_target_reverses_when_close_stop_is_ignored),
    TEST(test_move_to_target_close_stop_honored),
    TEST(test_move_to_target_late_close_stop_report_does_not_reverse),
    TEST(test_move_to_target_full_close_is_never_reversed),
    TEST(test_position_tracks_through_direct_reversal),
    TEST(test_move_to_target_rejects_tiny_moves),
    TEST(test_retry_door_command_repeats_the_pending_request),
    TEST(test_retry_door_command_refuses_toggle_only_and_unsynced),
    TEST(test_last_rx_advances_on_any_decoded_packet),
    TEST(test_refresh_status_requests_status_on_v2_only)
)
