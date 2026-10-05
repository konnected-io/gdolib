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
 * Security+ 2.0: decoding what the opener reports, and the commands the public API sends.
 */

#include "harness.h"

/* ---------------------------------------------------------------- decode */

static void test_status_frame_updates_state_and_fires_callbacks(void) {
    start_v2();
    // door OPEN, light on (byte2 bit 1), locked (byte2 bit 0)
    opener_v2(0, GDO_CMD_STATUS, GDO_DOOR_STATE_OPEN, 0x40, 0x03);
    run();

    CHECK_EQ(g_status.door, GDO_DOOR_STATE_OPEN);
    CHECK_EQ(g_status.light, GDO_LIGHT_STATE_ON);
    CHECK_EQ(g_status.lock, GDO_LOCK_STATE_LOCKED);
    CHECK_EQ(g_status.learn, GDO_LEARN_STATE_INACTIVE);
    CHECK_EQ(g_status.door_position, 0);
    CHECK_EQ(s_cb_events[GDO_CB_EVENT_DOOR_POSITION], 1);
    CHECK_EQ(s_cb_events[GDO_CB_EVENT_LIGHT], 1);
    CHECK_EQ(s_cb_events[GDO_CB_EVENT_LOCK], 1);

    // The opener's retransmit and periodic repeats change nothing, so fire no callbacks.
    opener_v2(74, GDO_CMD_STATUS, GDO_DOOR_STATE_OPEN, 0x40, 0x03);
    run();
    CHECK_EQ(s_cb_events[GDO_CB_EVENT_LIGHT], 1);
    CHECK_EQ(s_cb_events[GDO_CB_EVENT_LOCK], 1);
}

static void test_closed_sets_position_and_requests_openings(void) {
    start_v2();
    opener_sends(0, GDO_DOOR_STATE_CLOSED);
    run();

    CHECK_EQ(g_status.door, GDO_DOOR_STATE_CLOSED);
    CHECK_EQ(g_status.door_position, 10000);
    CHECK_EQ(queued(GDO_CMD_GET_OPENINGS), 1);
}

static void test_frames_with_our_client_id_are_ignored(void) {
    start_v2();
    // Our own transmissions echo back on the shared wire; they must not be decoded.
    uint8_t frame[FRAME_SIZE];
    uint32_t data = ((uint32_t)GDO_DOOR_STATE_CLOSING << 8) | (GDO_CMD_STATUS & 0xff);
    CHECK_EQ(encode_wireline(1, g_status.client_id, data, frame), 0);
    step_t *s = add_step(0);
    add_bytes(s, frame, FRAME_SIZE);
    s->breaks = 1;
    s->data_size = FRAME_SIZE;
    run();

    CHECK_EQ(g_status.door, GDO_DOOR_STATE_UNKNOWN);
}

static void test_light_action_from_opener(void) {
    start_v2();
    opener_v2(0, GDO_CMD_LIGHT, GDO_LIGHT_ACTION_ON, 0, 0);
    run();
    CHECK_EQ(g_status.light, GDO_LIGHT_STATE_ON);

    opener_v2(500, GDO_CMD_LIGHT, GDO_LIGHT_ACTION_TOGGLE, 0, 0);
    run();
    CHECK_EQ(g_status.light, GDO_LIGHT_STATE_OFF);
}

static void test_wall_button_press_and_release(void) {
    start_v2();
    opener_v2(0, GDO_CMD_DOOR_ACTION, GDO_DOOR_ACTION_TOGGLE, 1, 1);
    run();
    CHECK_EQ(g_status.button, GDO_BUTTON_STATE_PRESSED);
    CHECK_EQ(queued(GDO_CMD_GET_STATUS), 0);

    // Release asks for status so the resulting door motion is picked up promptly.
    opener_v2(300, GDO_CMD_DOOR_ACTION, GDO_DOOR_ACTION_TOGGLE, 0, 1);
    run();
    CHECK_EQ(g_status.button, GDO_BUTTON_STATE_RELEASED);
    CHECK_EQ(queued(GDO_CMD_GET_STATUS), 1);
    CHECK_EQ(s_cb_events[GDO_CB_EVENT_BUTTON], 2);
}

static void test_motion_detected_then_clears(void) {
    start_v2();
    opener_v2(0, GDO_CMD_MOTION, 0, 0, 0);
    run();
    CHECK_EQ(g_status.motion, GDO_MOTION_STATE_DETECTED);
    CHECK_EQ(queued(GDO_CMD_GET_STATUS), 1);

    // No further motion: the 3s timer clears it.
    wait_until(3500);
    run();
    CHECK_EQ(g_status.motion, GDO_MOTION_STATE_CLEAR);
    CHECK_EQ(s_cb_events[GDO_CB_EVENT_MOTION], 2);
}

static void test_motion_ignored_before_sync(void) {
    start_gdo((gdo_test_opts_t){ .protocol = GDO_PROTOCOL_SEC_PLUS_V2, .synced = false });
    opener_v2(0, GDO_CMD_MOTION, 0, 0, 0);
    run();
    CHECK_EQ(g_status.motion, GDO_MOTION_STATE_MAX);
    CHECK_EQ(s_cb_events[GDO_CB_EVENT_MOTION], 0);
}

static void test_openings_paired_devices_battery_ttc(void) {
    start_v2();
    opener_v2(0, GDO_CMD_OPENINGS, 0, 0x01, 0x2c); // 300 openings, our request (flag 0)
    opener_v2(100, GDO_CMD_PAIRED_DEVICES, GDO_PAIRED_DEVICE_TYPE_REMOTE, 0, 3);
    opener_v2(200, GDO_CMD_PAIRED_DEVICES, GDO_PAIRED_DEVICE_TYPE_ALL, 0, 5);
    opener_v2(300, GDO_CMD_BATTERY_STATUS, 0, GDO_BATT_STATE_CHARGING, 0);
    opener_v2(400, GDO_CMD_SET_TTC, 0, 0x01, 0x00); // 256 s
    run();

    CHECK_EQ(g_status.openings, 300);
    CHECK_EQ(g_status.paired_devices.total_remotes, 3);
    CHECK_EQ(g_status.paired_devices.total_all, 5);
    CHECK_EQ(g_status.battery, GDO_BATT_STATE_CHARGING);
    CHECK_EQ(g_status.ttc_seconds, 256);
    CHECK_EQ(s_cb_events[GDO_CB_EVENT_OPENINGS], 1);
    CHECK_EQ(s_cb_events[GDO_CB_EVENT_PAIRED_DEVICES], 2);
}

static void test_unsolicited_openings_ignored_until_known(void) {
    start_v2();
    // A nonzero flag means another device asked; with no baseline yet, don't trust it.
    opener_v2(0, GDO_CMD_OPENINGS, 1, 0x00, 0x10);
    run();
    CHECK_EQ(g_status.openings, 0);

    opener_v2(100, GDO_CMD_OPENINGS, 0, 0x00, 0x10);
    opener_v2(200, GDO_CMD_OPENINGS, 1, 0x00, 0x11);
    run();
    CHECK_EQ(g_status.openings, 0x11);
}

static void test_obstruction_from_status_bit(void) {
    start_gdo((gdo_test_opts_t){ .protocol = GDO_PROTOCOL_SEC_PLUS_V2, .synced = true,
                                 .obst_from_status = true });
    opener_v2(0, GDO_CMD_STATUS, GDO_DOOR_STATE_OPEN, 0x00, 0); // bit 6 low = obstructed
    run();
    CHECK_EQ(g_status.obstruction, GDO_OBSTRUCTION_STATE_OBSTRUCTED);

    opener_v2(10000, GDO_CMD_STATUS, GDO_DOOR_STATE_OPEN, 0x40, 0);
    run();
    CHECK_EQ(g_status.obstruction, GDO_OBSTRUCTION_STATE_CLEAR);
    CHECK_EQ(s_cb_events[GDO_CB_EVENT_OBSTRUCTION], 2);
}

static void test_obstruction_status_bit_ignored_without_option(void) {
    start_v2();
    opener_v2(0, GDO_CMD_STATUS, GDO_DOOR_STATE_OPEN, 0x00, 0);
    run();
    CHECK_EQ(g_status.obstruction, GDO_OBSTRUCTION_STATE_MAX);
}

static void test_long_obstruction_markers(void) {
    start_gdo((gdo_test_opts_t){ .protocol = GDO_PROTOCOL_SEC_PLUS_V2, .synced = true,
                                 .obst_from_status = true });
    opener_v2(0, GDO_CMD_PAIR_3_RESP, 0, 0x0e, 0);
    run();
    CHECK_EQ(g_status.obstruction, GDO_OBSTRUCTION_STATE_OBSTRUCTED);

    opener_v2(8000, GDO_CMD_PAIR_3_RESP, 0, 0x09, 0);
    opener_v2(8471, GDO_CMD_OBST_1, 0, 0, 0); // trailing clear edge must not re-trip
    run();
    CHECK_EQ(g_status.obstruction, GDO_OBSTRUCTION_STATE_CLEAR);
}

/* ---------------------------------------------------------------- commands */

static void test_door_open_sends_press_and_release(void) {
    start_v2();
    opener_sends(0, GDO_DOOR_STATE_CLOSED);
    run();
    s_queued_count = 0;
    uint32_t rolling = g_status.rolling_code;

    CHECK_EQ(gdo_door_open(), ESP_OK);
    CHECK_EQ(s_queued_count, 2);
    CHECK_EQ(s_queued[0].cmd, GDO_CMD_DOOR_ACTION);
    CHECK_EQ(s_queued[0].nibble, GDO_DOOR_ACTION_OPEN);
    CHECK_EQ(s_queued[0].byte1, 1); // pressed
    CHECK_EQ(s_queued[1].byte1, 0); // released
    // Both halves share one rolling code, and the counter advances once.
    CHECK_EQ(s_queued[0].rolling, rolling);
    CHECK_EQ(s_queued[1].rolling, rolling);
    CHECK_EQ(g_status.rolling_code, rolling + 1);
    CHECK_EQ(s_queued[0].fixed & 0xffffffff, g_status.client_id);
    CHECK_EQ(g_status.door_target, 0);
}

static void test_door_commands_noop_when_already_there(void) {
    start_v2();
    opener_sends(0, GDO_DOOR_STATE_OPEN);
    run();
    s_queued_count = 0;

    CHECK_EQ(gdo_door_open(), ESP_OK);
    CHECK_EQ(gdo_door_stop(), ESP_OK); // not moving
    CHECK_EQ(s_queued_count, 0);

    CHECK_EQ(gdo_door_close(), ESP_OK);
    CHECK_EQ(queued(GDO_CMD_DOOR_ACTION), 2);
    CHECK_EQ(last_queued(GDO_CMD_DOOR_ACTION)->nibble, GDO_DOOR_ACTION_CLOSE);
}

static void test_toggle_while_moving_stops(void) {
    start_v2();
    opener_sends(0, GDO_DOOR_STATE_OPENING);
    run();
    s_queued_count = 0;

    CHECK_EQ(gdo_door_toggle(), ESP_OK);
    CHECK_EQ(last_queued(GDO_CMD_DOOR_ACTION)->nibble, GDO_DOOR_ACTION_STOP);
}

static void test_light_and_lock_commands(void) {
    start_v2();
    opener_v2(0, GDO_CMD_STATUS, GDO_DOOR_STATE_CLOSED, 0x40, 0x00); // light off, unlocked
    run();
    s_queued_count = 0;

    CHECK_EQ(gdo_light_on(), ESP_OK);
    CHECK_EQ(last_queued(GDO_CMD_LIGHT)->nibble, GDO_LIGHT_ACTION_ON);
    CHECK_EQ(queued(GDO_CMD_GET_STATUS), 1); // confirm the new state

    CHECK_EQ(gdo_lock(), ESP_OK);
    CHECK_EQ(last_queued(GDO_CMD_LOCK)->nibble, GDO_LOCK_ACTION_LOCK);

    // Already off / unlocked: nothing to send.
    s_queued_count = 0;
    CHECK_EQ(gdo_light_off(), ESP_OK);
    CHECK_EQ(gdo_unlock(), ESP_OK);
    CHECK_EQ(s_queued_count, 0);
}

static void test_learn_and_clear_paired_devices(void) {
    start_v2();
    CHECK_EQ(gdo_activate_learn(), ESP_OK);
    CHECK_EQ(last_queued(GDO_CMD_LEARN)->nibble, GDO_LEARN_ACTION_ACTIVATE);

    s_queued_count = 0;
    CHECK_EQ(gdo_clear_paired_devices(GDO_PAIRED_DEVICE_TYPE_ALL), ESP_OK);
    CHECK_EQ(queued(GDO_CMD_CLEAR_PAIRED_DEVICES), 4); // one per device type
    CHECK_EQ(queued(GDO_CMD_GET_PAIRED_DEVICES), 1);
    CHECK_EQ(gdo_clear_paired_devices(GDO_PAIRED_DEVICE_TYPE_MAX), ESP_ERR_INVALID_ARG);
}

static void test_commands_are_paced_by_min_interval(void) {
    start_v2();
    CHECK_EQ(gdo_set_min_command_interval(200), ESP_OK);
    CHECK_EQ(gdo_light_toggle(), ESP_OK); // LIGHT + GET_STATUS
    wait_until(1000);
    run();

    CHECK_EQ(transmitted_count(), 2);
    const fake_tx_t *first = NULL, *second = NULL;
    transmitted(0, &first);
    transmitted(1, &second);
    CHECK(first && second && second->at_ms - first->at_ms >= 200);
    CHECK_EQ(tx_queue_depth(), 0);
}

RUN_TESTS(
    TEST(test_status_frame_updates_state_and_fires_callbacks),
    TEST(test_closed_sets_position_and_requests_openings),
    TEST(test_frames_with_our_client_id_are_ignored),
    TEST(test_light_action_from_opener),
    TEST(test_wall_button_press_and_release),
    TEST(test_motion_detected_then_clears),
    TEST(test_motion_ignored_before_sync),
    TEST(test_openings_paired_devices_battery_ttc),
    TEST(test_unsolicited_openings_ignored_until_known),
    TEST(test_obstruction_from_status_bit),
    TEST(test_obstruction_status_bit_ignored_without_option),
    TEST(test_long_obstruction_markers),
    TEST(test_door_open_sends_press_and_release),
    TEST(test_door_commands_noop_when_already_there),
    TEST(test_toggle_while_moving_stops),
    TEST(test_light_and_lock_commands),
    TEST(test_learn_and_clear_paired_devices),
    TEST(test_commands_are_paced_by_min_interval)
)
