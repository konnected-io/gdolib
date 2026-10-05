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
 * Security+ 1.0: decoding the 2-byte command/response pairs on the bus, and the
 * press/release toggles the public API sends.
 */

#include "harness.h"

/* Counts the single bytes written to the UART that equal cmd. */
static int transmitted_v1(uint8_t cmd) {
    int n = 0;
    const fake_tx_t *tx;
    for (int i = 0; transmitted(i, &tx); i++) {
        n += tx->len == 1 && tx->bytes[0] == cmd;
    }
    return n;
}

/* ---------------------------------------------------------------- decode */

static void test_door_status_responses(void) {
    static const struct {
        uint8_t resp;
        gdo_door_state_t door;
    } cases[] = {
        {0x02, GDO_DOOR_STATE_OPEN},
        {0x05, GDO_DOOR_STATE_CLOSED},
        {0x01, GDO_DOOR_STATE_OPENING},
        {0x04, GDO_DOOR_STATE_CLOSING},
        {0x06, GDO_DOOR_STATE_STOPPED},
    };

    start_v1();
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        opener_v1(i * 1000, V1_CMD_QUERY_DOOR_STATUS, cases[i].resp);
        run();
        CHECK_EQ(g_status.door, cases[i].door);
    }
    CHECK_EQ(s_cb_events[GDO_CB_EVENT_DOOR_POSITION], 5);
}

static void test_other_status_light_and_lock(void) {
    start_v1();
    opener_v1(0, V1_CMD_QUERY_OTHER_STATUS, 0x04); // light bit set, lock bit clear = locked
    run();
    CHECK_EQ(g_status.light, GDO_LIGHT_STATE_ON);
    CHECK_EQ(g_status.lock, GDO_LOCK_STATE_LOCKED);

    opener_v1(1000, V1_CMD_QUERY_OTHER_STATUS, 0x08);
    run();
    CHECK_EQ(g_status.light, GDO_LIGHT_STATE_OFF);
    CHECK_EQ(g_status.lock, GDO_LOCK_STATE_UNLOCKED);
}

static void test_obstruction(void) {
    start_v1();
    opener_v1(0, V1_CMD_OBSTRUCTION, 0x01);
    run();
    CHECK_EQ(g_status.obstruction, GDO_OBSTRUCTION_STATE_OBSTRUCTED);

    opener_v1(1000, V1_CMD_OBSTRUCTION, 0x00);
    run();
    CHECK_EQ(g_status.obstruction, GDO_OBSTRUCTION_STATE_CLEAR);
}

static void test_wall_button(void) {
    start_v1();
    opener_v1(0, V1_CMD_TOGGLE_DOOR_PRESS, 0x00);
    run();
    CHECK_EQ(g_status.button, GDO_BUTTON_STATE_PRESSED);

    opener_v1(300, V1_CMD_TOGGLE_DOOR_RELEASE, 0x00);
    run();
    CHECK_EQ(g_status.button, GDO_BUTTON_STATE_RELEASED);
}

static void test_leading_garbage_byte_is_skipped(void) {
    start_v1();
    step_t *s = add_step(0);
    static const uint8_t bytes[] = {0x99, V1_CMD_QUERY_DOOR_STATUS, 0x02};
    add_bytes(s, bytes, sizeof(bytes));
    s->data_size = sizeof(bytes);
    run();
    CHECK_EQ(g_status.door, GDO_DOOR_STATE_OPEN);
}

static void test_packet_split_across_reads(void) {
    start_v1();
    static const uint8_t cmd = V1_CMD_QUERY_DOOR_STATUS, resp = 0x05;
    step_t *s = add_step(0);
    add_bytes(s, &cmd, 1);
    s->data_size = 1;
    s = add_step(5);
    add_bytes(s, &resp, 1);
    s->data_size = 1;
    run();
    CHECK_EQ(g_status.door, GDO_DOOR_STATE_CLOSED);
}

static void test_panel_query_0x37_asks_for_other_status(void) {
    start_v1();
    opener_v1(0, V1_CMD_QUERY_DOOR_STATUS_0x37, 0x00);
    run();
    CHECK_EQ(queued(V1_CMD_QUERY_OTHER_STATUS), 1);
}

/* ---------------------------------------------------------------- commands */

static void test_door_open_is_a_press_then_release(void) {
    start_v1();
    opener_v1(0, V1_CMD_QUERY_DOOR_STATUS, 0x05); // closed
    run();

    CHECK_EQ(gdo_door_open(), ESP_OK);
    wait_until(1000);
    run();

    const fake_tx_t *press = NULL, *release = NULL;
    transmitted(0, &press);
    transmitted(1, &release);
    CHECK(press && press->len == 1 && press->bytes[0] == V1_CMD_TOGGLE_DOOR_PRESS);
    CHECK(release && release->len == 1 && release->bytes[0] == V1_CMD_TOGGLE_DOOR_RELEASE);
    CHECK(press && release && release->at_ms - press->at_ms >= g_tx_delay_ms);
}

static void test_light_and_lock_toggles(void) {
    start_v1();
    opener_v1(0, V1_CMD_QUERY_OTHER_STATUS, 0x08); // light off, unlocked
    run();

    CHECK_EQ(gdo_light_on(), ESP_OK);
    wait_until(500);
    run();
    CHECK_EQ(gdo_lock(), ESP_OK);
    wait_until(1000);
    run();

    CHECK_EQ(transmitted_v1(V1_CMD_TOGGLE_LIGHT_PRESS), 1);
    CHECK_EQ(transmitted_v1(V1_CMD_TOGGLE_LIGHT_RELEASE), 1);
    CHECK_EQ(transmitted_v1(V1_CMD_TOGGLE_LOCK_PRESS), 1);
    CHECK_EQ(transmitted_v1(V1_CMD_TOGGLE_LOCK_RELEASE), 1);
}

static void test_v2_only_features_not_supported(void) {
    start_v1();
    CHECK_EQ(gdo_activate_learn(), ESP_ERR_NOT_SUPPORTED);
    CHECK_EQ(gdo_deactivate_learn(), ESP_ERR_NOT_SUPPORTED);
    CHECK_EQ(gdo_clear_paired_devices(GDO_PAIRED_DEVICE_TYPE_ALL), ESP_ERR_NOT_SUPPORTED);
}

static void test_toggle_only_open_from_stopped_while_opening(void) {
    start_v1();
    gdo_set_toggle_only(true); // what a completed v1 sync sets
    opener_v1(0, V1_CMD_QUERY_DOOR_STATUS, 0x01);   // opening
    opener_v1(2000, V1_CMD_QUERY_DOOR_STATUS, 0x06); // stopped
    run();
    CHECK_EQ(g_status.last_move_direction, GDO_DOOR_STATE_OPENING);

    // A single toggle would now close the door, so open is toggle (starts closing),
    // toggle (stops), toggle (opens): three presses.
    CHECK_EQ(gdo_door_open(), ESP_OK);
    wait_until(4000);
    run();
    CHECK_EQ(transmitted_v1(V1_CMD_TOGGLE_DOOR_PRESS), 3);
    CHECK_EQ(transmitted_v1(V1_CMD_TOGGLE_DOOR_RELEASE), 3);
}

static void test_toggle_only_close_from_open_is_one_toggle(void) {
    start_v1();
    gdo_set_toggle_only(true);
    opener_v1(0, V1_CMD_QUERY_DOOR_STATUS, 0x02); // open
    run();

    CHECK_EQ(gdo_door_close(), ESP_OK);
    wait_until(2000);
    run();
    CHECK_EQ(transmitted_v1(V1_CMD_TOGGLE_DOOR_PRESS), 1);
    CHECK_EQ(g_status.door_target, 10000);
}

RUN_TESTS(
    TEST(test_door_status_responses),
    TEST(test_other_status_light_and_lock),
    TEST(test_obstruction),
    TEST(test_wall_button),
    TEST(test_leading_garbage_byte_is_skipped),
    TEST(test_packet_split_across_reads),
    TEST(test_panel_query_0x37_asks_for_other_status),
    TEST(test_door_open_is_a_press_then_release),
    TEST(test_light_and_lock_toggles),
    TEST(test_v2_only_features_not_supported),
    TEST(test_toggle_only_open_from_stopped_while_opening),
    TEST(test_toggle_only_close_from_open_is_one_toggle)
)
