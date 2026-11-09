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
 * TX queue ownership: queue_command() mallocs each packet and gdo_main_task frees it,
 * so every path that doesn't hand the packet to the queue must free it itself.
 */

#include <stdlib.h>

static int s_live_allocs;

static void *counted_malloc(size_t n) {
    void *p = malloc(n);
    if (p) {
        s_live_allocs++;
    }
    return p;
}

static void counted_free(void *p) {
    if (p) {
        s_live_allocs--;
    }
    free(p);
}

#define malloc(n) counted_malloc(n)
#define free(p) counted_free(p)

#include "harness.h"

#define TX_QUEUE_LEN 16

static void test_v2_queue_full_frees_packet(void) {
    start_v2();
    int base = s_live_allocs;

    for (int i = 0; i < TX_QUEUE_LEN; i++) {
        CHECK_EQ(get_status(), ESP_OK);
    }
    CHECK_EQ(s_live_allocs - base, TX_QUEUE_LEN);

    CHECK_EQ(get_status(), ESP_ERR_NO_MEM);
    CHECK_EQ(get_status(), ESP_ERR_NO_MEM);
    CHECK_EQ(s_live_allocs - base, TX_QUEUE_LEN);
}

static void test_v1_queue_full_frees_packet(void) {
    start_v1();
    int base = s_live_allocs;

    for (int i = 0; i < TX_QUEUE_LEN; i++) {
        CHECK_EQ(queue_v1_command(V1_CMD_QUERY_DOOR_STATUS), ESP_OK);
    }
    CHECK_EQ(s_live_allocs - base, TX_QUEUE_LEN);

    CHECK_EQ(queue_v1_command(V1_CMD_QUERY_DOOR_STATUS), ESP_ERR_NO_MEM);
    CHECK_EQ(s_live_allocs - base, TX_QUEUE_LEN);
}

static void test_sent_packets_are_freed(void) {
    start_v2();
    int base = s_live_allocs;

    CHECK_EQ(get_status(), ESP_OK);
    CHECK_EQ(get_openings(), ESP_OK);
    wait_until(500);
    run();

    CHECK_EQ(transmitted_count(), 2);
    CHECK_EQ(s_live_allocs - base, 0);
}

/* gdo_sync_task polls the opener with v1 commands, then on silence flips g_status.protocol
 * to v2 without draining gdo_tx_queue. A v1 packet still queued at that moment is
 * transmitted by the v2 path, which writes GDO_PACKET_SIZE (19) bytes from it. */
static void test_v1_packet_sent_after_switch_to_v2_stays_in_bounds(void) {
    start_v1();
    int base = s_live_allocs;
    CHECK_EQ(queue_v1_command(V1_CMD_QUERY_DOOR_STATUS), ESP_OK);

    g_status.protocol = GDO_PROTOCOL_SEC_PLUS_V2;
    run();

    CHECK_EQ(transmitted_count(), 1);
    CHECK_EQ(s_live_allocs - base, 0);
}

/* #58: a Sec+ 1.0 bus that stays busy keeps every queued poll stuck behind the collision
 * retry while the 250 ms emulation timer keeps polling. Memory must stay bounded, and
 * polling must resume once the bus clears. */
static void test_v1_busy_bus_stays_bounded_and_recovers(void) {
    start_v1();
    int base = s_live_allocs;

    esp_timer_create_args_t args = { .callback = v1_status_timer_cb, .name = "v1_status_timer" };
    esp_timer_handle_t poll;
    CHECK_EQ(esp_timer_create(&args, &poll), ESP_OK);
    CHECK_EQ(esp_timer_start_periodic(poll, 250 * 1000), ESP_OK);

    fake_gpio_level = 1;
    int peak = 0;
    for (uint32_t at = 5000; at <= 60000; at += 5000) {
        wait_until(at);
        run();
        if (s_live_allocs - base > peak) {
            peak = s_live_allocs - base;
        }
    }
    CHECK_EQ(transmitted_count(), 0);
    CHECK_EQ(tx_queue_depth(), TX_QUEUE_LEN);
    // Each queued packet plus at most one pending collision-retry event per packet.
    CHECK(peak <= 2 * TX_QUEUE_LEN);
    int busy_allocs = s_live_allocs - base;

    fake_gpio_level = 0;
    wait_until(70000);
    run();
    CHECK(transmitted_count() > 0);
    CHECK(tx_queue_depth() <= 1);
    CHECK(s_live_allocs - base < busy_allocs);

    int sent = transmitted_count();
    wait_until(75000);
    run();
    // Back to one poll per 250 ms tick.
    CHECK(transmitted_count() - sent >= 18);
}

RUN_TESTS(
    TEST(test_v2_queue_full_frees_packet),
    TEST(test_v1_queue_full_frees_packet),
    TEST(test_sent_packets_are_freed),
    TEST(test_v1_packet_sent_after_switch_to_v2_stays_in_bounds),
    TEST(test_v1_busy_bus_stays_bounded_and_recovers)
)
