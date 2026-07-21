# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

`gdolib` is an ESP-IDF component (not a standalone app) that drives Chamberlain/LiftMaster garage door openers over Security+ v1 and v2. It is statically linked into a host ESP32 project and exposes a C API in [include/gdo.h](include/gdo.h).

## Build

This repo is consumed as an ESP-IDF component, so it is built from a host project, not on its own. The example under [examples/](examples/) is the canonical host project for sanity-building the library.

```sh
# from a host project that includes this as a component (e.g. examples/)
. $IDF_PATH/export.sh        # if not already on PATH
idf.py build                 # produces build/esp-idf/gdolib/libgdolib.a
idf.py -p <PORT> flash monitor
```

There is no test suite, linter, or formatter configured. Requires ESP-IDF v4.4+.

The component declares its sources in [CMakeLists.txt](CMakeLists.txt) (`gdo.c`, `gdo_utils.c`, `secplus.c`); new `.c` files must be added there or they will not be compiled.

### Debugging the protocol against a live opener

Some questions — obstruction timing, undocumented frames, edge semantics — can only be
answered by watching real Security+ frames on a live device. The full loop (build a debug
gdolib, OTA-flash it via ESPHome, stream logs, and correlate) plus the non-obvious traps
(local IDF `path:` component, ESPHome ≥ 2026.7.0 native build, un-stripping gdolib's
`ESP_LOGx`, avoiding log-stream frame drops, OTA/WiFi safety) is documented in
[docs/live-capture-workflow.md](docs/live-capture-workflow.md). Reach for it when a
protocol change would otherwise be a guess.

## Architecture

### Three translation units, one library

- [gdo.c](gdo.c) — the entire driver: public API, state machine, packet decode/encode, FreeRTOS tasks, ISRs, timers. Almost all logic lives here.
- [gdo_utils.c](gdo_utils.c) — state-enum-to-string tables (`gdo_door_state_str[]`, etc.) referenced via `extern` in [gdo_priv.h](gdo_priv.h).
- [secplus.c](secplus.c) / [secplus.h](secplus.h) — vendored from [argilo/secplus](https://github.com/argilo/secplus). Pure encode/decode of Security+ v1, v2, and wireline frames. Do not edit unless syncing upstream.

Public surface is [include/gdo.h](include/gdo.h); internals shared between `gdo.c` and `gdo_utils.c` go in [gdo_priv.h](gdo_priv.h).

### Runtime model

The driver runs as two FreeRTOS tasks plus several ESP timers, communicating through two queues:

- **`gdo_event_queue`** — fed by both the UART driver (raw `uart_event_t`) and our own `gdo_event_t` enum values posted from ISRs/timers/API calls. Drained by `gdo_main_task`, which decodes incoming packets, runs the state machine, and fires the user's `gdo_event_callback_t`.
- **`gdo_tx_queue`** — outbound `gdo_tx_message_t` packets. `gdo_main_task` pulls from this queue and writes to UART, respecting `g_tx_delay_ms` (min interval between commands).
- **`gdo_main_task`** ([gdo.c:1434](gdo.c#L1434)) — the long-lived event loop.
- **`gdo_sync_task`** ([gdo.c:823](gdo.c#L823)) — short-lived; auto-detects the opener's protocol on startup, then deletes itself. See "Protocol detection" below.

All shared state lives in the file-static `g_status` struct ([gdo.c:67](gdo.c#L67)), protected by the `gdo_spinlock` portMUX. `gdo_get_status()` snapshots it under the spinlock; direct readers inside `gdo.c` generally do not lock individual fields, so when adding fields think about whether the existing critical section in `gdo_get_status` is enough.

The `gdo_tx_queue` handle doubles as the "is initialized" flag (see `gdo_init` / `gdo_deinit` / `gdo_start`). `gdo_main_task_handle` doubles as the "is started" flag. Preserve these proxy checks if you refactor lifecycle.

### Protocol detection (Security+ v1 vs v2)

`gdo_sync_task` decides between three protocols at runtime ([gdo.c:823](gdo.c#L823)):
1. Start UART at 1200 baud, even parity (Sec+ v1). Wait for a V1 wall panel to chatter.
2. If a panel is heard → `GDO_PROTOCOL_SEC_PLUS_V1_WITH_SMART_PANEL`.
3. If silent → emulate a V1 panel by polling at 250 ms.
4. If the opener still doesn't respond → switch to 9600 baud, no parity (`GDO_PROTOCOL_SEC_PLUS_V2`) and run the v2 sync handshake (`GET_STATUS` → `GET_OPENINGS` → `GET_PAIRED_DEVICES`).

Callers can short-circuit this by calling `gdo_set_protocol()` before `gdo_start()` (sets `g_protocol_forced`). For Sec+ v2 they should also call `gdo_set_client_id()` and `gdo_set_rolling_code()` from persisted storage before `gdo_start()` — otherwise the rolling code starts at 0 and the opener will reject commands.

Packet sizing is protocol-dependent: `GDO_PACKET_SIZE` ([gdo_priv.h:30](gdo_priv.h#L30)) returns 19 for v2, 2 for v1.

### Obstruction sensing (two paths)

Configured via `gdo_config_t.obst_from_status`:
- `false` (default): a GPIO ISR on `obst_in_pin` counts pulses; `obst_timer` periodically classifies pulse density as obstructed/clear.
- `true`: derive obstruction from the OBST_1/OBST_2 bits in incoming v2 status frames. This is faster but only works on Sec+ v2.

### Door position model

Position is tracked as 0–10000 (0 = open, 10000 = closed). Openers don't report position directly — `gdo_door_move_to_target()` integrates `open_ms` / `close_ms` (set via `gdo_set_open_duration` / `gdo_set_close_duration`) against elapsed time. If those durations aren't known, `move_to_target` fails with `ESP_ERR_INVALID_STATE`.

`toggle_only` mode is for openers without a real status line: there is no "open" or "close" command, only a single toggle, so the driver tracks `last_move_direction` and may schedule a double-toggle to break out of a stopped state ([gdo.c:393](gdo.c#L393)).

## Conventions

- All public functions return `esp_err_t`. Return `ESP_OK` on success and a specific `ESP_ERR_*` on failure — the existing kdoc comments document the contract for each.
- New events go in both `gdo_cb_event_t` (public, in [include/gdo.h](include/gdo.h)) and `gdo_event_type_t` (internal, in [gdo_priv.h](gdo_priv.h)). Public events drive the user callback; internal events drive the queue.
- New state-string entries must be appended in the same order as the enum and to both the enum in [include/gdo.h](include/gdo.h) and the `*_str[]` array in [gdo_utils.c](gdo_utils.c) — they are indexed by enum value.
- License headers: every file is GPL-3.0-or-later. Preserve them; vendored `secplus.*` are also GPL-3.0-or-later but credit Clayton Smith.
