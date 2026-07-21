# Live-device capture workflow (Security+ 2.0 protocol debugging)

How to build a debug gdolib, OTA-flash it to a live opener, and stream/correlate
the wire traffic. This is the loop used to resolve issues #27/#28/#29 — reach for it
whenever a protocol question can only be answered by watching real frames.

The host project is [`konnected-esphome`](https://github.com/konnected-io/konnected-esphome),
which consumes gdolib. Paths below are from the setup used originally; adjust to yours.

## TL;DR

```sh
# 1. Instrument gdo.c (see "Instrumentation" below), then from the esphome host repo:
cd ~/workspace/konnected-esphome
esphome compile <local-config>.yaml          # build against local gdolib
esphome upload  <local-config>.yaml --device <device-ip>   # OTA
esphome logs    <local-config>.yaml --device <device-ip> > live.log &   # stream

# 2. Physically exercise the device; correlate:
grep -aE '#28CAP|cmd=|Obstruction' live.log
```

## One-time host-config setup

Copy the stock device config (e.g. `garage-door-GDOv2-Q.yaml`) to a `*-LOCAL-*.yaml`
and make four changes. All four matter — each corresponds to a trap hit the first time.

### 1. Point gdolib at your local working copy (NOT a git ref / lib_deps)

gdolib is an **ESP-IDF managed component**, added via `esp32.framework.components`.
Use a local `path:` — do **not** use `platformio_options.lib_deps: symlink://…`
(PlatformIO's library manager rejects gdolib with `MissingPackageManifestError`
because it has no `library.json`).

```yaml
esp32:
  framework:
    components:
      - name: gdolib
        path: /absolute/path/to/gdolib     # your working copy; edits are picked up on rebuild
```

### 2. Use ESPHome >= 2026.7.0 (native ESP-IDF build)

Older ESPHome builds through a **PlatformIO-wrapped** esp-idf toolchain that silently
mis-links a local IDF `path:` component (it compiles a stale `.pioenvs/…/gdo.c.o` and
your edits never make it into the firmware). ESPHome 2026.7.0+ defaults to a **native
IDF build** (`.esphome/build/<name>/build/`, `Discovering available ESP-IDF components`)
that handles `path:` deps correctly.

```sh
brew upgrade esphome        # or however you installed it
esphome version             # want >= 2026.7.0
```

### 3. Un-strip gdolib's ESP-IDF logs (the big one)

gdolib logs via raw ESP-IDF `esp_log.h` (`ESP_LOGI/W/D`), **not** ESPHome's logger.
The native build defaults `CONFIG_LOG_MAXIMUM_LEVEL` to **ERROR**, which **compile-strips
every gdolib log call** — the format strings aren't even in the binary. Raise the
compile-time maximum and the runtime default:

```yaml
esp32:
  framework:
    sdkconfig_options:
      CONFIG_LOG_MAXIMUM_LEVEL_VERBOSE: "y"   # compile the logs IN
      CONFIG_LOG_DEFAULT_LEVEL_INFO: "y"      # emit them at runtime
```

Verify the strings actually made it into the firmware before flashing:

```sh
strings .esphome/build/<name>/build/firmware.elf | grep -c '#28CAP'   # expect >= 1
```

### 4. Keep the log stream from dropping frames

The wifi driver spews DEBUG (`bssid equal: ss_state=…`, ~7 lines/s) that **cannot** be
silenced with `esp_log_level_set()` and congests the API log stream — during a burst it
backs up several seconds and **drops frames** (detectable as a gap in the rolling-code
sequence). Set the ESPHome logger to **INFO** so its router discards all `[D]` noise;
gdolib's frame lines (`ESP_LOGI`) and your `#28CAP` markers (`ESP_LOGW`) survive, as do
`[S]` sensor-state lines. Remove any per-tag `logs:` overrides more verbose than INFO
(ESPHome rejects the config otherwise).

```yaml
logger:
  level: INFO
```

(`esp_log_level_set()` in an `on_boot` lambda *does* work for httpd/mdns/nvs/netif — worth
adding to trim those — but not for the wifi driver's internal debug.)

## Instrumentation

Add a millisecond-stamped, greppable line in `decode_packet()` (gdo.c). ESPHome's log
header is only second-resolution, so the in-message `t=` (ms since boot) is what makes
inter-frame timing measurable. Keep it **log-only** so device behaviour is unchanged:

```c
uint32_t time_now = esp_timer_get_time() / 1000;   // already computed in decode_packet
/* ... after cmd/nibble/byte1/byte2 are decoded ... */
if (cmd == GDO_CMD_OBST_1 || cmd == GDO_CMD_OBST_2 ||
    cmd == GDO_CMD_PAIR_3_RESP || cmd == GDO_CMD_STATUS) {
    ESP_LOGW(TAG,
             "#28CAP t=%" PRIu32 "ms cmd=%03x(%s) byte2=%02x byte1=%02x nibble=%01x "
             "statusObstBit=%d reported=%s",
             time_now, cmd, cmd_to_string(cmd), byte2, byte1, nibble,
             cmd == GDO_CMD_STATUS ? (int)((byte1 >> 6) & 1) : -1,
             gdo_obstruction_state_to_string(g_status.obstruction));
}
```

`WARN` level + a unique token (`#28CAP`) make it trivial to grep out of the stream.
`reported=` is `g_status` *before* this frame is handled, so you can watch state
transitions frame-by-frame. This is a throwaway capture aid — **do not commit it**;
stash it (`git stash push gdo.c`) while working on real fixes.

## OTA safety on a live device

- Build from a copy of the **device's own running config** (or the stock config with
  captive-portal on). WiFi credentials live in NVS and survive OTA, so the device
  rejoins the network after the reboot. Never flash a config with wrong/missing WiFi
  or you'll knock it offline and need physical recovery.
- OTA is unauthenticated on this hardware (`ota: platform: esphome`, no password), so
  `--device <ip>` uploads to the running firmware regardless of device name.
- The instrumentation is log-only, so the flashed firmware behaves exactly like the
  branch it's built from — safe to leave on temporarily, but reflash the released
  firmware when done.

## Capturing and correlating

```sh
esphome logs <config>.yaml --device <ip> > live.log &     # continuous capture

# Obstruction-relevant frames, denoised, one line each:
grep -aE '#28CAP|cmd=084|cmd=085|cmd=0a1|Obstruction' live.log \
  | sed -E "s/^\[([0-9:.]+)\].*gdolib: /[\1] /"
```

- Go one physical scenario at a time; the events stand out against idle chatter.
- **Trust the in-message `t=` (device ms), not the `[HH:MM:SS]` wall-clock** — the
  network stream can lag/batch several seconds under load.
- **Check rolling-code continuity** (`received rolling=…`) around any burst; a gap
  means dropped log frames — quiet the noise (step 4) and re-run before trusting counts.

## Reverting when done

```sh
git stash pop            # or: git checkout gdo.c   to drop the instrumentation
# reflash the device with its released firmware
```
