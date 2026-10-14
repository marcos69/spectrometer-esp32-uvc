# ESP32-S3 Spectrometer

An ESP32-S3 firmware that turns a cheap USB webcam (UVC) + diffraction-grating
frontend into a working spectrometer — a port of
[PySpectrometer](https://github.com/leswright1979/PySpectrometer) (v3.2) to
ESP-IDF 5.5, with the spectrum served on a browser page over WiFi.

```
light -> slit -> grating -> webcam sensor -> ESP32-S3 (USB host, UVC)
      -> row DSP (Savitzky-Golay + peak find) -> web UI (live graph)
```

## Hardware

| Part | Detail |
|---|---|
| MCU | ESP32-S3, 8 MB flash, octal PSRAM (frame buffers + PNG scratch in PSRAM) |
| Camera | Generic UVC webcam on the **left** USB-C (native USB pins → OTG). Any 640×480-class webcam with MJPEG works; the reference unit also offers YUY2 320×240 |
| Serial | **Right** USB-C = USB-UART bridge → UART0 logging (115200 8N1). It can never carry the camera |
| PSU | Powered hub recommended; the camera plus WiFi radios strain a single port |

## Features

- **Live web UI** (`http://<device-ip>/`): canvas graph, false-color bars,
  full nm graticule. No app install, works from phone or laptop.
- **Fixed full-scale axis 1250 nm → 22 nm** (left→right; the sensor's
  dispersion was verified against two known lines: 635 nm @ px201,
  570 nm @ px222). The ruler never moves — zoom/pan is the viewport.
- **Calibration** without touching the axis: click two known peaks, type their
  nm, apply — peak labels, the `peaks:` line and the cursor readout switch to
  calibrated nm. `reset cal` returns to the fixed-ruler estimate. Calibration
  is stored in NVS and survives power cycles.
- **Zoom & pan**: wheel zooms around the cursor, drag pans the graph directly,
  sliders stay synced.
- **Two capture formats with live toggle**: MJPEG 800×600@15 (workhorse) and
  raw YUY2 320×240@5 (no JPEG loss, see warning below). The toggle reopens
  the stream without a reboot.
- **Snapshot PNG / CSV** (`/snapshot.png`, `/snapshot.csv`), rendered with the
  same fixed ruler and calibrated labels as the web page.
- Peak hold, threshold/peak-width/filter-order/gain sliders, manual exposure
  (as far as the camera honors it), WiFi setup portal.

## Known issues

### ⚠ YUY2 (raw) mode: the sensor crop guts the signal

The camera **crops** — does not rescale — when it switches to smaller modes.
The spectral line sits at ~74 % of the frame height in the native 800×600
image, but the only uncompressed mode offered is 320×240, whose crop cuts the
line almost entirely out of frame. Result in YUY2: a usable-looking stream
(valid YUY2 bytes, verified to the byte) but a flat ~16–28 intensity graph —
only the *tip* of the line pokes into the bottom row band.

This is an optical/framing limitation, **not** a decode bug. Two ways out:

1. **Re-aim mechanically** — tilt/shift the camera (or the slit) so the line
   rides *higher* in the small-mode crop. Watch the raw YUY2 graph while you
   adjust: the moment the line enters the frame, YUY2 becomes the precision
   mode it was meant to be.
2. **Measure in MJPEG 800×600** — fully working today; JPEG loss on a bright
   spectral line is minor.

### Other camera quirks (this model, likely firmware-limited)

- **Exposure control is refused**: every UVC exposure control transfer
  STALLs, and the failure is cached by the camera controller. The gain
  slider (display gain) is the working light lever. Auto-exposure stays on.
- The camera's ~300 KB/s total encode/stream budget makes 640×480@30
  unusable (a stream of unfinished proto-JPEGs) — the format picker
  settles on 800×600@15, and the restart loop rotates formats on decode
  failures until it finds a working mode.
- Occasional `buffer exhaustion` warnings in the log are dropped USB frames;
  harmless for the graph.

## Build & flash

```bash
# ESP-IDF 5.5.x (env: python 3.11 shim first — see tools/build.sh)
bash tools/build.sh                  # -> build/spectrometer.bin
```

Over the network serial bridge (ser2net on a Mac, rfc2217):

```bash
cd build
python -m esptool --port rfc2217://HOST:2217 --chip esp32s3 \
  --before no-reset --after no-reset write_flash \
  --flash-mode dio --flash-freq 80m --flash-size 8MB \
  0x0 bootloader/bootloader.bin 0x8000 partition_table/partition-table.bin \
  0x10000 spectrometer.bin
```

Over local USB instead: put the board in download mode (hold BOOT, tap RST)
and let `idf.py flash` do the same. After flashing, tap RST to boot.

> **Serial bridge note:** the capture tool must disconnect before esptool
> takes the port and can be re-armed after (single-client rfc2217).
> DTR/RTS over the bridge cannot enter download mode — do it manually.

## Code map

```
main/app_main.c     boot: NVS -> camera -> WiFi -> web
main/camera.c       UVC host: frame list, format picker (+ ladder), frame
                    loop, MJPEG bisect/decode, YUY2 Y-plane, row window,
                    exposure prefs (NVS), diagnostics
main/web.c          HTTP: page, /api/*, snapshots; embedded PAGE_HTML
main/web_index.html UI source of truth -> tools/html2c.py -> main/web_index.h
                    (auto-generated; never edit the header)
main/render.c       snapshot PNG: fixed-ruler graph, hand-rolled store-deflate
main/net.c          WiFi STA + AP fallback, credentials in NVS
components/spectra  DSP: ring buffer, S-G filter, threshold/peak find, cal,
                    gain, hold — pure C, UI-agnostic
components/jpeg_decode  vendored libjpeg (half-scale gray decode)
managed_components/espressif__usb_host_uvc
                    vendored UVC host driver + LOCAL PATCHES: uvc_isoc.c
                    frame-delimiting gates + uvc_types_priv.h frame timestamp
                    + CMakeLists esp_timer require. Component re-download
                    (dependency solve / idf.py reconfigure on a fresh tree)
                    silently LOSES all three — reapply together.
```

## Licenses & attribution

| Code | License | Notes |
|---|---|---|
| This project | Apache-2.0 (`LICENSE`) | © Marc; contributions by Hermes Agent |
| [PySpectrometer](https://github.com/leswright1977/PySpectrometer) (concept + algorithms this port follows) | Apache-2.0 | by Les Wright |
| `components/jpeg_decode` (vendored IJG libjpeg, heavily trimmed) | IJG (see its `README`) | kept intact in-tree; *This software is based in part on the work of the Independent JPEG Group* |
| `espressif/usb_host_uvc` (fetched by the component manager) | Apache-2.0 | **not committed** — fetched by `idf.py`; local patches live in `patches/usb_host_uvc_patches.md` |

The vendored jpeg_decode tree keeps its original `README` (which contains the
IJG license statement) — do not strip it. Publishing them together is
permitted by both licenses; keep the LICENSE files, name the origin of
everything you didn't write, and mark modified upstream files
(Apache-2.0 §4) — this README + the patch doc do that.

## Building from a fresh checkout

`managed_components/` is git-ignored (the component manager fetches it), so a
fresh clone needs the three local patches from
[`patches/usb_host_uvc_patches.md`](patches/usb_host_uvc_patches.md) reapplied
after the dependency install:

```bash
idf.py set-target esp32s3
idf.py add-dependency "espressif/usb_host_uvc"   # or just idf.py reconfigure
# apply patches/usb_host_uvc_patches.md to managed_components/espressif__usb_host_uvc
idf.py build
```

`dependencies.lock` is committed so everyone resolves the same component
versions. The JVM-free JPEG decode path is exercised by
`tools/test_jpegd.py` (bit-exactness vs PIL) if you touch the codec.

## HTTP API

| Endpoint | Method | Purpose |
|---|---|---|
| `/` | GET | UI page (`Cache-Control: no-store` — the page changes with builds) |
| `/api/data` | GET | full state: `n, int[], peaks[{px,nm,val}], cal[4], md, th, or, g, hold, fmt, w, h, fps, frames, stream, net, ip, raw, exp` |
| `/api/cal` | POST | `px1=&w1=&px2=&w2=` (nm 250..1200) or `reset=1` |
| `/api/settings` | POST | `md=` `th=` `or=` `g=` `hold=0|1` `rhold=1` |
| `/api/cam` | POST | `raw=0|1` (live stream reopen) `exp=` µs |
| `/api/net` | POST | `ssid=&pass=` (persists, reconnects STA) |
| `/preview` | GET | latest JPEG frame |
| `/snapshot.png` `/snapshot.csv` | GET | rendered graph / raw columns |

## Diagnostics

Serial is the ground truth. Useful tags: `camera` (opens, decode fails,
`cam alive:` heartbeat, per-frame band stats after each reopen), `uvc-isoc`
(wire counters: `fid_toggles/eof_bits` per window — trust these over
size arithmetic), `web` (`api/data: n=… int=lo..hi peaks=…` proves the
server path end-to-end), `render` (PNG scratch).

Quick triage:
- Graph draws but no bars → check `/api/data` `int[]` (server) before JS.
- `int=` confined to a few LSBs → underexposed slit, add light (or gain).
- Format flapping → watch `pick:` / `ladder:` / `stream opened:` lines and
  wire counters; the wire budget rule lives in `main/camera.c` comments.
