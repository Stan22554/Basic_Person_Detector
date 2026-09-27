# LD2410C ESP32 Presence Detector

A PlatformIO / Arduino project for an ESP32 and LD2410C. It serves a responsive live dashboard with a color-and-intensity presence indicator, target measurements, link diagnostics, and a rolling log of raw UART frames.

## Wiring

| LD2410C | ESP32 |
|---|---|
| TX | RX2 / GPIO16 (ESP32 receives sensor data) |
| RX | TX2 / GPIO17 (ESP32 sends UART data) |
| OUT | GPIO27 |
| VCC | Board's supported supply voltage / VCC |
| GND | GND |

UART2 runs at **256000 baud, 8N1**, which is the LD2410C default. Ensure your ESP32 board and sensor share a common ground. The sensor uses 3.3 V UART logic; check the board's supply pin and the sensor module documentation before powering it.

> GPIO16/17 and GPIO27 are assumptions for a classic ESP32 DevKit. If your board maps RX2/TX2 differently, change `SENSOR_RX_PIN`, `SENSOR_TX_PIN`, and `SENSOR_OUT_PIN` near the top of `src/main.cpp`.

## Build and flash

1. Open this folder in VS Code with the PlatformIO extension installed.
2. Build and upload the `esp32dev` environment.
3. Open the serial monitor at 115200 baud for the dashboard URL.
4. Connect to Wi-Fi access point **LD2410C-Presence** (password **presence**) and visit **http://192.168.4.1/**.

The ESP32 starts its own access point. To also connect it to an existing Wi-Fi network, either copy `include/config.h.example` to `include/config.h` and set `WIFI_SSID` and `WIFI_PASSWORD`, or configure the network in the dashboard. Wi-Fi credentials entered in the dashboard are stored in ESP32 NVS and take precedence over the compile-time values. Use **Clear Wi-Fi** in the dashboard to remove them; the setup AP stays available.

The dashboard's **Configuration** section can also set moving and stationary sensitivity (0-100) for each of the LD2410C's nine gates, maximum range (gate 0-8, in 0.75 m steps), and the sensor's no-target hold timeout in seconds. Sensor settings are sent over UART and restored after reboot. Sensitivity and maximum range are gate-specific; the LD2410C timeout is a single global setting, not an individual duration for every gate.

## Dashboard details

- **No Presence** uses a muted LED; **Moving Presence** is amber; **Non-Moving Presence** is teal. If the sensor reports both moving and stationary targets simultaneously, the LED is violet.
- LED brightness / scale indicate proximity, using the nearest active moving or stationary distance (closer is brighter). The distance-to-intensity scale assumes the sensor's nominal 0–600 cm reporting range. The sensor's factory distance resolution is typically 0.75 m per gate, so its reported distance is an estimate and may not match a tape measure at close range; 0.20 m resolution is configurable on the sensor.
- Numeric readings include moving and stationary distances and energy values, detection distance, and the raw target-state code. The information grid shows OUT level, report age, frame/byte counts, Wi-Fi IP, UART configuration, and pin assignments.
- The transaction table shows recent complete UART frames (including non-target/configuration frames) as timestamped hex, with payload length and footer validity. It keeps up to 40 transactions in the ESP32 ring buffer and the browser displays the newest 40.
- Standard target reports are decoded as data type `0x02`, header marker `0xAA`, nine target-data bytes, tail marker `0x55`, and trailing `0x00`. Target distances are little-endian centimeters. Other UART frame types are retained in the raw transaction stream but are not interpreted.
- Dashboard configuration changes are validated before saving. A sensor configuration is reported as applied only when the LD2410C acknowledges its UART commands; changes are persisted in ESP32 NVS so they can be retried on reboot.

No external Arduino libraries are required; the project uses the ESP32 Arduino core's `WiFi` and `WebServer` libraries.
