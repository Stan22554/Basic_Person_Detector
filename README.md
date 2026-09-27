# LD2410C ESP32 Presence Detector

A PlatformIO / Arduino project for an ESP32 and LD2410C. It serves a responsive live dashboard with a color-and-intensity presence indicator, target measurements, link diagnostics, and a rolling log of raw UART frames.

## Wiring

| LD2410C | ESP32 |
|---|---|
| TX | RX2 / GPIO16 (ESP32 receives sensor data) |
| RX | TX2 / GPIO17 (ESP32 sends UART data) |
| OUT | GPIO2 (D2 on boards that label that pin D2) |
| VCC | Board's supported supply voltage / VCC |
| GND | GND |

UART2 runs at **256000 baud, 8N1**, which is the LD2410C default. Ensure your ESP32 board and sensor share a common ground. The sensor uses 3.3 V UART logic; check the board's supply pin and the sensor module documentation before powering it.

> GPIO16/17 and GPIO2 are assumptions for a classic ESP32 DevKit. If your board's RX2/TX2 or D2 map differently, change `SENSOR_RX_PIN`, `SENSOR_TX_PIN`, and `SENSOR_OUT_PIN` near the top of `src/main.cpp`.

## Build and flash

1. Open this folder in VS Code with the PlatformIO extension installed.
2. Build and upload the `esp32dev` environment.
3. Open the serial monitor at 115200 baud for the dashboard URL.
4. Connect to Wi-Fi access point **LD2410C-Presence** (password **presence**) and visit **http://192.168.4.1/**.

The ESP32 starts its own access point. To also connect it to an existing Wi-Fi network, copy `include/config.h.example` to `include/config.h` and set `WIFI_SSID` and `WIFI_PASSWORD`. The setup AP remains enabled.

## Dashboard details

- **No Presence** uses a muted LED; **Moving Presence** is amber; **Non-Moving Presence** is teal. If the sensor reports both moving and stationary targets simultaneously, the LED is violet.
- LED brightness / scale indicate proximity, using the nearest active moving or stationary distance (closer is brighter). The distance-to-intensity scale assumes the sensor's nominal 0–600 cm reporting range.
- Numeric readings include moving and stationary distances and energy values, detection distance, and the raw target-state code. The information grid shows OUT level, report age, frame/byte counts, Wi-Fi IP, UART configuration, and pin assignments.
- The transaction table shows recent complete UART frames (including non-target/configuration frames) as timestamped hex, with payload length and footer validity. It keeps up to 40 transactions in the ESP32 ring buffer and the browser displays the newest 40.
- The standard target report format is decoded. Other valid UART frame types are retained in the raw transaction stream but are not interpreted.

No external Arduino libraries are required; the project uses the ESP32 Arduino core's `WiFi` and `WebServer` libraries.
