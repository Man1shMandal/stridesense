# StrideSense

ESP32 firmware that turns a plain treadmill into a smart one: tap an RFID card to start, and it counts steps and shows speed, distance and incline on your phone.

The ESP32 runs its own Wi-Fi access point with a captive portal, so any phone that joins the network gets the dashboard without typing an address. An MPU6050 mounted on the treadmill picks up footfall vibration for step counting and its tilt gives the incline angle.

## What it does

- RFID tap (MFRC522) starts a 3-2-1 countdown, then a session.
- Step detection from accelerometer vibration peaks, with a 250 ms refractory window.
- Cadence (steps/min), speed (km/h) and distance (km) derived from a fixed 0.70 m step length. Speed and cadence decay to zero when steps stop.
- Incline from MPU6050 pitch, zeroed at boot. Keep the treadmill flat and still while it calibrates.
- Pause, resume and stop from the web page. After stop it shows the summary for a few seconds, then goes back to ready.
- Buzzer tones for boot, card tap, countdown, start, pause, stop and errors.
- Status LED for boot, running session and fault (fast blink if the MPU, RFID reader or access point failed to start).
- Web dashboard with live stats, session timer, an activity graph and a small walking animation that only moves while steps are being detected.

## Hardware

- ESP32 dev board
- MPU6050 (I2C, address 0x68)
- MFRC522 RFID reader + cards/tags
- Passive buzzer
- LED (or the onboard LED on GPIO 2)

| Part | Signal | ESP32 GPIO |
| --- | --- | --- |
| MPU6050 | SDA | 21 |
| MPU6050 | SCL | 22 |
| MFRC522 | SS (SDA) | 5 |
| MFRC522 | RST | 4 |
| MFRC522 | SCK | 18 |
| MFRC522 | MISO | 19 |
| MFRC522 | MOSI | 23 |
| Buzzer | + | 15 |
| Status LED | + | 2 |

Power the MPU6050 and MFRC522 from 3.3 V.

## Stack

- Arduino C++ on the ESP32 core (`WiFi`, `WebServer`, `DNSServer`, `Wire`, `SPI`)
- MFRC522 library
- Dashboard is HTML/CSS/JS embedded in the sketch, polling `/data` for JSON

## Flash

The sketch is `AlwarTredmill/AlwarTredmill.ino`.

Arduino IDE: install the "esp32 by Espressif Systems" board package and the "MFRC522" library, open the sketch, select your ESP32 board and port, and upload.

arduino-cli:

```bash
arduino-cli core install esp32:esp32
arduino-cli lib install MFRC522
arduino-cli compile --fqbn esp32:esp32:esp32 AlwarTredmill
arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32 AlwarTredmill
```

Serial Monitor runs at 115200 baud and prints sensor checks, calibration and card UIDs.

## Use

1. Power the board with the treadmill at zero incline and not moving.
2. Join the access point from a phone. The SSID and password are set at the top of the sketch (`AP_SSID`, `AP_PASSWORD`); change the password before using it anywhere shared.
3. The dashboard should open by itself. If not, go to http://192.168.4.1.
4. Tap a card on the reader to start.

## Endpoints

- `GET /` - dashboard
- `GET /data` - live JSON (state, speed, distance, steps, cadence, inclination, session time, sensor status)
- `POST /pause` - pause or resume
- `POST /stop` - end the session
