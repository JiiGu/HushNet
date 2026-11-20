# HushNet

**ESP32-based noise monitor with real-time signal processing, smart alarm logic, and ESP-NOW wireless sync between devices.**

HushNet measures sound levels in real time, shows them on a TFT screen and LED bar, and raises an alarm only when noise stays too loud for too long. Every unit broadcasts its readings over ESP-NOW, so each screen shows both its own level and a remote device's level, with no Wi-Fi router needed.

<!-- Add a photo or GIF of the device running -->
<!-- ![HushNet running](images/hushnet.jpg) -->

---

## Features

- **Real-time sound measurement:** 60 rapid samples per reading from a MAX4466 microphone, converted to peak-to-peak amplitude
- **Baseline calibration:** a 3-second quiet-room measurement at startup adapts the scale to the environment
- **Stable readings:** envelope detection, dual smoothing (fast rise, slow fall), and moving-average filtering
- **Smart alarm logic:** hysteresis with time thresholds, so short spikes never trigger it
- **Self-noise suppression:** the microphone is ignored while the buzzer beeps and for 200 ms after, and briefly after each radio transmission
- **Dual-panel display:** LOCAL and REMOTE panels on a 320 × 240 ILI9341 TFT, each with a percentage, a green-to-red gradient bar, a peak-hold marker, and a status label
- **Flicker-free UI:** only changed regions are redrawn, at about 25 frames per second
- **9-LED level bar:** PWM-dimmed LEDs for a smooth, gradual fill
- **ESP-NOW wireless link:** broadcasts noise level, peak, and alarm state every 500 ms; shows a remote alarm indicator and a "(no link)" state after 5 seconds without data

---

## Hardware (per unit)

| Component | Qty | Notes |
| --- | --- | --- |
| ESP32 development board | 1 | Use two or more units for remote monitoring |
| MAX4466 microphone module | 1 | Sound input |
| ILI9341 TFT display (320 × 240, SPI) | 1 | Live display |
| Active buzzer | 1 | Alarm output |
| LEDs | 9 | Level bar |
| Resistors | 9 | Current-limiting resistors for the LEDs |
| Breadboard and jumper wires | — | — |

---

## Wiring

| Component | Pin | ESP32 GPIO |
| --- | --- | --- |
| MAX4466 | OUT | 34 (ADC1, works alongside Wi-Fi) |
| Active buzzer | + | 25 |
| ILI9341 TFT | CS | 5 |
| ILI9341 TFT | DC | 2 |
| ILI9341 TFT | RST | 4 |
| ILI9341 TFT | SCK / MOSI / MISO | 18 / 23 / 19 (default hardware SPI) |
| LEDs 1–9 | Anode (via resistor) | 13, 14, 16, 17, 21, 22, 26, 27, 32 |

Power the MAX4466 and TFT from 3.3 V, and connect all grounds together.

---

## How It Works

1. **Calibrate.** On startup, the device measures the room for 3 seconds and stores the average amplitude as its baseline.
2. **Sample.** Each loop reads 60 microphone samples and takes the peak-to-peak amplitude.
3. **Smooth.** A moving average and an envelope detector stabilize the signal. The level then rises quickly but falls slowly, so it reacts fast without jittering.
4. **Scale.** The amplitude is mapped from the baseline to a 0–100% noise level: QUIET below 25%, MODERATE below 60%, LOUD above.
5. **Track.** A slow long-term average separates sustained noise from short spikes.
6. **Alarm.** The alarm turns on when the long-term level stays above 75% for 2 seconds, and off when it stays below 55% for 2 seconds. The buzzer beeps (200 ms on, 800 ms off) instead of sounding continuously, and a warning popup appears on screen.
7. **Share.** Every 500 ms the device broadcasts its level, peak, and alarm state over ESP-NOW. Other units show it in their REMOTE panel.

---

## Getting Started

### Requirements

- [Arduino IDE](https://www.arduino.cc/en/software)
- ESP32 board support, version 3.x (Boards Manager → **esp32** by Espressif Systems)
- Libraries (Library Manager): **Adafruit GFX Library**, **Adafruit ILI9341**

### Upload

1. Clone this repository or download it as a ZIP.
2. Open `HushNet/HushNet.ino` in the Arduino IDE.
3. Give each unit a different `DEVICE_ID` near the top of the sketch.
4. Select your ESP32 board and port under **Tools**, then click **Upload**.
5. Keep the room quiet for 3 seconds while it calibrates.

No MAC addresses need to be entered: units find each other through ESP-NOW broadcast. Read the note on broadcast below before using HushNet near other ESP32 projects.

### Tuning

| Setting | Default | What it changes |
| --- | --- | --- |
| `SENSITIVITY_RANGE` | 1500 | Amplitude span mapped to 0–100% (lower = more sensitive) |
| `ALARM_ON_LEVEL` / `ALARM_OFF_LEVEL` | 75 / 55 | Alarm thresholds in % |
| `ALARM_ON_TIME` / `ALARM_OFF_TIME` | 2000 / 2000 ms | How long noise must stay past a threshold |
| `SEND_INTERVAL_MS` | 500 ms | How often readings are broadcast |

---

## Known Limitations

> [!WARNING]
> **HushNet uses ESP-NOW broadcast.** Every unit sends its readings to all ESP-NOW devices in range on the same Wi-Fi channel, not only to other HushNet units.

- **Other ESP32 projects nearby may receive HushNet's packets.** If another ESP-NOW device in the area is listening for broadcast messages, it will receive HushNet's packets every 500 ms and may misread them as its own data.
- **HushNet may misread other devices' packets.** Incoming messages are only checked by size, so any other ESP-NOW packet of the same length would be shown as a remote reading.
- **Recommended for shared spaces** (labs, classrooms, demos with other ESP32 projects):
  - Switch to **unicast**: replace `broadcastAddress` with each partner unit's MAC address (printed to Serial Monitor at startup as `Local MAC`), and add one peer per unit.
  - Or add a **group check**: include a fixed identifier in `esp_msg_t` and ignore any message that doesn't carry it.

---

## Project Info

Course project, FEU Institute of Technology, 2026.
Embedded systems development: [Juan Diego Dela Paz](https://www.linkedin.com/in/jd-dela-paz/)
<!-- Add groupmates here if they agree -->
