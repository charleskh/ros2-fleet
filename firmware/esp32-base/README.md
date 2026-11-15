# esp32-base — Devastator motor + IMU node

Firmware for the ESP32-S3 that owns the Devastator's actuators and inertial sensing. Today it is a
bench smoke test for the IMU; it grows into the micro-ROS node (`/cmd_vel` in, `/imu` and
`/wheel_odom` out) in Phase 2 M3.

## Hardware

- **MCU:** Freenove ESP32-S3-WROOM (N8R8), onboard camera removed. Plugs into female sockets on the
  logic perfboard, so the dev board itself is never soldered.
- **IMU:** Adafruit BNO055 (on-chip sensor fusion), I2C at 0x28, on its own chassis pad via a short
  twisted cable.
- **Motor driver:** SparkFun TB6612FNG breakout, socketed on a separate power perfboard so motor
  current never runs under the MCU.
- **Power:** the ESP32 is powered **only** from the Jetson's USB (no buck converter, so no 5V
  back-feed). If the ESP32 loses power, the TB6612 logic does too and the motors cannot run.

## Pin map

| Function | GPIO | Notes |
|---|---|---|
| TB6612 PWMA, AIN1, AIN2, STBY, BIN1, BIN2, PWMB | 4, 5, 6, 7, 15, 16, 17 | adjacent on one header row |
| Encoders A1, B1, A2, B2 | 10, 11, 12, 13 | reserved; encoders not yet confirmed |
| BNO055 SDA, SCL | 41, 42 | adjacent, bottom row |
| Battery sense (100k / 47k divider) | 1 | ADC1; 8.4V full pack reads ~2.7V |

Avoid: 43/44 (UART0, the micro-ROS link), 19/20 (native USB), 0/3/45/46 (strapping), 2 (IO2 LED),
48 (RGB LED), 35–37 (octal PSRAM), 38–40 (SD card).

## Ports

- **Native USB port** (`303a:1001`): flashing and the serial console. Used by this firmware.
- **UART bridge port:** reserved for the micro-ROS transport to hyperion. It stays enumerated
  across MCU resets, which native USB does not.

## Build and flash

```bash
pio run                    # build
pio run -t upload          # flash over native USB
pio device monitor         # serial console, 115200
```

Linux needs serial access: add yourself to `dialout`, or a scoped udev rule such as
`SUBSYSTEM=="tty", ATTRS{idVendor}=="303a", OWNER="<user>", GROUP="dialout", MODE="0660"`.

## Status

- 2026-11-15: BNO055 verified end to end. Fused heading/roll/pitch track motion, magnetometer
  calibrates to 3 with a figure-8, under 0.15° drift at rest over 70s. Calibration offsets are not
  yet persisted (the BNO055 forgets them at power-off); that lands with the micro-ROS node.
