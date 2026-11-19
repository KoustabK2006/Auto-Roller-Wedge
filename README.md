# Auto-Roller Wedge

A connected inclined-plane rig that measures the **coefficient of static friction** automatically. A motorised wedge tilts at 1° per second until a block slips; an ultrasonic sensor detects the slip, and the friction coefficient is computed and shown on a 1.3" OLED.

## How it works

At the angle of repose the block is on the verge of sliding, so:

```
μ = tan(θ)
```

1. The wedge returns to 0° and the OLED prompts you to place the block.
2. After 3 seconds an ultrasonic baseline distance is taken (average of 8 readings).
3. The wedge tilts +1° every second while the HC-SR04 keeps measuring the block distance.
4. When the distance changes by 25 mm or more, slip is declared. The wedge freezes, the red LED lights, the buzzer sounds, and the OLED shows θ and μ.
5. Touch the sensor (or call `/reset`) to run again.

## Features

- Non-blocking state machine
- 1.3" SH1106 OLED showing μ = tan(θ), angle and status
- Built-in 12x8 LED matrix showing the live angle
- Green LED while running, red LED on slip, buzzer cue
- Touch-sensor reset
- WiFi REST API for live data (`/status`, `/reset`) with CORS enabled

## Hardware

| Component | Pin |
|---|---|
| Arduino UNO R4 WiFi | board |
| Green LED | D2 |
| Red LED | D3 |
| Touch sensor (active HIGH) | D8 |
| Servo 1 (90 + angle) | D9 |
| Servo 2 (90 - angle) | D10 |
| HC-SR04 Trig | D11 |
| HC-SR04 Echo | D12 |
| Active buzzer | D13 |
| OLED SDA | A4 |
| OLED SCL | A5 |

The OLED is a 1.3" SH1106, I2C address 0x3C. For a true SSD1306 module, replace `U8G2_SH1106_128X64_NONAME_F_HW_I2C` with `U8G2_SSD1306_128X64_NONAME_F_HW_I2C`.

## Libraries

- WiFiS3, Servo, Arduino_LED_Matrix (bundled with the UNO R4 core)
- ArduinoGraphics (Library Manager)
- U8g2 by olikraus (Library Manager)

## Setup

1. Clone the repo and open `auto_roller_wedge/auto_roller_wedge.ino` in the Arduino IDE.
2. Copy `arduino_secrets.h.example` to `arduino_secrets.h` in the same folder and enter your 2.4 GHz WiFi credentials.
3. Select **Arduino UNO R4 WiFi**, then upload.
4. Open the Serial Monitor at 115200 baud to see the board's IP address.

## REST API

| Method | Endpoint | Description |
|---|---|---|
| GET | `/status` | Angle, distance, baseline, phase, slip state, slip angle |
| GET / POST | `/reset` | Start a new run (returns `busy` if already resetting) |

Example response from `/status`:

```json
{"angle":32,"distance":148,"initialDist":150,"slipped":false,"slipAngle":null,"phase":"RUNNING","maxAngle":60,"slipTolerance":25,"uptimeMs":84213}
```

## Calibration notes

- `slipTolerance` (default 25 mm) must be larger than any distance change caused by the tilt alone and smaller than the block's actual slide. Adjust to your mounting geometry.
- If the block never slips, the wedge stops at `maxAngle` (60°).
- Mount the sensor so it faces the block directly along the slope.

## Project structure

```
auto_roller_wedge/
  auto_roller_wedge.ino
  arduino_secrets.h.example
.gitignore
LICENSE
README.md
```

## License

MIT, see [LICENSE](LICENSE).
