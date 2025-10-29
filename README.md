# Automated Elevator

An automated two-floor elevator built with an ESP32 microcontroller, ultrasonic sensors, current sensing (ACS712), servo-controlled doors, and a simple web API for remote monitoring and control. Designed and implemented in collaboration with my groupmate, Rafi. I spent a full week designing, building, and debugging both the hardware and firmware.

---

## Project Overview

This project implements a compact, safety-conscious automated elevator tailored for a two-floored small building or demonstration rig. When a person stands in front of the elevator door, the system automatically triggers a ride sequence: the elevator moves to the calling floor, opens the door, allows boarding, and then takes the passenger to the opposite floor. The system includes features for safety (object detection at the door), overload protection (current monitoring), and a simple remote monitoring/control interface via a local web API.

Key goals:

- Autonomous two-floor operation triggered by presence detection.
- Door obstacle detection to prevent accidental closing on objects/people.
- Overcurrent detection to stop the motor on overload.
- Remote status reporting (`/status`) and remote control (`/control`) endpoints.

---

## Features

- **Automatic presence detection** using two ultrasonic sensors (one per floor approach).
- **Door control** using an MG995 servo motor to open/close the door.
- **Object detection at the door** via an IR sensor; the door will not close if an obstacle is present.
- **Overcurrent protection** using an ACS712 current sensor (configured for up to 30 A), which stops the motor and signals an alarm if the current exceeds the safety limit.
- **Visual and audible alerts**: Red/Green LEDs and two buzzers for notifications and warnings.
- **Smooth motor control** with analog PWM speed control for gentle acceleration/deceleration.
- **Local Web API** to query status and request moving between floors.
- **Debug-friendly**: serial logs for every critical action (sensor readings, motor events, door status, API calls).

---

## Hardware Components

- **ESP32** development board
- **MG995** servo motor (door actuator)
- **DC motor + H-bridge driver** (motor pins: IN1, IN2, ENA in firmware)
- **ACS712 30A** current sensor module (analog)
- **Ultrasonic sensors** (HC-SR04 or compatible) ×2 (TRIG1/ECHO1 and TRIG2/ECHO2)
- **IR obstacle sensor** for door safety
- **LEDs** (Green and Red)
- **Buzzers** ×2
- **Push switch** (optional for manual control)
- Power supply capable of driving the motor and MG995 servo (make sure the servo and motor supply can provide sufficient current; MG995 can draw considerable current under load)

> **Note on the servo:** The code in the repository attaches the servo with pulse widths configured for typical hobby servos. An MG995 is more powerful than an SG90; ensure you provide a stable 5–6V supply and that the servo pulse-width range is compatible (MG995 often uses the same 500–2500 µs range, but check your servo datasheet). Avoid powering the MG995 from the ESP32 5V line if it cannot source the required current.

---

## Wiring Summary (matching firmware pin definitions)

- `IN1` → Motor driver input A (GPIO 4)
- `IN2` → Motor driver input B (GPIO 5)
- `ENA` → Motor driver PWM enable (GPIO 19)
- `IR_SENSOR` → IR obstacle input (GPIO 14)
- `CURRENT_SENSOR` → ACS712 analog output (ADC pin 27)
- `SERVO_PIN` → Servo signal (GPIO 25)
- `TRIG1`/`ECHO1` → Ultrasonic sensor 1 (GPIO 26 / GPIO 16)
- `TRIG2`/`ECHO2` → Ultrasonic sensor 2 (GPIO 32 / GPIO 33)
- `LED_GREEN` → Green LED (GPIO 2)
- `LED_RED` → Red LED (GPIO 15)
- `BUZZER1` → Buzzer 1 (GPIO 22)
- `BUZZER2` → Buzzer 2 (GPIO 23)
- `SWITCH_PIN` → Optional manual switch (GPIO 21)

---

## Firmware & Libraries

The firmware is written for the ESP32 using the Arduino framework. Required libraries (included by `#include` in the sketch):

- `WiFi.h` (ESP32 core)
- `WebServer.h` (ESP32 WebServer)
- `ArduinoJson.h` (for building JSON responses)
- `ESP32Servo.h` (servo control)
- `ACS712.h` (helper for current sensor — optional; code uses raw ADC conversion)

**Important build notes:**

- Configure your Arduino IDE / PlatformIO for the correct ESP32 board variant.
- Install `ArduinoJson` and `ESP32Servo` libraries via Library Manager.

---

## API Endpoints

The firmware exposes a minimal local web API (uncomment the WiFi/server code in `setup()` to enable):

### `GET /status`

Returns a JSON object with cached sensor values and state.

Example response:

```json
{
	"currentFloor": 1,
	"irSensor": 0,
	"currentA": 0.123,
	"distance1_cm": 15.3,
	"distance2_cm": 12.0,
	"doorStatus": "closed"
}
```

### `GET /control?floor=<1|2>`

Requests the elevator to move to floor `1` or `2`. The endpoint replies immediately with a confirmation message while the ESP32 executes the movement sequence.

Example: `/control?floor=2`

Response:

```json
{ "status": "ok", "message": "Moved to floor 2" }
```

---

## How It Works — Operation Flow

1. **Idle**: the elevator rests at a default floor (code toggles `currentFloor`). The system continuously polls ultrasonic sensors and updates cached readings.
2. **Presence Detection**: When an ultrasonic sensor detects an object within the configured threshold (e.g., `d < 10 cm`), the system treats that as a call from that floor.
3. **Movement**: depending on the current and requested floors, the motor is driven with PWM-controlled acceleration/deceleration (`pullUp()`/`pullDown()` routines).
4. **Door Handling**: When the lift reaches the floor, the door is opened with the MG995. Before closing, the `safeDoorClose()` routine checks the IR sensor — if an obstacle is present, the system will warn (buzzers + LEDs) and wait until clear.
5. **Overcurrent Safety**: the `readCurrent()` routine samples the ACS712. If measured current exceeds `currentLimit` (default `30 A`), movement stops immediately and alarms trigger.
6. **Remote Control**: The web API allows an external client to query the state and request moves. The firmware frequently calls `server.handleClient()` within long loops so the API remains responsive.

---

## Calibration & Tuning

- **ACS712 sensitivity and zero offset**: the code assumes a 30A ACS712 with sensitivity ~`0.066 V/A` and a mid-point (`zeroOffset`) at ~`1.65 V`. Calibrate by measuring the unloaded ADC voltage and adjusting `zeroOffset`.
- **Ultrasonic thresholds**: adjust the `d1 < 10` and `d2 < 10` thresholds to match your installation distance and field-of-view.
- **Servo angles and timing**: MG995 may need a larger torque but a similar angle range. Verify `SERVO_OPEN_ANGLE`/`SERVO_CLOSED_ANGLE` values and allow sufficient `delay()` time for the servo to move.
- **Motor PWM values**: tune acceleration and PWM ranges inside `pullUp()` and `pullDown()` to match your motor and load characteristics.

---

## Troubleshooting

- **Servo jitter or resets**: ensure the servo has a stable power supply (separate from ESP32 5V if necessary) and common ground.
- **Ultrasonic reads `0` or `nan`**: check wiring for TRIG/ECHO and add proper pull-ups if using long wires.
- **Inaccurate ACS712 readings**: ensure proper grounding, measure ADC reference, and calibrate `zeroOffset`.
- **WiFi not connecting**: uncomment the WiFi block in `setup()` and set your SSID/password. Use serial prints to debug.

---

## Safety Notes

- The system is designed for a demonstration rig. For any human-carrying lift or real installation, follow local building codes and safety regulations.
- MG995 and the motor draw significant current — provide proper motor drivers, fuses, and an appropriate power supply.
- Use limit switches for final production use to avoid positional drift. Ultrasonic sensors are good for presence detection, but not reliable as a sole final position sensor.

---

## Future Improvements

- Add limit switches or encoders for precise floor detection.
- Replace `delay()`-heavy loops with non-blocking state machines for smoother multitasking.
- Add HTTPS + authentication for the web API.
- Add a web UI with live charts for current and distance readings.
- Add a persisted log (SD card or remote server) for fault diagnosis.

---

## Example Usage

1. Modify WiFi credentials in the sketch and uncomment the WiFi/server block in `setup()`.
2. Upload firmware to ESP32.
3. Open serial monitor (115200 baud) to watch logs and sensor readings.
4. Call `http://<ESP32_IP>/status` to view status or `http://<ESP32_IP>/control?floor=2` to move.

---

## Credits

- Kazi Rifat Al Muin [Hardware Design, Logic & Implementation]
- Arman Rahman Rafi [Motor Functionality & Website Integration]

---

## License

This project is released under the MIT License. Feel free to reuse, modify, and redistribute with attribution.

---

If you want, I can also:

- Generate a compact wiring diagram diagram (text-based or SVG),
- Create a short web UI that talks to the `/status` and `/control` endpoints, or
- Turn this README into a repository `README.md` file and suggest a `.gitignore` and `LICENSE` file.
