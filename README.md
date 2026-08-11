# Adaptive Modular Hexapod Robot for LiDAR Terrain Mapping

This is my hexapod robot project for LiDAR-based terrain mapping. The main idea behind the robot is not only to walk on rough terrain, but to survive damage while doing it.

Most small walking robots assume all legs are always attached and working. I wanted to design something different: a modular hexapod where each leg can be removed, replaced, detected by the controller, and handled in software. If one, two, or even three legs are damaged, the robot should still try to balance itself and generate a new gait pattern using the remaining legs.

<p align="center">
  <img src="Robot%20files/Robot%20redered%20img.jpeg" alt="Rendered adaptive modular hexapod robot" width="760">
</p>

This repository contains the ESP32 firmware for that idea. The mechanical side was designed separately as a full 3D model in Fusion 360, including custom leg joints and custom removable connectors.

## The Original Goal

The task was to design a hexapod robot that can be used for LiDAR-based terrain mapping. For that kind of robot, stability and reliability matter a lot. If the robot is moving through unknown terrain, a leg can get damaged, stuck, or mechanically disconnected. Instead of treating that as a complete failure, I wanted the robot to understand its own body condition and continue with whatever legs are still available.

So the project became two connected problems:

- Mechanical design: create removable legs, custom joints, and connectors that can be replaced easily.
- Firmware design: detect connected legs and adapt the motion logic depending on the available legs.

## My Design Thinking

I started by thinking about the robot as a modular system instead of a fixed six-leg machine. Each leg had to be a separate unit. That meant the joint design, connector design, wiring, and software all had to support removability.

The mechanical design was done in Fusion 360. I designed custom joints for the leg mechanism and custom connectors so each leg could be attached or removed from the body. This helped make the robot easier to repair and also made the adaptive gait idea possible, because the robot could physically operate with different leg combinations.

### Full 3D Mechanical Model

<p align="center">
  <img src="Robot%20files/robot%203D%20model%20ss%20fusion.jpeg" alt="Hexapod 3D model in Fusion 360" width="760">
</p>

### Custom Joint and Modular Leg Assembly

<p align="center">
  <img src="Robot%20files/Designed%20joint.jpeg" alt="Custom designed hexapod leg joint" width="46%">
  <img src="Robot%20files/assembled%20robot%20leg.jpeg" alt="Assembled modular robot leg" width="46%">
</p>

On the electronics and firmware side, I first focused on the basics: mapping every servo motor, finding the initial position of each joint, testing safe lift positions, and making sure no servo was moved accidentally. After that, I added leg availability detection, serial/Bluetooth control, and adaptive walking logic.

## What We Built

- A modular hexapod robot concept for LiDAR terrain mapping.
- Custom-designed leg joints.
- Custom-designed removable leg connectors.
- Full 3D mechanical model in Fusion 360.
- ESP32 firmware using PlatformIO and Arduino framework.
- Two PCA9685 PWM servo driver boards for up to 18 servos.
- Analog leg detection for all six removable legs.
- USB Serial and Bluetooth Serial command control.
- Safe homing that skips disconnected legs.
- Manual leg lift command for testing each leg.
- Forward walking logic that changes based on connected legs.
- Left and right rotation commands using available legs.

### Physical Robot Base

<p align="center">
  <img src="Robot%20files/robot%20base.jpeg" alt="Hexapod robot base assembly" width="46%">
  <img src="Robot%20files/robot%20base1.jpeg" alt="Hexapod robot base from another view" width="46%">
</p>

## Problems Encountered And How We Solved Them

### 1. Servo Mapping Was Confusing

At the beginning, the biggest firmware problem was knowing which servo belonged to which leg and joint. A hexapod has 18 possible motors, so it is easy to mix up hip, femur, and tibia channels.

The solution was to create a clear mapping rule: each leg uses three channels, grouped in blocks of four on the PCA9685. Legs 1 to 3 are on the first PCA9685 board, and legs 4 to 6 are on the second board. This made the code easier to understand and made testing much safer.

### 2. The Robot Needed A Safe Initial Position

Before walking, the robot needed a known starting posture. Without that, every test could begin from a different physical angle, which is risky for the servos and the body frame.

The solution was to define one home position for every leg:

| Joint | Home Angle |
|---|---:|
| Hip | `90` |
| Femur | `153` |
| Tibia | `160` |

The `h` command returns all connected legs to this initial position.

### 3. Removable Legs Should Not Receive Servo Commands

Because the legs are removable, the firmware cannot blindly command all six legs. If a leg is disconnected, the robot should know that and skip it.

The solution was to add one detect line for each leg. The ESP32 reads these lines as analog voltages and compares them with a threshold. If the voltage is high enough, that leg is marked as available. If it is below the threshold, the leg is treated as disconnected.

Current threshold:

```cpp
#define LEG_DETECT_THRESHOLD_MV 2500
```

### 4. Digital HIGH Was Not Reliable For Detection

At first, leg detection was thought of as a simple HIGH/LOW digital signal. But the detected "high" voltage was not always a full ESP32 logic HIGH. It could be around a lower analog voltage depending on the circuit.

The solution was to use `analogReadMilliVolts()` instead of `digitalRead()`. This made the detection more realistic because the firmware now prints the actual measured voltage and decides availability using a millivolt threshold.

Example output:

```text
Leg 1 detect pin D34: 2600 mV -> AVAILABLE
Leg 2 detect pin D35: 120 mV -> NOT CONNECTED
```

### 5. Testing One Leg At A Time Was Needed

Moving the whole robot immediately is risky. I needed a way to test one leg at a time and check whether the lift angles were correct.

The solution was to add simple serial commands. Pressing `1`, `2`, `3`, `4`, `5`, or `6` prints that leg's availability and lifts only that leg if it is connected.

Manual lift pose:

| Joint | Lift Angle |
|---|---:|
| Femur | `160` |
| Tibia | `130` |

### 6. Motor Calibration Took Careful Step-By-Step Testing

Another major problem was calibration. Even when the servo mapping was correct, the physical motor direction and horn position could still make a leg move the wrong way. A femur angle that lifts one leg can press another leg downward if the servo is mounted in the opposite direction.

The solution was to calibrate the motors slowly and one joint at a time. First, I set all legs to the home position. Then I tested each leg individually using the serial commands. For each motor, I checked whether increasing or decreasing the angle produced the expected movement.

The calibration process was:

1. Start from the home position using `h`.
2. Test one leg by entering its number, for example `1`.
3. Watch the hip, femur, and tibia movement carefully.
4. Adjust the target angles in small steps.
5. Re-test until the leg lifts without forcing the servo.
6. Use the same method for the remaining legs.

The important calibrated positions in the current firmware are:

| Purpose | Hip | Femur | Tibia |
|---|---:|---:|---:|
| Home / initial stance | `90` | `153` | `160` |
| Manual lift test | unchanged | `160` | `130` |
| Walking lift | unchanged | `165` | `125` |

The PCA9685 pulse range is also part of calibration:

| Constant | Value |
|---|---:|
| `SERVOMIN` | `150` |
| `SERVOMAX` | `600` |
| `SERVO_FREQ` | `50 Hz` |

These values convert servo angles into PWM pulses. If a servo strains, hits a mechanical limit, or cannot reach the expected angle, these values and the joint angle targets must be tuned carefully.

### 7. Different Missing-Leg Cases Need Different Gaits

A normal tripod gait works when all six legs are present, but it is not enough for a modular robot. If one leg is missing, the gait should change. If two or three legs are available, the robot needs a more limited recovery motion.

The solution was to build adaptive walking logic. Before walking, the robot checks which legs are available, counts them, and chooses the best supported gait pattern.

Current behavior:

- 6 legs connected: tripod-style gait.
- 5 legs connected: recovery gait based on which leg is missing.
- 4 legs connected: recovery walking with selected support pairs.
- 3 neighboring legs connected: three-leg recovery gait.
- 2 legs connected: pair-based movement.
- 1 leg connected: femur/tibia push motion only.
- Unsupported combinations: walking is cancelled safely.

### 8. The Robot Needed Easier Control

USB Serial is useful while programming, but for a mobile robot it is better to control it without always connecting a cable.

The solution was to add Bluetooth Serial. The robot now appears as:

```text
Hexapod-Control
```

Commands can be sent either through the PlatformIO serial monitor or through a Bluetooth terminal.

## Hardware And Firmware Overview

Main controller:

- ESP32 Dev Module, 30-pin style

Servo control:

- PCA9685 board 1 at `0x40`
- PCA9685 board 2 at `0x60`

Software:

- PlatformIO
- Arduino framework
- Adafruit PWM Servo Driver library
- BluetoothSerial

### Robot Electronics / PCB

<p align="center">
  <img src="Robot%20files/robot%20PCB.jpeg" alt="Hexapod robot electronics and PCB" width="720">
</p>

## Servo Mapping

| Leg | PCA9685 Address | Hip | Femur | Tibia |
|---:|---:|---:|---:|---:|
| 1 | `0x40` | CH0 | CH1 | CH2 |
| 2 | `0x40` | CH4 | CH5 | CH6 |
| 3 | `0x40` | CH8 | CH9 | CH10 |
| 4 | `0x60` | CH0 | CH1 | CH2 |
| 5 | `0x60` | CH4 | CH5 | CH6 |
| 6 | `0x60` | CH8 | CH9 | CH10 |

Each leg has:

| Joint Number | Joint Name |
|---:|---|
| `1` | Hip |
| `2` | Femur |
| `3` | Tibia |

## Leg Detection Pins

| Leg | ESP32 GPIO |
|---:|---:|
| 1 | GPIO 34 |
| 2 | GPIO 35 |
| 3 | GPIO 32 |
| 4 | GPIO 33 |
| 5 | GPIO 25 |
| 6 | GPIO 26 |

GPIO 34 and GPIO 35 are input-only pins, which is fine because they are only used for detection.

## Commands

| Command | Action |
|---|---|
| `1` to `6` | Print that leg's availability and lift it if connected |
| `f` | Walk forward for 1 cycle |
| `f3` | Walk forward for 3 cycles |
| `r` | Rotate right for 1 cycle |
| `r3` | Rotate right for 3 cycles |
| `l` | Rotate left for 1 cycle |
| `l3` | Rotate left for 3 cycles |
| `h` | Return all connected legs to initial position |
| `?` | Print command help |

## Results So Far

The robot firmware can now detect connected and disconnected legs, print the availability status, skip removed legs, lift individual legs for testing, return to the home position, and select a walking strategy based on the remaining legs.

### Robot in Motion

<p align="center">
  <img src="Robot%20files/Robot%20video.gif" alt="Adaptive modular hexapod robot motion demonstration" width="760">
</p>

The latest firmware build was successful:

```text
pio run
SUCCESS
```

Latest observed build usage:

| Resource | Usage |
|---|---:|
| RAM | `12.3%` |
| Flash | `86.5%` |

## Project Structure

```text
.
+-- platformio.ini
+-- src/
|   +-- main.cpp
+-- include/
+-- lib/
+-- test/
```

## Build And Upload

Build:

```bash
pio run
```

Upload:

```bash
pio run --target upload
```

Open serial monitor:

```bash
pio device monitor
```

## Final Thoughts

This project became more than just a walking robot. The interesting part was making the robot aware of its own physical condition. The removable leg design, custom joints, custom connectors, and adaptive firmware all support the same idea: a robot should not fail completely just because one part is damaged.

There is still more tuning to do, especially in the gait angles and real-world balance testing, but the foundation is working. The robot can identify its connected legs, avoid unsafe commands, and start adapting its movement to the body it currently has.
