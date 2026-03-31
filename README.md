# Wire-Following Robot with IR Remote Control and Mission Logging

An embedded robotics project built on the **EFM8LB1** microcontroller that supports both **autonomous wire-following** and **manual IR remote control**. The robot uses analog inductive sensors to track a wire, a **VL53L0X time-of-flight sensor** for obstacle detection, and a custom **H-bridge motor driver** for differential drive. It also includes **mission logging over UART**, allowing run data to be exported in CSV format for analysis.

## Features

- **Autonomous wire-following** using three analog inductive sensors
- **Manual drive mode** using an IR remote
- **Obstacle detection and stop logic** using the VL53L0X distance sensor
- **Three selectable autonomous paths** with programmed intersection decisions
- **PWM motor control** using a Timer 2 interrupt at 10 kHz
- **Mission logging** stored in XDATA and dumped over UART as CSV
- **Differential drive turning and spinning** for path navigation
- **Flash-based variable restore support**

## System Overview

The robot operates in two main modes:

### 1. Manual Mode
In manual mode, the robot receives IR packets and directly sets the left and right motor commands. This allows the user to drive the robot remotely.

### 2. Autonomous Mode
In autonomous mode, the robot:
- follows a wire using left and right inductive sensors,
- detects intersections using the center sensor,
- executes a predefined path sequence,
- stops if an obstacle is detected within the collision threshold,
- resumes once the obstacle is removed,
- logs mission data during operation.

## Hardware Used

- **Microcontroller:** EFM8LB1
- **Distance Sensor:** VL53L0X
- **Motor Driver:** Custom H-bridge
- **IR Receiver:** TSOP33338
- **Drive System:** Two DC motors
- **Sensors:** Three inductive wire-following sensors
- **Communication:** UART for CSV data dump

## Pin Configuration

### H-Bridge Motor Outputs
- `P1.2` → Left motor forward
- `P1.3` → Left motor backward
- `P1.0` → Right motor forward
- `P1.1` → Right motor backward

### Inductive Sensors
- `P2.1` → Left sensor
- `P2.2` → Center sensor
- `P2.3` → Right sensor

### IR Receiver
- `P0.2` → TSOP33338 output (active low)

### LEDs
- `P0.5` → Left indicator LED
- `P0.6` → Right indicator LED
- `P0.7` → Brake/status LED

## Timer Usage

- **Timer 0** → SMBus / I2C clock for VL53L0X
- **Timer 1** → UART baud rate generation
- **Timer 2** → 10 kHz PWM interrupt for motor control
- **Timer 3** → microsecond delay utility
- **Timer 4** → free-running counter for IR pulse timing

## Autonomous Path Options

The robot supports three predefined path sequences:

- **Path 1:** `f, l, l, f, r, l, r, s`
- **Path 2:** `l, r, l, r, f, f, s`
- **Path 3:** `r, f, r, l, r, l, f, s`

Where:
- `f` = move forward
- `l` = spin left
- `r` = spin right
- `s` = stop and dump mission log

These paths are triggered at intersections detected by the center inductive sensor.

## Wire-Following Logic

The robot compares the left and right inductive sensor voltages:
- If the difference is within a dead band, it drives forward
- If the right sensor is stronger, it turns left
- If the left sensor is stronger, it turns right

This provides simple closed-loop tracking of the buried or guided wire.

## Obstacle Detection

The VL53L0X sensor continuously measures distance ahead of the robot.

- If an object is detected closer than **150 mm**, the robot stops
- The robot remains stopped until the obstacle is removed
- Obstacle events are logged in memory

## IR Control Protocol

The IR receiver decodes packets by measuring pulse widths using Timer 4.  
Supported packet types include:

- **Mode switching**
- **Manual motor drive commands**

The checksum is validated before commands are accepted.

### Supported Mode Selection
- **Manual mode**
- **Auto path 1**
- **Auto path 2**
- **Auto path 3**

## Mission Logging

During autonomous operation, the robot stores mission data in XDATA memory.  
Each log entry contains:

- `sample_id`
- `dist_mm`
- `left_pwm`
- `right_pwm`
- `path_idx`
- `event`

### Event Codes
- `0` → Normal follow-wire sample
- `1` → Intersection detected
- `2` → Obstacle detected
- `3` → End of path

When the path ends, the robot stops and waits for a UART trigger from the PC. It then sends the mission log in **CSV format**.

### CSV Output Format
```csv
sample_id,dist_mm,left_pwm,right_pwm,path_idx,event
...
END
