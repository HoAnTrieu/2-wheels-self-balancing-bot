# 2-Wheels Self-Balancing Robot

A self-balancing robot project using ESP32, MPU6050 gyroscope/accelerometer, L298N motor driver, and PID control algorithm. The robot balances on two wheels like an inverted pendulum.

## Project Overview

This project implements a two-wheeled self-balancing robot that maintains upright position through PID control. It uses sensor fusion from the MPU6050 to estimate tilt angle and drives two DC motors to correct balance.

The project includes:
- Arduino ESP32 firmware for balancing control
- MATLAB scripts for system modeling and parameter calculation
- 3D printable chassis designs
- Required libraries

## Hardware Requirements

- ESP32 development board (e.g., ESP32 DevKit V1)
- MPU6050 gyroscope/accelerometer module
- L298N motor driver module
- 2x DC motors with wheels (3-6V, 200RPM recommended)
- 2x 18650 batteries or suitable power source (7.4V)
- Battery holder and power switch
- Jumper wires for connections
- 3D printed chassis (STL files included from Thingiverse)

## Software Requirements

- Arduino IDE with ESP32 support
- MATLAB (optional, for system analysis)
- Adafruit MPU6050 library
- Adafruit BusIO library
- Adafruit Unified Sensor library

## Wiring Diagram

### ESP32 to MPU6050 (I2C)
- ESP32 GPIO 21 (SDA) → MPU6050 SDA
- ESP32 GPIO 22 (SCL) → MPU6050 SCL
- ESP32 3.3V → MPU6050 VCC
- ESP32 GND → MPU6050 GND

### ESP32 to L298N Motor Driver
- ESP32 GPIO 25 → L298N ENA (Motor A PWM)
- ESP32 GPIO 26 → L298N IN1 (Motor A direction)
- ESP32 GPIO 27 → L298N IN2 (Motor A direction)
- ESP32 GPIO 33 → L298N ENB (Motor B PWM)
- ESP32 GPIO 16 → L298N IN3 (Motor B direction)
- ESP32 GPIO 17 → L298N IN4 (Motor B direction)
- ESP32 5V → L298N VCC (logic power)
- External 7.4V → L298N VS (motor power)
- ESP32 GND → L298N GND

### Motor Connections
- Motor A → L298N OUT1/OUT2
- Motor B → L298N OUT3/OUT4

## Installation and Setup

1. **Install Arduino IDE and ESP32 Support**
   - Download Arduino IDE from arduino.cc
   - Add ESP32 board support via Board Manager (search for "ESP32")

2. **Install Required Libraries**
   - Open Arduino IDE
   - Go to Sketch → Include Library → Manage Libraries
   - Install "Adafruit MPU6050", "Adafruit BusIO", "Adafruit Unified Sensor"

3. **Prepare the Hardware**
   - Assemble the 3D printed chassis
   - Mount motors, wheels, and electronics
   - Wire according to the diagram above
   - Ensure MPU6050 is mounted securely and centered

4. **Upload the Code**
   - Open one of the Arduino sketches (e.g., PID-main.ino)
   - Select ESP32 Dev Module from Tools → Board
   - Set upload speed to 115200
   - Select correct COM port
   - Click Upload

5. **Power On**
   - Connect power to L298N (motor power)
   - The robot should start balancing
   - Monitor serial output for debugging

## Code Files Explanation

### PID-main.ino
The main balancing firmware. Features:
- Complementary filter for angle estimation from MPU6050
- PID controller for balance correction
- Motor control with deadband compensation
- Safety shutdown when tilt exceeds 35 degrees
- Serial debugging output

Key parameters to tune:
- Kp, Ki, Kd: PID gains (start with Kp=18, Ki=0.6, Kd=0.8)
- PWM_DEADZONE: Minimum PWM to overcome motor friction (130)
- PWM_SLEW: Maximum PWM change per cycle for smooth control (20)

### PID-balencing-lite.ino
A simplified version with arm/disarm safety feature. Includes:
- Automatic arming when within safe angle
- Fixed PID parameters
- Different complementary filter implementation

### PID-balencing-test.ino
Test firmware for hardware validation. Runs motors through predefined sequences while monitoring sensors. Useful for:
- Testing motor directions
- Calibrating sensor readings
- Debugging hardware connections

### MATLAB/ThongSoTWBR.m
MATLAB script for system modeling. Calculates:
- State space matrices (A, B, C, D)
- Controllability and observability analysis
- Transfer functions
- Pole-zero plots

Parameters include:
- Motor constants (km, ke, R)
- Physical dimensions (wheel radius r, distance l)
- Masses and inertias (Mp, Mw, Ip, Iw)

## How the Code Works

### Sensor Reading and Angle Estimation
The MPU6050 provides accelerometer and gyroscope data. A complementary filter combines:
- Accelerometer: Provides absolute tilt angle (pitch/roll)
- Gyroscope: Provides angular rate for short-term accuracy

The filter equation: `angle = α * accel_angle + (1-α) * (previous_angle + gyro_rate * dt)`

### PID Control
The PID controller calculates motor output based on error (desired angle - current angle):
- Proportional (P): Immediate correction based on current error
- Integral (I): Corrects steady-state error accumulation
- Derivative (D): Dampens oscillations by predicting error change

Output = Kp * error + Ki * ∫error dt + Kd * d(error)/dt

### Motor Control
The L298N driver controls two motors independently:
- Direction pins (IN1-IN4) set rotation direction
- Enable pins (ENA/ENB) set speed via PWM
- Deadband compensation ensures motors start moving at low speeds

### Safety Features
- Automatic shutdown when tilt exceeds safe limits
- PWM limiting and slew rate limiting prevent sudden movements
- Error integral clamping prevents windup

## Tuning the PID Controller

1. **Start with P only**: Set Ki=0, Kd=0, increase Kp until oscillations begin
2. **Add D**: Increase Kd to dampen oscillations
3. **Add I**: Small Ki to eliminate steady-state error
4. **Fine-tune**: Adjust all three parameters gradually
5. **Test robustness**: Push the robot gently and observe recovery

Monitor serial output for angle and PWM values during tuning.

## MATLAB System Analysis

Run `ThongSoTWBR.m` in MATLAB to analyze the system:
- Check controllability (rank(P) should be 4)
- Check observability (rank(L) should be 4)
- Examine pole locations for stability
- Design controllers using root locus plots

## Troubleshooting

### Robot Falls Over Immediately
- Check MPU6050 orientation and mounting
- Verify I2C connections
- Confirm motor directions are correct
- Increase P gain or check angle calculation

### Motors Don't Move
- Check L298N power connections
- Verify PWM and direction pin assignments
- Test with PID-balencing-test.ino

### Unstable Balance
- Tune PID parameters
- Check sensor calibration
- Ensure smooth PWM output (adjust PWM_SLEW)

### Serial Shows "MPU6050 NOT FOUND"
- Check I2C wiring and pull-up resistors
- Verify MPU6050 power (3.3V)
- Test with I2C scanner sketch

## Contributing

This project combines code from various sources. Please cite original authors when using 3D models or code snippets.

## License

See LICENSE files in subdirectories for specific licensing information.