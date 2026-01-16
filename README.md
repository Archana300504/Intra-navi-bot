# Intra-navi-bot
A 4 Wheel Drive that could move at all the ways with the manual mobile through bluetooth communication

# Bluetooth & Joystick Controlled Robot with Encoder-Based Distance Measurement

This project implements a Four-wheel differential drive robot controlled using:
-Bluetooth (HC-05 / HC-06)
-Analog Joystick
The robot supports manual control (forward, reverse, left, right, stop) and can measure the distance travelled using encoder feedback.

# Hardware Components Used
- Arduino Uno
- L298N Motor Driver
- DC Motors with Quadrature Encoders
- Bluetooth Module (HC-05 / HC-06)
- Analog Joystick Module
- Power Supply (Battery)
- Jumper Wires
- Breadboard


# Features
# Robot Motion Control
- Forward
- Reverse
- Left turn
- Right turn
- Stop

# Control Methods
1. Joystick Control
   - X-axis → Forward / Reverse
   - Y-axis → Left / Right

2. Bluetooth Control
   - `f` → Forward
   - `b` → Reverse
   - `l` → Left
   - `r` → Right
   - `s` → Stop

# Encoder-Based Distance Measurement
- Uses quadrature encoder interrupts
- Measures wheel rotation count
- Calculates distance travelled
- Stops motor after reaching a threshold distance (e.g., 100 cm)

# Working Principle
- Joystick or Bluetooth sends movement commands
- Arduino drives motors using L298N
- Encoder interrupts track wheel rotation
- Distance is calculated incrementally
- Robot can stop automatically after a fixed distance

# Serial Monitor Output
- Joystick X and Y values
- Encoder count
- PWM values
- Distance travelled (in cm)

# Future Improvements
- PID-based speed control
- Obstacle avoidance using ultrasonic sensors
- GPS + GSM integration
- ROS interface with Raspberry Pi
- Autonomous navigation

# Author
Archana Devi P M
