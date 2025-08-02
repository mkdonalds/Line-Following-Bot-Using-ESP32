Overview
This line-following robot is an autonomous device designed to follow a black line using infrared (IR) sensors and a PID-based control system. The robot features differential drive steering and real-time path correction capabilities.

Hardware Components
•	Microcontroller: ESP32
•	Sensors: Two IR sensors (left and right) for line detection
•	Motors: Two DC motors with independent control
•	Motor Driver: L298N
•	Power Supply: Three 3.7V batteries

Core Functionality
Sensing Mechanism:
•	Two IR sensors mounted at the front of the robot
•	Each sensor outputs a digital signal (HIGH/LOW)
  	HIGH when detecting white or light-colored surface
  	LOW when detecting black line (or vice versa, depending on sensor type)
Motor Control Strategy:
1.	Straight Line Motion
•	Both sensors on white: Both motors run at full base speed (230/255 PWM)
•	Robot moves straight ahead
2.	Turn Detection and Response
•	Right sensor detects black: Right motor slows down to full stop
•	Left sensor detects black: Left motor slows down to full stop
•	Both sensors on black: Both motors stop

PID Control System
Parameters:
•	Proportional Gain (Kp) = 30.0
•	Integral Gain (Ki) = 2
•	Derivative Gain (Kd) = 6
Function:
•	Continuously monitors sensor input
•	Calculates error from center line
•	Adjusts motor speeds to maintain course

Circuit Diagram:
<img width="820" height="954" alt="image" src="https://github.com/user-attachments/assets/5a47cb18-73fe-4605-b8f6-0eca381efe9e" />

