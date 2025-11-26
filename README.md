# 🧩 Labyrinth Solver Robot using Tiva C & Ultrasonic Sensor

This project presents the design and implementation of an autonomous labyrinth-solving robot capable of detecting and avoiding obstacles using an ultrasonic sensor, servo motor, and wheel encoders. The system is developed using the Tiva C LaunchPad (EK-TM4C123GXL) and programmed through the Energia IDE.

## Project Overview

The robot is designed to navigate through a labyrinth(single path with no branch) autonomously.
It continuously moves forward, detects obstacles in its path, scans for alternative directions using a servo-mounted ultrasonic sensor, and chooses the optimal path to proceed.

The system integrates:

- Ultrasonic sensing for distance measurement

- Motor driver for robot movement

- Servo motor for directional scanning

- Optical encoders for speed and movement feedback


## Component Used
<img width="479" height="336" alt="image" src="https://github.com/user-attachments/assets/2a8c16e1-5be9-4415-8680-04cb4dd30c86" />

## Working Principle

- The robot moves forward continuously.

- The ultrasonic sensor measures the distance to obstacles.

- If an obstacle is detected below a threshold distance:

- The robot stops.

- The servo rotates the ultrasonic sensor to check left and right.

- Based on the measured distances:

- It turns left, right, or moves backward.

- Wheel encoders provide speed feedback to maintain straight movement.


 
 ## Features

- Autonomous obstacle detection and avoidance

- Adaptive movement based on real-time sensor input

- Servo-based directional scanning

- Encoder-based movement correction

- PID-like motor balancing using encoder difference

## Software & Tools

- Energia IDE (for Tiva C programming)

- Embedded C

- Serial Monitor for debugging


## Pin Configuration
<img width="781" height="624" alt="image" src="https://github.com/user-attachments/assets/b80b8b4b-622d-4e69-8202-007cc7c4b710" />


## How to Use

- Upload the provided .ino file to your Tiva C board using Energia IDE.

- Connect all hardware components as per the pin configuration.

- Power the robot and place it inside a maze environment.

- The robot will start navigating autonomously.
