# Line-follower-pick-and-place-robot

<img width="1152" height="648" alt="8c604dcd-63df-4154-b3c7-e7de016d16f9" src="https://github.com/user-attachments/assets/bff603f0-52c0-4c00-b923-45a2be88aec1" />

the robot which  follow the line and recoginize the color and will pick and place a box at specified location.

# Project Overview

This project is an autonomous line-following pick-and-place robot designed for a competition challenge involving navigation, color detection, counting, decision-making, and object manipulation.

# Core Functionalities

1. Line Following & Color Detection

The robot follows a predefined black line track using IR sensors.
It detects and counts Red, Green, and Blue cubes placed on the left side of the track.
Maintains a real-time count of each detected color while navigating.
Detects track intersections marked by a fully black strip.
At the intersection, the robot analyzes the collected counts and determines the dominant color (most frequently detected color).

2. Intelligent Pick and Place Operation

After identifying the dominant color, the robot moves forward.
It locates the corresponding colored box placed after the intersection.
Uses the pick-and-place mechanism to pick the correct box.
Continues line following toward the destination zone.

3. Finish Region Detection

The robot detects broken line patterns indicating the final stage.
After detecting broken lines, it continuously scans for a fully black FINISH region.
Stops at the finish area and places the picked box on the ground.

# Competition Task Flow
Follow black track
Detect and count RGB cubes
Reach checkpoint/intersection
Compare color counts
Identify dominant color
Pick corresponding box
Continue navigation
Detect broken line section
Detect finish region
Place box successfully

# Technologies Used
Arduino
IR Sensors (Line Following)
Color Sensor
Servo Motor (Pick & Place)
DC Motors with Motor Driver
Embedded C / Arduino Programming

# Short GitHub description:
Autonomous line-following robot with RGB color counting, dominant color decision-making, and intelligent pick-and-place operation for competition automation.
