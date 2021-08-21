---
layout: article
date: 2021-07-30
title: Start Here
---

## Goal

Our goal for this project is to be able to create a fully autonomous, life sized chess game that people will be able to play using only voice commands. This project was inspired by the Wizard's Chess game in Harry Potter and the Philosopher's Stone. 

## Setup

For our setup, we will have a Raspberry Pi 4 control center that will be controlling all 64 robots on the field. The control center will keep track of the position and valid movements of each robot using our Chess algorithm. Each tile on the field will have unique RFID tags to indicate its spot on the board and each robot will have an RFID reader to scan the tags. This way, the robot knows exactly where it is at all times.
To differentiate between pieces, we will add LED indicators on each piece. That way, we will also be able to play in the dark!

## Subsystems

#### Hardware
- Robot
- Field

#### Software
- Control Center
    - Overarching game controller
    - Determines valid moves based on input
- Sensors
    - RFID tags and scanners
    - Gyro


## What have we been up to?

### Spring 2021

This spring was our first semester on this project! 

- Designed the triangular chess pieces
  - Each robot is about 7 in by 9 in and laser cut from .5 in plywood and each square on the field is 1.5 ft by 1.5 ft
  
<img src="assets/images/WC-prototype1.jpg" alt="Chess Piece Prototype" width="400"/>

- Put together prototype with motor movement

{% include googleDrivePlayer.html id="18uXqdxGblfsNUdoEI2qEVPu5-sYCvQ7a/preview" %}

- Wrote chess algorithm using Python
- Simulated robot movement in ROS 

## Plans for the Future

In order to increase autonomy, we plan on working with cameras and object detection rather than sensors to determine robot positioning and move validation. 