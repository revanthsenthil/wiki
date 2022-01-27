---
title: Drone Delivery - Overview
date: 2021-10-27 00:00:00 Z
author: Sooraj Chetput
---

## Mission

Through the exploration of computer-vision, flight, and obstacle avoidance, the goal of this project is to develop fully autonomous drones that can deliver light-weight packages.

The quadcopter is one of the simplest and most stable types of drones. The long term goal of this project would be to deliver packages across long distances. The user will be able to enter the end destination of a package on a mobile app and the drone can autonomously drive to the requested location to drop off the package.

## Short term (Fall 2021)

We plan on begining our prototyping with a drone that ARC has had for a few years. We want to

1. set up a computer vision system that can detect any object
2. enable the ARC drone to fly in complex routes without any obstacles.

## Technologies we might be using in this project (from our discussion during meeting 1.

1. Pixhawk
2. Jetson Nano
3. Raspberry PI
4. OpenCV/Some sort of vision framework.
5. Google maps APIs.
   We could keep adding to the list.

## How we plan on splitting it right now.

1. **Level D** -> Plans out the overall route, uses the Maps API, probably.
2. **Level C** -> Senses obstacles in the path as the robot is flying, and tries to calculate appropriate routes to not crash into obstacles.
3. **Level B** -> Actual GPS coordinates that will be fed to the pixhawk.
4. **Level A** -> Perception -> camera, GPS, sensors and motors.

## current subteams:

1. Obstacle Avoidance -----> Currently working on searching and understanding avoidance alogithms.
2. Computer Vision --------> Setting up a basic vision system that can recognize objects
3. Mid-level software -----> Working on getting the drone to fly.
4. Hardware. ---------> Designing the drone

Through the course of the semester, we will post updates about our progress here.
