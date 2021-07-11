---
date: 2021-07-10
title: Start Here
---

## Goal

Build a system of autonomous, scaled vehicles to play head-to-head in a game of high-speed soccer. Inspiration draws from the game, Rocket League, in which rocket-powered vehicles play soccer in 3v3 matches.

Future work may entail using the working system to launch a campus-wide competition. Here, students would have limited amount of time to build their own autonomous strategies and face-off in a tournament bracket.

## Current State

The components of the system have been outlined in order of abstraction:

![System Overview](assets/images/system-overview.png)

<!-- TODO: include ROS graph -->

Each current status of each component is as follows.

### Car

The car is the complete physical system of one player on the field. Tests were performed on off-the-shelf cars, however none met the desired criteria for acceleration and control. To solve this issue, the team replaced the electronics of the best-tested car and found the following results:

<figure class="video_container">
  <iframe src="https://drive.google.com/file/d/1hoZkHQMXcIDrOJjSXYIXwCfNXiyw8jH6/view?resourcekey" frameborder="0" allowfullscreen="true"> </iframe>
</figure>

> Left: upgraded car, middle: stock car, right: control

The car's upgrades replaced: the servo motors, receiver, motor controller and battery.

<!--
TODO: include more info on car's specific upgrades
TODO: include picture of car's upgrades
-->

### Field

<!-- TODO: fill in section -->

### ROS Interface

<!-- TODO: fill in section -->

### Sensors

<!-- TODO: fill in section -->

### Low Level Controller

<!-- TODO: fill in section -->

### Localization

<!-- TODO: fill in section -->

### Waypoint Controller

<!-- TODO: fill in section -->

### Trajectory Planner

<!-- TODO: fill in section -->

### High Level Planner

<!-- TODO: fill in section -->

## Future work

<!-- TODO: fill in section -->

## Quick links

- [GitHub](https://github.com/purdue-arc/rocket_league)
- [Spring 2021 Presentation](https://drive.google.com/file/d/1zw7jYFSYIVamnQTyYaT1TCJGP7sZOg1J/view?usp=sharing)
