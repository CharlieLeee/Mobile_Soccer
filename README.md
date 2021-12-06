# Final Project for Mobile Robotics

## Introduction

Soccer Player.

## Modules

### Thymio Interface

Interface to communicate with Thymio, including command the motors, get the sensors' values and so on.

### Motion Control

Motion Control Strategy

### Vision

Exact environment information, including

* Coordination Axis based on soccer field
* Positions of
  * Ball
  * Gate
  * Obstacles(with shape, or in pixel?)
* Pose of Thymio(by vision)

### Filtering

Calculate where Thymio is based on

* odometer
* vision
* *accelerator*

### Local Navigation

Local Obstacles(out of expectation) strategy

### Global Navigation

* Calculate thymio's center pose that the head of thymio will be at the goal point
* Plan a path from the start to the goal avoiding obstacles.
  * Simplify the nodes with same direction
  * Simplify the path with Ramer–Douglas–Peucker Algorithm or other one.
* return a path with direction suggestion

## Main Program

1. Settings

    1. Set all the parameters
    2. Set up Communication with Thymio \<`Thymio Interface`>
    3. Get map information. \<`Vision`>

2. Set up for the Problems

    1. Get the real goal of Thymio\<`Filtering`>
    2. Plan a path(list of waypoints) from the start to the goal\<`Global Navigation`>

3. Track the Thymio\<`Filtering`>

    1. filter with odometer(every 10ms?)
    2. filter with odometer and vision info(every 2s?)

4. Follow the Path\<`Motion Control`>

    1. Check if it reached the next waypoint
        * Yes: delete this waypoint, and check if we've reached the end of the path(i.e. goal)
        * No: `4.2`
    2. Check if the direction is not far from the direction to next waypoint
        * Yes: Check if it is near to the waypoint
            * Yes: Move Carefully
            * No: Just move forward at normal speed
        * No: Add some rotational velocity to correct the direction

5. Tackle the problems
    1. Track the Thymio
        1. Kidnapped? [TODO]
            * Yes: reset the problem and `5.`
            * No:  `5.2`
    2. Finished?
        * Yes: Stop and Go to the Next Problem
        * No: `5.3`
    3. Obstacle on the way?
        * Yes: Ask \<`Local Navigation`> what we should do
        * No: follow the path `4.`

## Scenarios

### Kidnapping[TODO]
