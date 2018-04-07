# CarND Path Planning
Self-Driving Car Engineer Nanodegree Program

## Project Objectives
In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. Are provided the car's localization and sensor fusion data. There is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, even though  other cars try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

## Project Files

* [Source code](https://github.com/schambon77/CarND-Path-Planning/tree/master/src)
* [ReadMe](https://github.com/schambon77/CarND-Path-Planning/blob/master/README.md)

## Project Rubric Points

Project rubric points can be found [here](https://review.udacity.com/#!/rubrics/1020/view).

### Compilation

The code compiles without errors with cmake and make.

### Valid Trajectories

The car is able to drive at least 4.32 miles without incident.

![Overall][image1]

The car drives according to the speed limit.

Max Acceleration and Jerk are not Exceeded.

Car does not have collisions.

The car stays in its lane, except for the time between changing lanes.

The car is able to change lanes

![Change Lane][image2]

### Reflection

There is a reflection on how to generate paths.

The code model for generating paths is described in detail. This can be part of the README or a separate doc labeled "Model Documentation".

[//]: # (Image References)

[image1]: ./overall.jpg
[image2]: ./change_lane.jpg
