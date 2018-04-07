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

The car drives according to the speed limit. As shown in the image below, the full lap of 4.32 miles is driven in about 5 min 30 s.

![Full lap][image3]

Max Acceleration and Jerk are not Exceeded.

Car does not have collisions.

The car stays in its lane, except for the time between changing lanes.

The car is able to change lanes

![Change Lane][image2]

### Reflection

#### Introduction

This section provides details on the underlying model that generates the car path.

The code provided during the walkthrough is reused as a starting point and will not be explained in further details.

The strategy is then to add:

* a finite state machine
* implement other vehicle predictions
* and define cost functions in order to balance the goal and constraints

#### Refactoring Walkthrough Code

The code provided in the walkthrough is extracted from the `main` function into the `get_trajectory` function. It requires several input arguments, but basically generates the set of next x and y points in map coordinates given the previous path points, the current state of the car, and 2 important parameters which are the specified lane and reference velocity.

The internal back and forth conversion between map and car coordinates is encapsulated in this function, so as the generation of Frenet coordinates to simplify the spline computation.

#### Finite State Machine

The finite state machine contains 5 states:

* KLN: keep lane no change
* KLA: keep lane accelerate
* KLD: keep lane decelerate
* LCL: lane change left
* LCR: lane change right

The `get_successor_states` function defines the possible next states based on the 2 parameters that can change between states:

* the lane
* the reference velocity

Generally, all 5 states are possible except when:

* the reference velocity is close to the speed limit, meaning accelerating is not an option
* the reference velocity is null, meaning the car is close to a stop and decelerating is not an option
* no left or right lane is available given the position of the car on the road, meaning the corresponding lane change is not an option 

The next step is to assess each possible next state in light of the corresponding planned trajectory and other vehicle predicted trajectories.

Our car trajectory is planned using the refactored walkthrough code mentioned above, with the relevant state arguments passed in each case:

* for KLN: no change in lane nor reference velocity
* for KLA: no change in lane, but reference velocity is incremented by 0.224 (value used in walkthrough).
* for KLD: no change in lane, but reference velocity is decremented by 0.224.
* for LCL: no change in reference velocity, but lane is decremented by 1 (going towards left side of road). 
* for LCR: no change in reference velocity, but lane is incremented by 1 (going towards right side of road). 

#### Other Vehicle Predictions 

Before predicting other car trajectories, we use 2 functions in order to identify the cars we should mostly focus on:

* the `get_closest_front_car_in_lane` returns the id of the vehicle right next in front of our car (no matter how far): this will be the only vehicle we'll consider when evaluating the state KLN, KLA, and KLD since in these states there is no change of lanes.
* the `get_closest_cars_in_side_lane` returns the id of the vehicles (can be several) in the lane right next to our car, either on the left or on the right: these will be used when evaluating a lane change respectively left or right. Only the vehicles within a certain distance from our car are actually considered. The distance is computed based on the reference velocity in order to include all vehicles within 3 seconds of driving at the specified reference velocity. The distance is approximated by a simple comparison in Frenet coordinate s and does not include the value d.

For each of the identified vehicle of interest, we perform a simple prediction in the `get_prediction` function. The vehicle trajectory is approximated by assuming a constant speed and no change of lane. This is a simplification good enough for our fast loop cost optimization. Frenet coordinates are handy here to get a list of 50 predicted x and y values for each of the vehicles.  

#### Cost Optimization

Once we have for each next possible state:

* the intended new state
* the planned car trajectory
* the list of other vehicles of interest
* and their predicted trajectories,

we can run a cost optimization in order to define which state should be next.

We consider here 4 different costs:

* a collision cost computed in the `get_collision_cost` function: this is a binary cost which is set to 1 if at any point in considered trajectories our car and the vehicle of interest are predicted to be less than 10 meters away. Cost is set to 0 otherwise.
* a cost associated with how close we'll get to another vehicle with the `get_too_close_cost` function: here a safety distance, equivalent to 1.5 seconds, is computed based on the reference velocity. The cost is gradually increasing from 0 (outside the safety distance) to 1 (if distance is 0, i.e. a collision about to happen).
* a cost associated to the acceleration of the car, in the `get_acceleration_cost` function: here we compute the average car acceleration over 0.2 s (10 timesteps of 0.02 s) and check if it does not go over a specified limit of 8 m/s2 (conservatively lower than the hard margin of 10 m/s2 given in the rubric).
* a cost related to the overall efficiency of the driving, with the `get_efficiency_cost` function: the cost function compares the reference velocity to the speed limit, and decreases gradually from 1 (at start) to 0 (when the car reaches the spee limit). 

The other constraints (jerk under limit, car stay in lane) were anticipated to be met by default as a consequence of using the code presented in the walkthrough.

Weights (fined-tuned through common senses and experiments) are applied to these different costs to:

* 100 for collision
* 50 for getting too close
* 20 for accelerating too much
* 5 for lack of efficiency

In the case of the evaluation of lane changes, there can be several other vehicles so these costs must be computed against each of them, and the nax value is kept as the overall cost for that particular possible next state.

The final step is to execute the next state with the lowest cost, i.e. propagate a permanent change in the lane and reference velocity state, and return the corresponding planned trajectory points to the simulator.

[//]: # (Image References)

[image1]: ./overall.JPG "Overall result"
[image2]: ./change_lane.JPG "Change lane"
[image3]: ./full_lap.JPG "Full lap"
