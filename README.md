# Generation of Yaw Moment Diagrams
This repository contains algorithms to generate Yaw Moment Diagrams (YMDs) on basis of existing vehicle dynamics models and to perform further analysis.
A Yaw Moment Diagram (YMD) can provide information on maneuverability, stability, and control of a vehicle. Generally, they are applied to perform overall vehicle model and parameter studies.
Examples of Key Performance Indicators (KPIs) that can be obtained:
- maximum yaw moment and lateral acceleration
- maximum lateral acceleration in steady state and at the handling limit
- vehicle behavior at the handling limit (oversteering/understeering)
- control and stability KPIs within the possible maneuver space

![ymd_example](/resources/YMD_ax0mps2_v75mps.png)

# Software Overview
The algorithms for YMD generation are provided in two versions:
* `Matlab/Simulink`: Implementation for Matlab/Simulink 2018b (newer/older version might cause problems)
* `Python`: Implementation for Python 3.8

The `Matlab/Simulink` implementation requires the [vehicle dynamics simulation repository](https://github.com/TUMFTM/sim_vehicle_dynamics) where freely parameterizable non-linear single-track and dual-track models are included. The `Python` implementation requires an installable `.whl`-package, which is shipped with this repository. This package includes the same models as in the `Matlab/Simulink` version (non-linear single-track and dual-track models).

**Note:** The YMD Application is currently only setup for rear-wheel-driven vehicles.

## Developer
[Frederik Werner](mailto:frederik.s.werner@gmail.com)\
[Leonhard Hermansdorfer](mailto:leo.hermansdorfer@tum.de)

## Interfaces
Much effort was put into maintaining identical interfaces to the YMD generation algorithms. This allows to replace the vehicle dynamics models with your own existing ones, either in Python or in Matlab/Simulink.
The exact naming may differ between both implementation and can be found in the respective models.

The vehicle dynamics model should contain the following inputs:
1. Wheel longitudinal torque in Nm [1x4]
2. Steering angle at wheel in rad [1x4]
3. Brake pressure at each wheel in pa [1x4]
4. External Forces on CoG  in N [3x1]
5. External Moments on CoG in Nm [3x1]
6. Wind speed, ground profile, track banking and friction coefficient are optional

The following output from the vehicle dynamics model should be fed back into the YMD application:
1. long. velocity in m/s [1x1]
2. lat. velocity in m/s [1x1]
3. lat. acceleration in m/s^2 [1x1]
4. yaw rate in rad/s [1x1]
5. tire slip angle in rad (Matlab only) [1x4]

An overview of the interfaces is provided here:

![overview_YMDgeneration](/resources/overview_YMDgeneration.png)

# Running the code
Please consult the specific README file, either `/matlab_simulink/README.md` or `/python/README.md`, for further details on how to run the software in Python or in Matlab/Simulink.

# Analysis of generated Yaw Moment Diagrams
After the simulation run is finished you will find automatically generated plots and evaluated KPIs in the specified output directory.

## Visualization
For each combination of velocity and longitudinal acceleration there will be a plot:
1. Yaw Moment Diagram
3. Control Moment Diagram (change of yaw moment in respect to steering wheel angle changes)
4. Control Force Diagram (change of lateral acceleration in respect to steering angle changes)
5. Stability  Diagram (change of yaw moment in respect to body slip angle changes)
7. SA delta Diagram (wheel slip angle difference between axles (front axle slip angle - rear axle slip angle)

## Evaluated KPIs and sign convention
The KPIs will be evaluated at three different points of each Yaw Moment Diagram:
* First point is at the limit (@lim). This is the point of the maximum lateral acceleration.
* Second point is the trimmed limit (@trim). This is the point of the maximum steady state lateral acceleration (yaw moment = 0)
* Third point is straight driving (@straight).

A quick rundown on the KPIs and their sign convention (LH=left hand turn, RH=right hand turn) is provided here:
1. Mz: Yaw Moment around the z-axis (positive: LH turn in; negative: RH turn-in)
2. ay: lateral acceleration (positive: LH turn; negative: RH turn)
3. SAf: Slip angle front axle (positive: largely RH; negative: largely LH)
4. SAr: Slip angle front axle (positive: largely RH; negative: largely LH)
5. SAdiff:(Wheel slip angle difference between axles (front axle slip angle - rear axle slip angle)
6. beta: body slip angle (positive: RH turn; negative: LH turn)
7. delta: steering angle (positive: LH turn; negative: RH turn)
8. Control Diagram: change of Yaw Moment in respect to steering wheel angle changes (positive: the vehicle can still increase Yaw Moment by increasing steering angle)
9. Steering Sensitivity: change of lateral acceleration in respect to steering wheel angle changes (positive: the vehicle can still increase ay by increasing steering angle)
10. Stability: change of Yaw Moment in respect to body slip angle changes (positive: stable reaction to external disturbance; negative: unstable reaction to external disturbance)

# References
A quick and easy to read introduction into yaw moment diagrams is given in the [DrRacings Blog](https://drracing.wordpress.com/2015/07/20/the-joy-of-yaw-moment-diagrams/)

Literature from which these algorithms are inspired from:
```
@book{Milliken.1995,
 author = {Milliken, William F. and Milliken, Douglas L.},
 year = {1995},
 title = {Race Car Vehicle Dynamics},
 address = {Warrendale, Pa., USA},
 edition = {3.},
 publisher = {{SAE International}},
 isbn = {1-56091-526-9}}
```
```
@proceedings{Kang2005,
 year = {2005},
 title = {Implementing the Milliken Moment Method using Controlled Dynamic Simulation},
 volume = {1916},
 publisher = {{SAE International} and {Society of Automotive Engineers}},
 isbn = {0-7680-1561-8},
 series = {SAE-SP},
 editor = {Kang, Dongsoo and Stein, Jeffrey L. and Hofmann, Robert C. and Louca, Loucas S. and Huh, Kunsoo},
 institution = {{SAE International} and {SAE world congress}},
 venue = {Detroit, Michigan}}
``
