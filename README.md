# The C++ FCND Control Project Readme #

This project consist on the development of a flight control program for a quad-rotor. The solution was implemented using the python controller solution as a reference: [Python controller code](solution_Lesson_4-3D_Drone-Full-Notebook.py)

Below are details on the implementation:

## Motor Commands ##

The `GenerateMotorCommands` function was implemented considering the forces acting on the quad-rotor (Thrust and Moments) based on the following equations:
Ftotal = F1+F2+F3+F4
Mx = (F1+F4-F2-F3)*l
My = (F1+F2-F3-F4)*l
Mz = (F1+F3-F2-F4)*kappa

where l is the distance from the center of the drone to the center of the propellers on the x and y axis; and kappa is the drag/thrust ratio

Those equations were solved simultaneously to get the equations for F1, F2, F3 and F4

## Body rate controller  ##

The `BodyRateControl` is a proportional controller responsible of adjusting the angular velocity of the quad-rotor. The resulting moment is calculated based on the difference between the desired body rates minus the measured body rates; that difference is multiplied by the moment of inertia and the proportional gain `kpPQR`.

## Roll pitch controller  ##

The `RollPitchControl` is responsible for commanding the roll and pitch rates and it is based on a proportional controller, where `kpBank` is the proportional gain. The tilt angle is constrained not to exceed the `maxTiltAngle`. The equations used to develop the pitch-roll controller are shown in section 4.2 in the [3D drone lesson](Lesson_4-3D_Drone-Full-Notebook.ipynb)

## Altitude controller  ##

The `AltitudeControl` is a PID controller responsible for controlling the altitude of the drone. `kpPosZ`, `kpVelZ`, `KiPosZ` are the proportional, derivative and integral gains respectively. To avoid exceeding the maximum ascent and descent rates, the acceleration is forced to be within the following range: `-maxAscentRate/dt` and `maxDescentRate/dt`. The equations used to implement the altitude controller are shown in section 5.3 in the [3D drone lesson](Lesson_4-3D_Drone-Full-Notebook.ipynb).

## lateral position controller  ##

The `LateralPositionControl` is a proportional-derivative controller. `kpPosXY` and `kpVelXY` are the proportional and derivative gains. The mean velocity and acceleration are calculated as the square root of the sum of the square of the values (root mean square). The maximum velocity and acceleration are limited to `maxSpeedXY` and `maxAccelXY` respectively. The controller also accounts for the feed-foward acceleration. The equations used to implement the lateral position controller are shown in section 4.1 in the [3D drone lesson](Lesson_4-3D_Drone-Full-Notebook.ipynb).

## Yaw controller  ##

The `YawControl` is responsible for controlling the angular velocity around the z-axis. This is a proportional controller with `kpYaw` as the proportional gain. The yaw error is constraint to be within 0 to 2 pi. The equation to implement the yaw controller is shown in section 5.2 in the [3D drone lesson](Lesson_4-3D_Drone-Full-Notebook.ipynb).

## Tuning ##

All the gains were tuned based on the different scenarios to ensure all the criteria are met.
