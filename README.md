# CarND-Controls-PID
My solution of CarND-PID-Control-Project assignment from Udacity Self Driving Car nanodegree course, Term 2. See project assignment starter code in https://github.com/udacity/CarND-PID-Control-Project

---

## Dependencies
The project compilation and work have been verified under Mac OX Sierra, CLion 2017.1.1. 

## Code Style
To enforce [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html), I included Google's `cpplint.py` file available in `./src/Lint` folder. This tool expects installed python2.7. To check style of my code, run the following command line (from `./src/Lint`):
```
./cpplint.py ../*.hpp ../*.h ../*.cpp
```
## Project Instructions and Rubric

The project builds **pid** executable, communicating with PID Project Simulator. Detail instructions on how to set up development and test environments are given in https://github.com/udacity/CarND-PID-Control-Project and assignment description.

The project implements two PID controllers. The first controls steering behavior of the car, estimating cross track error. The second controls throttle of the car, estimating the difference between current and desired speed values.

See also https://en.wikipedia.org/wiki/PID_controller for good theory explanation.

### Effect of P,I,D components on implementation

PID controller, declared in PID.h, has three hyper-parameters:
* Kp_ determines how much current error value affects the controller output
* Ki_ determines how much past error values, accumulated over time, affect the controller output
* Kd_ determines how much future error values, expressed by the error value derivate, affect the controller output

All the hyper-parameters are non-negative.

Kp_ and Ki_ control convergence time and overshooting. Larger values result in oscillations around true value with higher amplitude and frequency:
* Steering controller forces the car to rotate left and right around the middle of the road
* Throttle controller forces the car to constantly accelerate and decelerate in a loop

Ki_ > 0 allows to eliminate constant bias error. However, a bigger value of Ki_ or long convergence time may result in loss of stability.

For example, in case of the steering wheel controller, if the car does not move for certain amount of time and is slightly off the road, the integral term collects a big error, which results in divergence from the road after the car starts its motion.

This problem is called integral wind-up. To prevent this, integral component is collected only after the car has some speed above threshold: see a boolean argument in `PID::UpdateError()` method. 

Small positive values of Kd_ helps to bring in stability into the system and dampen oscillations.

### Calibration method

Manual naive calibration of the PID controller is hard, in spite of only three hyper-parameters. I tried https://en.wikipedia.org/wiki/PID_controller#Manual_tuning but it took a lot of time and did not bring any stable result. Therefore, I ended up with the following methods:

For throttle PID, I borrowed hyperparameter values from Term 1, "Behavioral Cloning" project.

For steering PID, I applied Ziegler–Nichols method, using the following methodology:
1. Set all hyperparameters to zero
2. Slowly increase Kp_ value until the car starts to oscillate around the middle of the road. Save this value in kKu constant, defined in `main.cpp`
3. Measure the period of oscillations. In case of the project, the metric of time is the number of iterations. Therefore, dump the number of each iteration into console.
4. Find the beginning and the end of one oscillation by tracking sign change of CTE from negative to positive, and write their number difference in kTu constant, defined in `main.cpp`.
5. Calculate P, I, D values using formulae from https://en.wikipedia.org/wiki/PID_controller#Ziegler.E2.80.93Nichols_method table
6. Tune up kKu and desired speed a little further until achieving good results