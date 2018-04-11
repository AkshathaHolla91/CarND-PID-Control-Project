# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)




## Reflection

A PID controller is used here to calculate the steering angle of the simulated car. The PID controller calculates an error value (cross track error) between the desired trajectory and the current position and applies a correction to a parameter(in this case steering angle) based on proportional, integral and differential terms.

The three terms contribute as follows

* The Proportional gain or Kp here causes the car to steer in proportion to the cross track error. The cross track error determines the position error of the car from the center of the lane. High values of the proportionality term cause the steering angle to be proportional to the error which leads to the car wobbling a lot from left to right when it tries to correct the position and also leads to a lot of overshooting of position of the car. At lower values the steering angle is changed slowly proportional to the error and hence oscillations are reduced.

* The Differential gain or Kd here is introduced to control the rate of change of error ie. it is the derivative of change of cross track error from one position to next. The differential term is used to counter the effects of overshooting by using the derivative to counter steer and smoothen out the correction when it notices that the error is becoming smaller/larger over time. This results in a smoother correction trajectory instead of an oscillating one.

* The Integral gain or Ki is introduced here with summation of all cross track errors which help in countering the effects of system bias in the car. When the cross track error is continuously large at a constant value(The car is travelling continuously shifted from the center  along one side of the lane) then the summation of the errors multiplied by the integral gain(Ki) term is used to compensate the bias and slowly bring the car to the center of the lane. It is also useful in correcting the position of the car when the it  travels along curved road by countering the drift in the car and brings it to the center.


 I started with tuning the gain parameters and set the value of Kp to 1 which led to lot of oscillations by keeping the other 2 parameters at zero. I slowly reduced the value of Kp until the oscillations were minimal. Later I started with a value of 1 for the differential gain which brought back the oscillations and hence I had to reduce the term till 0.5 to get good results , still I was facing issues in the turnings and curves of the track hence I set the value ok Ki at 0.005 to try and ensure minimal deviation from the center of the lane during simulation. These values worked well in my machine(which has a pentium processor) but gave bad results in machines with better performance. Hence I tried various combinations and found that the values provided in the classroom session were found to be optimal with a throttle of 0.3 in most machines and worked well with a throttle of 0.1 in machines with lower performance and decided to use those values finally.

The video of my result can be seen [here](https://github.com/AkshathaHolla91/CarND-PID-Control-Project/blob/master/output_video.m4v)




