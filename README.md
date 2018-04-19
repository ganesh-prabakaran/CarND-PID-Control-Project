# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

## PID Controller
A PID (proportional–integral–derivative) controller is a control loop feedback mechanism widely used in industrial control systems / applications requiring continuosly modulated control. The controller continuously calculates the error value. With respect to this carnd project, this is known as cross-track error (cte) which indicates the distance between the actual car position on the road and the reference trajectory. The controller in turn use this error information to generate an output (steering angle of the car) with an objective to minimize the cte.

### P - proportional gain
The proportional term denotes an output that is proportional to the cte. P controller (instead of PID) has the drawback of oscillating around the desired value. The proportional gain generates the control output (steering angle) which is -Kp times proportaional gain, where Kp is a positive constant.

### D - differential gain
The osciallation problem with the P controller can be addressed using a differential gain which is the difference between the current and past cte. Differential gain is added -Kd times to generate the steering angle of the PI controller where Kd is a positive constant. 

### I - integral gain
A typical PI controller has a potential problem in terms of bias/residual error. The integral term seeks to eliminate the residual error by adding a control effect due to the historic cumulative value of the error. When the error is eliminated, the integral term will cease to grow. This will result in the proportional effect diminishing as the error decreases, but this is compensated for by the growing integral effect. With respect to this project, this term is more about contributing corrections in the curves. Integral gain is added -Ki times to generate the steering angle  where Ki is a positive constant.
 

### Hyper Parameter tuning
In this project, all the parameters were tuned manually. There are other approaches like twiddle mentioned in the class, however, I prefered this approach to have a complete understanding on the effect of tuning each parameter and also due to the slowness of the machine I used for this project. Though I did manually, I simply followed a brute force approach taking some leaf out of the twiddle. I calculated the Total Error after each lap and accordingly adjusted the Kp, Kd and Ki and finally arrived at a value which driven smoothly in the entire lap.

First I started with P controller where I set Ki and Kd values to 0 and located an optimal Kp value where the oscillates and settles over a time. Then, I added Kd to introduce dampening to mitigate the oscillation. However, I still found issues in the curves where the turn was too late and I used the Ki to improve the control of the car. Below are the final values 
Kp : 0.24 , Ki : 0.0004 , Kd :4.5 , throttle : 0.3 

[Link to Output Video of PID controller](https://youtu.be/r32rVirsbOA)

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
4. Run it: `./pid 0.24 0.0004 4.5 0.3`. 
 
