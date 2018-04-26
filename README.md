

# Model Predictive Control

In this project I have implemented Model Predictive Control algorithm that drives the car around the track in Udacity car simulator. There is an additional challenge: a 100 millisecond latency between actuations commands on top of the connection latency that I had to deal with.


# Reflection
#### Comparison of PID , Fuzzy and MPC Controller.
PID is very simple, (at least as far as control theory can be simple) and has the nice features of 1. being easy to empircally tune by hand and 2. not needing much processing power. It's also one of the best studied and most common.

With PID you have three terms: the P control, which applies a force or whatever... proportionally.

Let's say you are trying to park a car.

The P term means you give more gas based on how far the car is from the parking space

The I term means you give more gas if you haven't been moving toward the parking space for a while (it's proportional to the integral of the error value.)

The D term is a sort of damping / transient response enhancer, and would represent slowing down if you were quickly approaching the parking space.

In general P is by far the most important, while I is of intermediate importance (it lets you reach zero error over time) and D is least important, frequently set to zero in practice.

Fuzzy logic is popular in Japan. I don't know much about it. Basically, it uses less math and more manually written rules such as splitting a variable into "close to parking space" "far from parking space" "past the parking space", etc, and then defining what to do with each set of states, with a sort of smooth mixing algorithm between states. It can also be used to deal with edge cases of PID or other systems.

Model predictive control... well, it's time for the Laplace Transform party. Lots of math, and if you have no background in controls at all it will be confusing with lots of "divide by s" equals integration. Basically, you make a linear model of the process being controlled, and then solve some Lagrangey, Eulery stuff that optimizes a cost function you write, and you get control parameters.

PID is by far the simplest, and has probably been implemented for you. MPC lets you specify things like "try to save power" or the like in the cost function. (so does the LQR method, which is superficially similar in that it involves optimizing a cost function.)


In this project I have implemented Model Predictive Control algorithm that drives the car around the track in Udacity [car simulator](https://github.com/udacity/self-driving-car-sim/releases). There is an additional challenge: a 100 millisecond latency between actuations commands on top of the connection latency that I had to deal with.

### Compilation and building instructions

* Clone this repository
* Make a build directory: `mkdir build && cd build`
* Compile the project with `cmake .. && make`
* Run it: `./mpc`

### The model

#### Update equations:

1) kinematic update equations:

![equation](http://latex.codecogs.com/gif.latex?%5C%5C%20x_%7Bt&plus;1%7D%20%3D%20x_t%20&plus;%20v_t%20%5Ccdot%20%5Ccos%7B%5Cpsi_t%7D%20%5Ccdot%20dt%20%5C%5C%20y_%7Bt&plus;1%7D%20%3D%20y_t%20&plus;%20v_t%20%5Ccdot%20%5Csin%7B%5Cpsi_t%7D%20%5Ccdot%20dt%20%5C%5C%20%5Cpsi_%7Bt&plus;1%7D%20%3D%20%5Cpsi_t%20-%20%5Cfrac%7Bv_t%7D%7BL_f%7D%20%5Ccdot%20%5Cdelta_t%20%5Ccdot%20dt%20%5C%5C%20v_%7Bt&plus;1%7D%20%3D%20v_t%20&plus;%20a_t%20%5Ccdot%20dt)

2) cross track error update equation:

![equation](http://latex.codecogs.com/gif.latex?cte_%7Bt&plus;1%7D%20%3D%20cte_t%20&plus;%20v_t%20%5Ccdot%20%5Csin%7Be%5Cpsi_t%7D%20%5Ccdot%20dt%20%5C%5C),

![equation](http://latex.codecogs.com/gif.latex?cte_t%20%3D%20y_t%20-%20f%28x_t%29)

3) orientation error update equation:

![equation](http://latex.codecogs.com/gif.latex?e%5Cpsi_%7Bt&plus;1%7D%20%3D%20e%5Cpsi_t%20-%20%5Cfrac%7Bv_t%7D%7BL_f%7D%20%5Ccdot%20%5Cdelta_t%20%5Ccdot%20dt%20%5C%5C)

![equation](http://latex.codecogs.com/gif.latex?e%5Cpsi_t%20%3D%20%5Cpsi_t%20-%20%5Carctan%20f%27%28x_t%29)

#### State variables vector:

![equation](http://latex.codecogs.com/gif.latex?%5Cmathbf%7Bx%7D_t%20%3D%20%5Bx_t%2C%20%7Ey_t%2C%20%7Epsi_t%2C%20%7Ev_t%2C%20%7Ecte_t%2C%20%7Ee%5Cpsi_t%5D%5ET)

#### Actuators vector:

![equations](http://latex.codecogs.com/gif.latex?%5Cmathbf%7Bu%7D_t%20%3D%20%5B%5Cdelta_t%2C%20%7Ea_t%5D%5ET)

### MPC tuning

`N` - finite horizon size (number of predicted timesteps). This parameter heavily depends on computational capabilities, and in my case, I limited myself with `N = 10`. Higher `N`  leads to quite interesting effects on the turns: vehicle slightly overshoots but makes the turn without reducing the speed, whereas in case of small `N` it has to push the brake since it was not expecting the turn.

`dt` - time step. Again, this parameter depends on computer hardware. In case `dt` is small the controller calculates "very detailed" trajectory consisting of `N` pieces. The price for it is the need for fast computations between consecutive trajectory points. In my case, I observed increasing vehicle wiggling when `dt` is comparably small to the MPC algorithm execution time. So I set it to `dt = 0.1`.


Number one preferences are cross-track error and orientation error, then derivatives of actuator actions so that the trajectory of car movement stayed smooth enough. Finally, following velocity reference and reducing the amount of actuators power are the least prefferable goals of the controller.

### MPC preprocessing. Latency
#### Latency. Initial state prediction

There is an additional challenge in the system - 100 ms latency that is modelled by the following command: `this_thread::sleep_for(chrono::milliseconds(100));`. The problem is that MPC controller sees the system 100 ms before actuators are actually applied. The solution is to predict the system state for 100ms in future and use it as an initial condition for MPC controller:

```cpp

double latency = 0.1;
```

#### Latency. Prediciton of reference points

Additionally, when the local reference coordinates are calculated for the car, we need to predict car's position after 100 ms as well and then convert them to a local coordinate frame:

```cpp
v = v * 0.44704;
delta = delta * -1;
px = px + v*cos(psi)*latency;
py = py + v*sin(psi)*latency;
psi = psi + v * delta * latency/ Lf;
v = v + a * latency;
state << x, y, psi_local, v, cte, epsi;
