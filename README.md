# CarND MPC
Self-Driving Car Engineer Nanodegree Program

![alt text](results/CarND-MPC.mp4.gif "Result")

## The Project

In this project we'll implement **Model Predictive Control (MPC)** to drive the car around the track. This time however we're not given the cross track error, we'll have to calculate that ourselves! Additionally, there's a 100 millisecond latency between actuations commands on top of the connection latency.

This Project uses the bicycle kinematic model of the car. It is considering only accelerations, velocities and positions.

### Repo Structure

```
.
├── CMakeLists.txt
├── README.md
├── build
│   ├── CMakeCache.txt
│   ├── CMakeFiles
│   ├── Makefile
│   ├── cmake_install.cmake
│   └── mpc
├── cmakepatch.txt
├── results
│   ├── CarND-MPC.mp4
│   └── CarND-MPC.mp4.gif
└── src
    ├── MPC.cpp
    ├── MPC.h
    ├── json.hpp
    └── main.cpp
```

- The MPC is implemented on the [`src/MPC.cpp`](src/MPC.cpp).

- [`src/main.cpp`](src/main.cpp) uses the MPC class to control the vehicle around the track based on the measurements 
provided by the [simulator](https://github.com/udacity/self-driving-car-sim/releases).

### Implementation

#### Timestep Length and Elapsed Duration (N & dt)

```c++
size_t N  = 25;
double dt = 0.05;
```

##### Timestesp lenght `N`

- **Too high**: The number of parameters to be calculated by the optimiser would be too high leading to a longer processing time, potentially losing its realtime nature.

- **Too low**: The time that the optimiser is looking into the future, horizon, is very small. This leads to a lower chance of reaching the desired state

##### Elapsed duration between timesteps `dt`

- **Too high**: It will lead to unstability because it is changing the commands too late, making optimization much harder.

- **Too low**: Very similar to what happens if `N` is too low


> After some experimentation these values proved to be adequate.

#### Polynomial Fitting and MPC Preprocessing

I start by converting all waypoints to local car coordinates (`x = 0, y = 0 & psi = 0`) making sure the `cte` only depends on the `y` direction, thus simplifying calculations.

I then fitted a 3rd order polynomial to the waypoints, calculated the derivative which, due to the simplifications mentioned above,
is only dependent on the 2nd coefficient of the polynomial. 

#### Model Predictive Control with Latency

I did not optimize for latency as the controller is robust enough against it and should perform as expected. 

I've decided this way due also due to the fact that in a real world scenario the latency won't be constant or easy to measure accurately.

### Results

As seen on the gif above and the car meets the critiria:

_No tire may leave the drivable portion of the track surface. The car may not pop up onto ledges or roll over any surfaces that would otherwise be considered unsafe (if humans were in the vehicle)._

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
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).
