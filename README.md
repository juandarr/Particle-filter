# Particle filter

Context: A robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

This repository contains the implementation of a two dimensional particle filter in C++. The particle filter works in the context of a set of landmark data from a map and some initial localization information (similar to what a GPS would provide). At each time step the filter receives observation and control data. This data is used to perform the following steps:

### Initialization

This step is performed once at the beginning of program execution. It initializes the position, orientation and weight data of a number of particles created around given initial noisy location information.

### Prediction

In this step the state of each particle is predicted by motion models used to calculate a new position and orientation given the previous velocity, yaw rate and uncertainty parameters of the sensors.

### Update

This process updates the weights of the particles according to the likelihood of each particle representing the current robot location. The likelihood is calculated by measuring the landmark observations as seen by the sensor robot (from each particle point of view) in the map coordinate system (by means of the homogeneous transformation) and their association to actual map landmarks within sensor range. The set of weights for all particles is normalized after the likelihoods are calculated. 

### Resampling

Based on the normalized weights a new set of particles is created. Bigger relative weights among the values imply that the correpondent particle will be draw from the previous particle pool more often. This will end up in the best particle states representing the best chances to estimate the real robot location. 

## Running the Code

This project involves the Udacity Self Driving Car  Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases). The simulator serves as a visual output of the logic performed in the repository program, which creates a server that uses the simulation as a front end. 

This repository includes two files that can be used to set up and install uWebSocketIO for either Linux or Mac systems (`install_ubuntu.sh` and `install_mac.sh`). For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

- ./build.sh
- ./run.sh
 
 Any executables can be removed with the following script:

 - ./clean.sh

After running the program, open the siumulator. You should get a `connected` message in the terminal output, meaning the simulator is connected to the server program and is ready to run. 

## Data flow 

Here is the main protocol that main.cpp uses for uWebSocketIO in communicating with the simulator.

### INPUT: values provided by the simulator to the c++ program

```C++
// sense noisy position data from the simulator
["sense_x"]
["sense_y"]
["sense_theta"]


// get the previous velocity and yaw rate to predict the particle's transitioned state
["previous_velocity"]
["previous_yawrate"]

// receive noisy observation data from the simulator, in a respective list of x/y values
["sense_observations_x"]
["sense_observations_y"]
```

### OUTPUT: values provided by the c++ program to the simulator

```C++
// best particle values used for calculating the error evaluation
["best_particle_x"]
["best_particle_y"]
["best_particle_theta"]

//Optional message data used for debugging particle's sensing and associations

// for respective (x,y) sensed positions ID label
["best_particle_associations"]

// for respective (x,y) sensed positions
["best_particle_sense_x"] <= list of sensed x positions
["best_particle_sense_y"] <= list of sensed y positions
```

Your job is to build out the correct particle filter logic until the simulator output says:

```
Success! Your particle filter passed!
```

This repository contains the final outcome of this achievement. 

## Particle Filter folder tree

The directory structure of this repository is as follows:

```
root
|   build.sh
|   clean.sh
|   CMakeLists.txt
|   README.md
|   run.sh
|
|___data
|   |   
|   |   map_data.txt
|   
|   
|___src
    |   helper_functions.h
    |   main.cpp
    |   map.h
    |   particle_filter.cpp
    |   particle_filter.h
```

## License

Copyright (c) Udacity and Juan David Rios. All rights reserved.

Licensed under the [MIT](https://github.com/juandarr/particle-filter/blob/master/LICENSE) License.