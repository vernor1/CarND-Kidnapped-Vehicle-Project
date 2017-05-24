# Particle Filter Project

The goals / steps of this project are the following:

* Complete the Particle Filter algorithm in C++
* Ensure that your project compiles
* Test your Particle Filter against the sample data
* Check for division by zero
* Optimize expensive computations

## Project Introduction
Your robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

This project implements a two-dimensional particle filter in C++. The particle filter is given a map and some initial localization information (analogous to what a GPS would provide). At each time step the filter also gets observation and control data. 

## Running the Code
Once you have this repository on your machine, `cd` into the repository's root directory and run the following commands from the command line:

```
> ./clean.sh
> ./build.sh
> ./run.sh
```


## [Rubric](https://review.udacity.com/#!/rubrics/747/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.

---
### Accuracy
#### This criteria is checked automatically when you do ./run.sh in the terminal. If the output says "Success! The particle filter passed!" then it means you’ve met this criteria.

The output says:
```
...
Time step: 2443
Cumulative mean weighted error: x 0.115125 y 0.112031 yaw 0.00387008
Completion time (sec): 1.17535
Success! The particle filter passed!
```

---
### Performance
#### This criteria is checked automatically when you do ./run.sh in the terminal. If the output says "Success! Your particle filter passed!" then it means you’ve met this criteria.

See the output above: `Completion time (sec): 1.18469`

---
### General
#### There may be ways to “beat” the automatic grader without actually implementing the full particle filter. You will meet this criteria if the methods you write in particle_filter.cpp behave as expected.

The particle filter satisfies all specifications.

---
## Notes

* The code is complying with the [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html).
* The C++11 automatic type deduction is used wherever appropriate.
* All function/method comments are made Doxygen-friendly
* The directory structure of this repository is as follows:
```
root
|   build.sh
|   clean.sh
|   CMakeLists.txt
|   README.md
|   run.sh
|
|___data
|   |   control_data.txt
|   |   gt_data.txt
|   |   map_data.txt
|   |
|   |___observation
|       |   observations_000001.txt
|       |   ... 
|       |   observations_002444.txt
|   
|___src
    |   main.cpp
    |   particle_filter.cpp
    |   particle_filter.h
```

### Inputs to the Particle Filter
The inputs to the particle filter are in the `data` directory. 

#### The Map*
`map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
1. x position
2. y position
3. landmark id

\* Map data provided by 3D Mapping Solutions GmbH.


#### Control Data
`control_data.txt` contains rows of control data. Each row corresponds to the control data for the corresponding time step. The two columns represent
1. vehicle speed (in meters per second)
2. vehicle yaw rate (in radians per second)

#### Observation Data
The `observation` directory includes around 2000 files. Each file is numbered according to the timestep in which that observation takes place. 

These files contain observation data for all "observable" landmarks. Here observable means the landmark is sufficiently close to the vehicle. Each row in these files corresponds to a single landmark. The two columns represent:
1. x distance to the landmark in meters (right is positive) RELATIVE TO THE VEHICLE. 
2. y distance to the landmark in meters (forward is positive) RELATIVE TO THE VEHICLE.
