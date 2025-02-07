# MobileRobotKinematicsMATLAB

This repository contains MATLAB implementations for simulating and analyzing the kinematic control of 2R and 3R robotic manipulators. The codebase is organized into two main categories based on the base type of the robot:

1. **Fixed Base Manipulators**
2. **Movable Base Manipulators**

Each category includes folders for the **2R** and **3R** manipulators, and MATLAB scripts are provided for forward kinematics, inverse kinematics, Jacobian matrix computation, and kinematics control with obstacle avoidance.

---

## Folder Structure

### 1. **fixed-base/**

- Contains simulation and control code for **fixed-base robotic manipulators**.
- Subfolders:
  - **2R/**: Kinematic control for 2R manipulator.
  - **3R/**: Kinematic control for 3R manipulator.

### 2. **movable-base/**

- Contains simulation and control code for **movable-base robotic manipulators**.
- Subfolders:
  - **2R/**: Kinematic control for 2R manipulator.
  - **3R/**: Kinematic control for 3R manipulator.

---

## MATLAB Files Overview

The MATLAB files are located in the respective **2R/** and **3R/** folders for each manipulator type. Below is a description of the key scripts:

### General Scripts

- **ForKin.m**  
  Implements the forward kinematics for the manipulator, calculating the end-effector's position based on joint angles.

- **InvKin.m**  
  Solves the inverse kinematics problem to determine the required joint angles for a desired end-effector position.

- **Jacob.m / Jacob2.m**  
  Computes the Jacobian matrix of the manipulator, which is crucial for analyzing velocities and singularities.

- **TransM.m**  
  Defines the transformation matrices used in kinematic calculations.

### Fixed Base

- **simple_2R_KC.m**  
  Demonstrates kinematic control of a 2R manipulator, including simulation and visualization of motion.

- **simple_3R_KC.m**  
  Implements kinematic control for a 3R manipulator with fixed base.

- **simple_3R_multi_qd0.m**  
  Simulates the kinematic control of a 3R manipulator with multiple initial conditions.

- **simple_3R_obstacle.m**  
  Includes obstacle avoidance for a 3R manipulator during kinematic control.

### Movable Base

- **simple_car_3R_one_obstacle.m**  
  Simulates kinematic control of a 3R manipulator with a movable base and single obstacle avoidance.

---

## How to Use

1. Clone this repository:
   ```bash
   git clone https://github.com/your_username/robot-kinematics-control.git](https://github.com/MasoudAmirkhaniV/MobileRobotKinematicsMATLAB.git
   ```

2. Open MATLAB and navigate to the desired folder (e.g., `fixed-base/2R/`).

3. Run the scripts corresponding to your simulation needs.

4. Visualize the simulation results:
   - Animations will demonstrate the kinematics control and obstacle avoidance.
   - Plots for joint trajectories and end-effector paths are generated.

---

## Applications

- Educational purposes to understand forward and inverse kinematics.
- Simulation of robotic arm motion in environments with and without obstacles.
- Development of kinematic control algorithms for robotic manipulators.

---
## Author

[Masoud Amirkhani]

---

Feel free to update this document with any additional details or contributors.
