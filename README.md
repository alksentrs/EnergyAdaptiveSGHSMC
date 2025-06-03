# Energy-Adaptive SGHSMC Particle Filter for Tracking

This repository contains a MATLAB implementation of the Energy-Adaptive Stochastic Gradient Hamiltonian Monte Carlo (SGHSMC) particle filter, as described in the paper:

**"Energy-Adaptive SGHSMC: A Particle-Efficient Nonlinear Filter for High-Maneuver Target Tracking"**  
Chang Ho Kang (Sejong University) and Sun Young Kim (Kunsan National University)

## Paper Summary
Tracking targets with nonlinear motion patterns remains a significant challenge in state estimation. The Energy-Adaptive SGHSMC filter combines adaptive energy dynamics with efficient particle sampling, featuring a novel energy function that automatically adapts to target dynamics while minimizing the need for resampling. By integrating Hamiltonian Monte Carlo sampling with stochastic gradient techniques, this approach achieves a 40% reduction in computational overhead compared to traditional particle filters while maintaining particle diversity.

**Key results:**
- Simulation studies show a 39% improvement in tracking accuracy over the extended Kalman filter (EKF) and 29% over standard sequential Monte Carlo methods.
- Experimental validation with a quadrupedal robot demonstrates a 77% better accuracy than EKF, with computational efficiency suitable for real-time applications.
- The algorithm is effective in scenarios involving rapid state changes and irregular motion patterns, offering a robust solution for challenging target tracking problems.

**Keywords:** nonlinear filtering, target tracking, sequential Monte Carlo, adaptive energy function, stochastic gradient Hamiltonian Monte Carlo

## Features
- Implements the full SGHSMC particle filter algorithm with adaptive mass and energy parameters
- Supports multiple particles (Monte Carlo filtering)
- Simulates noisy measurements
- Visualizes tracking performance (robot, target, and measurements)

## Files
- `main.m`: Main script. Runs the simulation, filter, and plotting.
- `plot_results.m`: Plots the robot trajectory, target, and measurements.
- Helper functions in `main.m`: Implements prediction, mass matrix update, gradient computation, and measurement model.

## How to Run
1. Open `main.m` in MATLAB.
2. Run the script. The simulation will:
    - Generate a circular target trajectory
    - Simulate noisy measurements
    - Run the SGHSMC particle filter with 200 particles
    - Plot the results (robot estimate, target, and measurements)

## Parameters
You can tune the following parameters in `main.m`:
- `M`: Number of particles
- `params.R`: Measurement noise covariance (increase for more noise)
- `params.alpha0`, `params.gamma1`: Adaptation rates
- `params.C`, `params.B`: Friction and noise in the filter
- `params.epsilon`, `params.m`: Step size and number of inner simulation steps

## Output
- **Trajectory plots**: Show the true target, noisy measurements, and the filter's estimate.
- **Tracking plots**: Show position and velocity tracking over time.
- ![Screenshot 2025-06-02 220743](https://github.com/user-attachments/assets/617ba170-78f8-43b9-b6ed-607eb43dcba0)

## Reference
If you use this code, please cite the original paper:
> Kang, C. H., & Kim, S. Y. (2023). Energy-Adaptive SGHSMC: A Particle-Efficient Nonlinear Filter for High-Maneuver Target Tracking. Sejong University & Kunsan National University.

---
Feel free to modify the code for your own experiments or to track other trajectories or systems! 
