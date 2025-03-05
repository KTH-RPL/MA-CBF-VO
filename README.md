# VO-CBF Multi-Agent Collision Avoidance

This repository contains MATLAB code for implementing multi-agent collision avoidance by combining Velocity Obstacles (VO) with Control Barrier Functions (CBF). The approach is based on the paper:

**Multi-Agent Obstacle Avoidance using Velocity Obstacles and Control Barrier Functions**  
*Alejandro Sánchez Roncero, Rafael I. Cabral Muchacho, and Petter Ögren*  
[Download the paper](https://arxiv.org/abs/2409.10117v2)

## Overview

This project provides a MATLAB implementation that:
- Uses VO for guidance while incorporating a relaxed VO term into the optimization objective.
- Guarantees safety through a CBF constraint.
- Works with both 2nd-order integrator dynamics and car-like dynamics.

The main functionality is implemented in `h_cbf_system.m`. You can run the simulations with the helper script `run_scenario`, while simulation parameters are configured in `sim_config.m`. Different scenarios are provided in the `scenarios/` folder.

## Project Structure

- **h_cbf_system.m** – Main MATLAB file implementing the collision avoidance algorithm.
- **run_scenario.m** – Script/function to run the desired simulation scenario.
- **sim_config.m** – Configuration file containing simulation parameters.
- **scenarios/** – Directory with various simulation models/scenarios.

## Requirements

- MATLAB (R2018b or later is recommended)
- (Include any additional toolboxes if required, e.g., Optimization Toolbox, Simulink, etc.)

## How to Run the Code

1. **Open MATLAB** and set the current folder to the root directory of this repository.
2. **Add the repository to your MATLAB path** (if not already added):
   ```matlab
   addpath(genpath(pwd))
