# Multi-Agent Obstacle Avoidance using Velocity Obstacles and Control Barrier Functions

[![arXiv: 2409.10117v2](https://img.shields.io/badge/arXiv-2409.10117v2-B31B1B.svg)](https://arxiv.org/abs/2409.10117v2)
[![Video on YouTube](https://img.shields.io/badge/YouTube-Video-red.svg)](https://www.youtube.com/watch?v=Ox8v2s17gLw&ab_channel=AlejandroS%C3%A1nchezRoncero)

*Alejandro Sánchez Roncero, Rafael I. Cabral Muchacho, and Petter Ögren*

---

This repository contains the MATLAB code used for the experiments in the paper "**Multi-Agent Obstacle Avoidance using Velocity Obstacles and Control Barrier Functions**."

## Overview

This project provides a MATLAB implementation that:
- Uses Velocity Obstacles (VO) for guidance as a relaxed constraint in the optimization objective.
- Guarantees safety through a Control Barrier Function (CBF) constraint.
- Works with both second-order integrator dynamics and car-like dynamics.

There are two major folders—one for second-order integrator dynamics and another for car-like dynamics. Each folder has the same structure:

- **`h_cbf_system.m`**: Main file implementing the collision avoidance algorithm (methods and the optimization problem).
- **`run_scenario.m`**: Helper script to run the simulations.
- **`sim_config.m`**: File containing simulation parameters.
- **`scenarios/`**: Folder with various scenario files, each constructed as a Simulink model.

## Requirements

- **MATLAB**
- **Simulink**
- **Simulink 3D Animation** (or equivalent, if 3D visualization is needed)
- **Symbolic Math Toolbox** (used by `generate_functions.m` to create methods stored in `generated_functions/`. If you do not have this toolbox, you can implement those methods manually.)

## How to Run the Code

1. **Open MATLAB** and set the current folder to either the second-order integrator folder or the car-like dynamics folder, depending on which you want to simulate.
2. **Edit `sim_config.m`** to set the desired simulation parameters.
3. **Run `run_scenario.m`** to start the simulation.
