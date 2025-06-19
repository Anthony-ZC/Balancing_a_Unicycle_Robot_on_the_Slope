# Balancing a Unicycle Robot on the Slope
It is a MATLAB/Simulink implementation for **Columbi EE6602 Modern Control Theory** final project created by ***Zewen "Anthony" Chen***. A detailed report is also inclued in this repository.


## Project Overview
### System Modeling
This project mainly adopts and extented the modeling methodology in the paper:

> **"Pitch Control of an Active Omni-Wheeled Unicycle Using LQR"**  
> by Sudarshan M. Samarasinghe and Manukid Parnichkun, ICA-SYMP 2019  
> 📄 [IEEE Xplore](https://ieeexplore.ieee.org/document/8955066)

The original paper describe the Unicycle Robot Systmm on a flat plain and proposed a linearization method for the nonlinear system. I rewrite the System into a on-slope version and proposed 2 extra linearization methods and compare them in the project.
### Control Method
This project use following method to achieve stability of the closed-loop linear and non-linear system:
 - Output Feedback Control using Lyapunov Equation
 - State Feedback Control using H2 Optimal Control
 - State Feedback Controller using H∞ Optimal Control
 - Output Feedback Controller using H∞ Optimal Control
