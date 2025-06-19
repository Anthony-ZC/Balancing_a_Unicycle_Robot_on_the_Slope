# Balancing a Unicycle Robot on the Slope
It is a MATLAB/Simulink implementation for [**Columbi EE6602 Modern Control Theory**](https://www.columbia.edu/~ja3451/courses/e6602.html) final project created by ***Zewen "Anthony" Chen***. A detailed report) is also inclued in this repository.


## Project Overview
This project aims to stablize a Unicycle Robot on a non-flat condtion (slope), which is different from current research and also a harder problem.
### System Modeling
This project mainly adopts and extented the modeling methodology in the paper:

> **"Pitch Control of an Active Omni-Wheeled Unicycle Using LQR"**  
> by Sudarshan M. Samarasinghe and Manukid Parnichkun, ICA-SYMP 2019  
> 📄 [IEEE Xplore](https://ieeexplore.ieee.org/document/8955066)

The original paper describe the Unicycle Robot Systmm on a flat plain and proposed a linearization method for the nonlinear system. I rewrite the System into a on-slope version and proposed 2 extra linearization methods and compare them in the project.

### Control Method
This project use following advanced method to achieve stability of the closed-loop linear and non-linear system:
 - Output Feedback Control using Lyapunov Equation
 - State Feedback Control using H2 Optimal Control
 - State Feedback Controller using H∞ Optimal Control
 - Output Feedback Controller using H∞ Optimal Control

## Codebase Guideline
### Requirement
All the controllers' implementation require using [**CVX**](https://cvxr.com/cvx/doc/install.html#install) to solve the corresponding LMIs. Please install it.
### Initializig Linearized Model
Run the corresponding MATLAB files to setup the linearized models:
 - [Linearzing around straight up (regardless the slope angle)](./report_model_negative_theta.m)
 - [Linearzing around the equilibrium (related to the slope angle)](report_model_equilibrium.m)
### Get the closed-loop controllers
Run the corresponding MATLAB files to get the controllers and see the results of the closed-loop linearized model behavior.
 - [Lyapunov Equation based Output Feedback Controller](./report_controller_output_feedback_LMI.m)
 - [H2 Optimal State Feedback Controller](report_controller_state_feedback_H2.m)
 - [H∞ Optimal State Feedback Controller](report_controller_state_feedback_Hinf.m)
 - [H∞ Optimal Output Feedback Controller](./report_controller_output_feedback_Hinf.m)

### Closed-loop Nonlinear behavior
After runing a certain MATLAB file for initializig linearized model and a certain MATLAB file for controller. Open the [Simulink file](unicycle_nonlinear_Cl.slx) and run it to see the closed-loop nonlinear model behavior.
> [!Note]
> See section 4.4 of the report for more information about Simulink module functionality.
> See section 6 of the report for balancing the unicycle robot on the Time-Variant Slope.
