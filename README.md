# Balancing a Unicycle Robot on the Slope
It is a MATLAB/Simulink implementation for **Columbi EE6602 Modern Control Theory** final project created by ***Zewen "Anthony" Chen***. A detailed report is also inclued in this repository.


## Project Overview
### System Modeling
This project mainly adopts and extented the modeling methodology in the paper:

> **"Pitch Control of an Active Omni-Wheeled Unicycle Using LQR"**  
> by Sudarshan M. Samarasinghe and Manukid Parnichkun, ICA-SYMP 2019  
> 📄 [IEEE Xplore](https://ieeexplore.ieee.org/document/8955066)

The original paper describe the Unicycle Robot Systmm on a flat plain and proposed a linearization method for the nonlinear system. I rewrite the System into a on-slope version and proposed 2 extra linearization methods and compare them in the project.
### System Modeling

<img src="https://latex.codecogs.com/svg.image?\begin{gather}
\dot{x}=Ax(t)&plus;\begin{bmatrix}
B_1&B_2
\end{bmatrix}\begin{bmatrix}
w(t)\\u(t)
\end{bmatrix}\\\\
\begin{bmatrix}
z(t)\\y(t)
\end{bmatrix}=\begin{bmatrix}
C_1\\C_2
\end{bmatrix}x(t)&plus;\begin{bmatrix}
D_{11}&D_{12}\\\\D_{21}&D_{22}
\end{bmatrix}\begin{bmatrix}
w(t)\\u(t)
\end{bmatrix}
\end{gather}" alt="state-space model">
