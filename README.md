# DC Motor Modeling, Simulation and Control

## Overview

This MATLAB project demonstrates the mathematical modeling of a DC motor system, starting from the physical model and a set of assumptions. This model is then simulated through Runge Kutta techniques.
The motor is then controlled for performance requirements using a PID controller and a Linear Quadratic controler. The effect of uncertainty are analysised an the tunable controller is computed using mu synthesis.

## Tools Used

ode45(), PID, LQR, mu-synthesis.

## Running the Simulation

An overview of the script can be seen in the attached jupyter notebook file, or through the attached MATLAB file.

## Results

- Simulation results are plotted for the DC motor's angular velocity over time.
- Bode plot, pole-zero plot, and step response of the DC motor system are analysed.
- PID controller is designed and tuned to meet specific performance criteria.
- Linear quadratic control is tuned to meet the same performance criteria.
- Robust control through mu-synthesis are implemented and compared.

## Requirements

- MATLAB R2018b or later.
- Control System Toolbox for PID tuning and system analysis.
