# Constraint Tighteningn MPC code Description

## Structure
* 3 Scripts : Agent, Constraint Tightening MPC, Set_operation utils

## Prerequisite
* Required to have **numpy, scipy** in the same folder as this code. 

## Description
* I implemented a code for the paper named "Robust Stable Model Predictive Control with Constraint Tightening." It has some typos in the algorithm 2(Nilpotent LQR). I corrected them and checked if it works well.
* `agent.py` defines a system and how it evolves. `constraint_tightening_mpc.py` computes tightening sets iterately and solve the optimization problem for MPC. `set_operation.py` is a just tool for computing set-operations such as **Pontryagin difference**. 

