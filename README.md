# DDPG-LQR

REINFORCEMENT LEARNING BASED DESIGN OF LQR CONTROLLER WITH APPLICATION TO DRONES

List of files:


sys_params.m: Defines all parameters for the drone and hyperparameteres for the RL agent.


MATALB RL environment is used to create a base class for training all controllers.
* DroneEnvironment_BaseClass.m : Base class (has helper functions and equations of motion for quadrotor)


<img width="325" alt="image" src="https://user-images.githubusercontent.com/31934929/189780698-2eb07afe-779c-4941-b935-2155216594bb.png">


Sub classes are then created for each individual controller: 
* DroneEnvironment_BLQR.m: Implementation of Bryson's rule and LQR controller
* DroneEnvironment_LQR.m: Implementation of Vanilla LQR.
* DroneEnvironment_BLQRPD.m: Implementation of Bryson's rule with LQR-PD.
* DroneEnvironment_RL.m: Implementation of RL agent to select weightint matrices and LQR controller.
* DroneEnvironment_LQRFsolve.m: Implementation of LM algorithm and LQR controller.

Files used to create RL agent:
* critic.m: implements the DDPG critic
* actor.m: implements the DDPG actor
* layer.m: defines a fully connected layer used in actor-critic agent.
* agent_Layer.m: compbines the actor and critic.

Fsolve:
* fsolve_optimization_v2.m: finds the optimal Q matrix using LM algo.

Result scripts:
* get_ISE.m: gets the ISE error and maximum rotation speed of each rotor
* predict.class.m: used to create trajectory plots for all five controllers.
* get_RT_ST.m: gets the rise and settling times.

# Training

To train the RL agent to follow a random trajectory run the following command: "run_DDPG()". This will train the RL agent to predict the Q matrix in LQR in order to minimize the Integral Square Error

To train the LM method for predicting Q matrix to minimise the Integral Square error, run the following command: "fsolve_optimization_v2"

# Testing
To obtain trajectory plots for all 5 controllers run the following command: "predict_classic_all()"


To get average values of Rise time and settling time across all test setpoints run the following command "get_RT_ST(environment,method)"
(environment, method) pairs variable can be one of the following:
* DroneEnvironment_LQR('test'), "noRL"
* DroneEnvironment_BLQR('test'), "noRL"
* DroneEnvironment_BLQRPD('test'), "noRL"
* DroneEnvironment_RL('test'), "RL"
* DroneEnvironment_LQRFsolve('test'), "noRL"


To get average ISE and flight time for all test trajectories run the following command "get_ISE(environment,method)". (environment,method) pair can be the same as above.


