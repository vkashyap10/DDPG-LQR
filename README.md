# DDPG-LQR

REINFORCEMENT LEARNING BASED DESIGN OF LQR CONTROLLER WITH APPLICATION TO DRONES

List of files:


sys_params.m: Defines all parameters for the drone and hyperparameteres for the RL agent.


MATALB RL environment is used to create a base class for training all controllers.
* DroneEnvironment_BaseClass.m : Base class


<img width="325" alt="image" src="https://user-images.githubusercontent.com/31934929/189780698-2eb07afe-779c-4941-b935-2155216594bb.png">


Sub classes are then created for each individual controller: 
* DroneEnvironment_BLQR.m: Implementation of Bryson's rule.
* DroneEnvironment_LQR.m: Implementation of Vanilla LQR.
* DroneEnvironment_BLQRPD.m: Implementation of Bryson's rule with LQR-PD.
* DroneEnvironment_RL.m: Implementation of RL agent.
* DroneEnvironment_LQRFsolve.m: Implementation of LM algorithm.

Files used to create RL agent:
* critic.m: implements the critic
* actor.m: implements the actor
* layer.m: defines a fully connected layer used in actor-critic agent.
* agent_Layer.m: compbines the actor and critic.

Fsolve:
* fsolve_optimization_v2.m: finds the optimal Q matrix using LM algo.

Result scripts:
* get_ISE.m: gets the ISE error and maximum rotation speed of each rotor
* predict.class.m: used to create trajectory plots for all five controllers.
* get_RT_ST.m: gets the rise and settling times.
