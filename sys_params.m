
function params = sys_params()
% this file stores the simulation parameters

% Parameters for dynamic system
% Physical constants.
g = 9.81;
m = 0.8;
I = diag([5.17e-3, 5.17e-3, 1.7e-2]);
kd = 0.25;
l = 0.3;
b = 2e-4;
d = 7e-5;

% Simulation parameters
dt = 0.005; % time step
Tf = 10; % total simulation time
maxepisodes = 100; % total episodes to train
episodenum = 1;
% maxitrs = 100;

% RL agent parameters
% Initialize Observation settings required for Drone Environment
state_dim = 12; % Dimension of state vector
action_dim = 4; % Dimension of actor vector

% actor critic learning parameters agent_Layer class
critic_learning_rate = 1e-4;
actor_learning_rate = 1e-4;
actor_GradientThreshold = 1;
critic_GradientThreshold = 1;

q_limit = 1000;
q_limit_bias = 0;

%For continuous action signals, it is important to set the noise standard deviation appropriately to encourage exploration. It is common to set StandardDeviation*sqrt(Ts) to a value between 1% and 10% of your action range.
%If your agent converges on local optima too quickly, promote agent exploration by increasing the amount of noise; that is, by increasing the standard deviation. Also, to increase exploration, you can reduce the StandardDeviationDecayRate.
actorNoiseSD = ones(4,1)*15;
actorSDDecayRate = 1e-4;

% DDPG agent training options
SaveAgentValue = -1200; % save agent to disk if this reward achieved
StopTrainingValue = 5000; % end episode if this reward achieved

% mean and std norm over 100 simulations to normalize data
mean_norm = load("mean_100_sim.mat").mean;
std_norm = load("dev_100_sim.mat").dev;

% maximum number of wapoints
waypoints_total = 10;

params.g = g;
params.m = m;
params.I = I;
params.kd = kd;
params.dt = dt;
params.state_dim = state_dim;
params.action_dim = action_dim;
params.critic_learning_rate=critic_learning_rate;
params.actor_learning_rate = actor_learning_rate;
params.actor_GradientThreshold = actor_GradientThreshold;
params.critic_GradientThreshold = critic_GradientThreshold;
params.SaveAgentValue = SaveAgentValue;
params.StopTrainingValue = StopTrainingValue;
params.actorSDDecayRate = actorSDDecayRate;
params.actorNoiseSD = actorNoiseSD;
params.maxepisodes = maxepisodes;
params.Tf = Tf;
params.std_norm = std_norm;
params.mean_norm = mean_norm;
params.waypoints_total = waypoints_total;
params.q_limit = q_limit;
params.q_limit_bias = q_limit_bias;
params.episodenum = episodenum;
params.l = l;
params.b = b;
params.d = d;
% params.maxitrs = maxitrs;

end

