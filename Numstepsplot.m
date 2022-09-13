figure
addpath("savedAgents")
load('100_minus4_state_norm.mat')
x = linspace(1,100,100);
line_width = 1;
plot(x,savedAgentResult.EpisodeSteps,'LineWidth',line_width)
grid on
title('Number of steps taken to reach set point while training (RL agent)')
xlabel('Episode number')
ylabel('Number of steps')

% set_point_100_itr
% traj_100itr_result

% reward

figure

load('100_minus4_state_norm.mat')
x = linspace(1,100,100);
line_width = 1;
epreward = savedAgentResult.EpisodeReward;
avg_reward = cumsum(epreward)./(x.');
plot(x,savedAgentResult.EpisodeReward,'DisplayName','Episode reward','LineWidth',line_width)
hold on
plot(x,avg_reward.','--','DisplayName','Average reward','LineWidth',line_width)
grid on
title('Reward collected by RL agent (training)')
xlabel('Episode number')
ylabel('Reward')
legend show