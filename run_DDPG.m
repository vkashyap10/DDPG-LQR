% set training options and train RL agent to predict Q matrix values
function [trainingStats,DDPG_agent] = run_DDPG()
    params = sys_params;
    Ts = params.dt; % time step
    Tf = params.Tf; % simulation time
    maxepisodes = params.maxepisodes;
    waypoints_total = params.waypoints_total;
    maxsteps = ceil(Tf/Ts)*waypoints_total;
%     maxsteps = params.maxitrs;

    % initialise the environment.
    env = DroneEnvironment_RL('train');

    DDPG_agent = agent_Layer().DDPGagent;

    trainingOpts = rlTrainingOptions(...
        'MaxEpisodes',maxepisodes,...
        'MaxStepsPerEpisode',maxsteps,...
        'Verbose',true,...
        'Plots','training-progress',...
        'StopTrainingCriteria','EpisodeReward',...
        'StopTrainingValue',params.StopTrainingValue,...
        'SaveAgentCriteria','EpisodeReward',...
        'SaveAgentValue',params.SaveAgentValue);
    trainingOpts.ScoreAveragingWindowLength = 100;

    trainingStats = train(DDPG_agent,env,trainingOpts);

end 