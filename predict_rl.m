% plot test trajectory for rl

function predict_rl()
    % function samples 1000 random test cases and calculates average ISE

    % add path
    addpath('savedAgents');

    % load saved model
    simOpts = rlSimulationOptions(...
    'MaxSteps',20000,...
    'NumSimulations',1);

    load traj_without_nextstate_50itr.mat;
    env_init = DroneEnvironment_RL('test');
    xpr = sim(env_init,saved_agent,simOpts);

    % modify x,y and z
    x = env_init.state_actual(1,:).';
    y = env_init.state_actual(2,:).';
    z = env_init.state_actual(3,:).';
    psi = env_init.state_actual(6,:).';
    
    ts = squeeze((xpr(1).Observation.obs1.Time));

    figure;
    subplot(4,1,1);
    plot(ts,x);
    title('x position comparison - RL agent')
    xlabel('time') 
    ylabel('x')
    
    subplot(4,1,2);
    plot(ts,y);
    title('y position comparison- RL agent')
    xlabel('time') 
    ylabel('y')
    
    subplot(4,1,3);
    plot(ts,z);
    title('z position comparison- RL agent')
    xlabel('time') 
    ylabel('z')
    
    subplot(4,1,4);
    plot(ts,psi);
    title('psi comparison- RL agent')
    xlabel('time') 
    ylabel('psi')


end