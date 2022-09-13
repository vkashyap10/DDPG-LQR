% run over test samples to generate ISE values for given environment and
% method
function [score,average_num_steps,max_omega2] = get_ISE(environment,method)

% DroneEnvironment_LQR('test');
% DroneEnvironment_BLQR('test');
% DroneEnvironment_BLQRPD('test');
% DroneEnvironment_RL('test')
% DroneEnvironment_LQRFsolve('test')


    % add path
    numepisodes = 20;
    addpath('savedAgents');
    score = zeros(4,1);
    average_num_steps = 0;
    dt = 0.005; % time step

    if method ~= "RL"
        % predict using LQR
        for i=1:numepisodes
            IsDone = false;
            num_itr = 0;
            observations = environment.reset();

            while IsDone==false && num_itr < 2000*9
                [Observation,~,IsDone,~] = environment.step();
                score = score - (abs(Observation([1:3 6])).^2).*dt;
                num_itr = num_itr + 1;
            end

            average_num_steps = average_num_steps + num_itr;
        end
    else
        % Load all environments
        % load saved model
        simOpts = rlSimulationOptions('MaxSteps',2000*9,'NumSimulations',numepisodes);    
        load traj_without_next_state_100itr.mat;
        xpr = sim(environment,saved_agent,simOpts);
        for i=1:numepisodes
            % load first simulation
            simdata = xpr(i).Observation.obs1.Data;
            sum_curr = sum(squeeze(abs(simdata).^2),2);
            score = score - sum_curr([1:3 6]).*dt;
            average_num_steps = average_num_steps + size(simdata,3);
        end
    end

    average_num_steps = average_num_steps/numepisodes*dt;
    max_omega2 = environment.max_omega2;
    
end