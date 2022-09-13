% run over test samples to generate ISE values for given environment and
% method
function [itr_10per,itr_90per,itr_98per] = get_RT_ST(environment,method)

% DroneEnvironment_LQR('test');
% DroneEnvironment_BLQR('test');
% DroneEnvironment_BLQRPD('test');
% DroneEnvironment_RL('test')
% DroneEnvironment_LQRFsolve('test')

    % add path
    numepisodes = 50;
    addpath('savedAgents');
    itr_10per = zeros(4,numepisodes);
    itr_90per = zeros(4,numepisodes);
    itr_98per = zeros(4,numepisodes);

    if method ~= "RL"
        % predict using LQR
        for i=1:numepisodes
            IsDone = false;
            num_itr = 0;
            observations = environment.reset();
            initial_value = zeros(3,1);

            while IsDone==false && num_itr < 2000
                [Observation,~,IsDone,~] = environment.step();

                % store inital value to calculate rise time and settling
                % time
                if num_itr == 0
                    initial_value  = Observation(1:3);
                    disp(Observation(1:3))
                end
                
                num_itr = num_itr + 1;

                % check if 10 per cent reached
                diff = (initial_value - Observation(1:3))./(initial_value);

                for k=1:3
                    if itr_10per(k,i) == 0 && (diff(k) > 0.1)
                        itr_10per(k,i) = num_itr;
                    end
                    if itr_90per(k,i) == 0 && (diff(k) > 0.9)
                        itr_90per(k,i) = num_itr;
                    end

                    if ((itr_98per(k,i) == 0) && (abs(diff(k)) >= 0.9800 && abs(diff(k)) <= 1.0200))
                        itr_98per(k,i) = num_itr;
                    else
                        if ~(abs(diff(k)) >= 0.9800 && abs(diff(k)) <= 1.0200)
                            itr_98per(k,i) = 0;
                        end
                    end
                end
            end
        end
    else
        % Load all environments
        % load saved model
        simOpts = rlSimulationOptions('MaxSteps',2000,'NumSimulations',numepisodes);    
        load traj_without_next_state_100itr.mat;
        xpr = sim(environment,saved_agent,simOpts);
        for i=1:numepisodes
            % load first simulation
            simdata = xpr(i).Observation.obs1.Data;

            num_itr = 0;
            initial_value = zeros(3,1);

            while num_itr < size(simdata,3) && num_itr < 2000

                % store inital value to calculate rise time and settling
                % time
                if num_itr == 0
                    initial_value  = simdata(1:3,1,1);
                end
                
                num_itr = num_itr + 1;

                % check if 10 per cent reached
                diff = (initial_value - simdata(1:3,1,num_itr))./(initial_value);
                for k=1:3
                    if itr_10per(k,i) == 0 && (diff(k) > 0.1)
                        itr_10per(k,i) = num_itr;
                    end
                    if itr_90per(k,i) == 0 && (diff(k) > 0.9)
                        itr_90per(k,i) = num_itr;
                    end
                    if ((itr_98per(k,i) == 0) && (abs(diff(k)) >= 0.9800 && abs(diff(k)) <= 1.0200))
                        itr_98per(k,i) = num_itr;
                    else
                        if ~(abs(diff(k)) >= 0.9800 && abs(diff(k)) <= 1.0200)
                            itr_98per(k,i) = 0;
                        end
                    end

                end
            end

        end
    end
    itr_10per = mean(itr_10per,2)*0.005;
    itr_90per = mean(itr_90per,2)*0.005;
    itr_98per = mean(itr_98per,2)*0.005;
end