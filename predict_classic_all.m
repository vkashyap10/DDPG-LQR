% plot test trajectory for lqr, blqr and blqrpd
% calculate ISE, average steps taken. For set point analysis also state
% rise time and setting time.
function predict_classic_all()
    maxstepsi = 60000;
%     maxstepsi = 40;
%     params = sys_params;
    drone_env_name = ["RL";"LQR";"BLQR";"BLQRPD";"LQRFsolve"];
    line_style = ["-";"--";"-.";":";"-"];
    line_width = [1;1;1;2;2.5];
    
    figure;

    for i=1:length(drone_env_name)

        IsDone = false;
        num_itr = 0;

        env_name = drone_env_name(i);
        if env_name == "LQR"
            drone_env = DroneEnvironment_LQR('test');
        elseif env_name == "BLQR"
            drone_env = DroneEnvironment_BLQR('test');
        elseif env_name == "BLQRPD"
            drone_env = DroneEnvironment_BLQRPD('test');
        elseif env_name == "LQRFsolve"
            drone_env = DroneEnvironment_LQRFsolve('test');
        elseif env_name == "RL"
            % add path
            addpath('savedAgents');
        
            % load saved model
            simOpts = rlSimulationOptions(...
            'MaxSteps',maxstepsi,...
            'NumSimulations',1);
        
            load set_point_100_itr_2.mat;
            drone_env = DroneEnvironment_RL('test');
        end
        
        if env_name ~= "RL"
            observations = drone_env.reset();
            disp(env_name);
    
            while IsDone==false && num_itr < maxstepsi
                [Observation,~,IsDone,~] = drone_env.step();
                observations(:,end+1) = Observation;
                num_itr = num_itr + 1;
            end
            
            ts = 0.005*(0:size(observations,2)-1);
        else
            xpr = sim(drone_env,saved_agent,simOpts);
            ts = squeeze((xpr(1).Observation.obs1.Time));
        end
        
%         line_width = 1;

        % modify x,y and z
        x = drone_env.state_actual(1,:).';
        y = drone_env.state_actual(2,:).';
        z = drone_env.state_actual(3,:).';
        psi = drone_env.state_actual(6,:).';
        
        
        disp(size(x))
        subplot(4,1,1);
        disp(line_style(i));
        plot(ts,x,line_style(i),'DisplayName',env_name,'LineWidth',line_width(i));
        grid on
        title('x position comparison')
        xlabel('time (seconds)') 
        ylabel('x (meters)')
        hold on
        legend show
        
        subplot(4,1,2);
        plot(ts,y,line_style(i),'DisplayName',env_name,'LineWidth',line_width(i));
        grid on
        title('y position comparison')
        xlabel('time (seconds)') 
        ylabel('y (meters)')
        hold on
        legend show
        
        subplot(4,1,3);
        plot(ts,z,line_style(i),'DisplayName',env_name,'LineWidth',line_width(i));
        grid on
        title('z position comparison')
        xlabel('time (seconds)') 
        ylabel('z (meters)')
        hold on
        legend show
        
        subplot(4,1,4);
        plot(ts,psi,line_style(i),'DisplayName',env_name,'LineWidth',line_width(i));
        grid on
        title('psi comparison')
        xlabel('time (seconds)') 
        ylabel('psi (radians)')
        hold on
        legend show
        
    end

    hold off
    
end