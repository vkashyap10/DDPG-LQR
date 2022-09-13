% plot test trajectory for lqr, blqr and blqrpd
function predict_classic(method)
    IsDone = false;
    num_itr = 0;
    if method == "LQR"
        drone_env = DroneEnvironment_LQR('test');
    elseif method == "BLQR"
        drone_env = DroneEnvironment_BLQR('test');
    elseif method == "BLQRPD"
        drone_env = DroneEnvironment_BLQRPD('test');
    end
    
    observations = drone_env.reset();
    
    while IsDone==false && num_itr < 20000
        [Observation,~,IsDone,~] = drone_env.step();
        observations(:,end+1) = Observation;
        num_itr = num_itr + 1;
    end
    
    ts = 0.005*(0:size(observations,2)-1);
    
    
    % modify x,y and z
    x = drone_env.state_actual(1,:).';
    y = drone_env.state_actual(2,:).';
    z = drone_env.state_actual(3,:).';
    psi = drone_env.state_actual(6,:).';
    
    
    figure;
    subplot(4,1,1);
    plot(ts,x);
    title('x position comparison')
    xlabel('time') 
    ylabel('x')
    
    subplot(4,1,2);
    plot(ts,y);
    title('y position comparison')
    xlabel('time') 
    ylabel('y')
    
    subplot(4,1,3);
    plot(ts,z);
    title('z position comparison')
    xlabel('time') 
    ylabel('z')
    
    subplot(4,1,4);
    plot(ts,psi);
    title('psi comparison')
    xlabel('time') 
    ylabel('psi')
end