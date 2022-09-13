classdef DroneEnvironment_BLQRPD < DroneEnvironment_BaseClass
    %This class implements the step method for LQR with Bryson's rule to
    %set Q matrix. It also includes a PD control.

    properties
        state_storage = zeros(12,1)
        max_omega2 = zeros(4,1)
    end

    
    methods
        function s = DroneEnvironment_BLQRPD(mode)
         % Call asset and member class constructors
         s@DroneEnvironment_BaseClass(mode)
        end

        % step function
        function [Observation,Reward,IsDone,LoggedSignals] = step(this)

            LoggedSignals = [];

            this.episode_steps = this.episode_steps + 1;
            this.prev_state = this.state;
            
            % calculate input using LQR
            [i] = LQR_controller(this);

            % unpack state values
            thetadot = this.state(10:12);
            theta = this.state(4:6);
            x = this.state(1:3);
            xdot = this.state(7:9);
            
            % Apply motion equations            
            % Compute forces, torques, and accelerations.
            omega = this.thetadot2omega(thetadot, theta);

            a = acceleration(this,i, theta, xdot, this.params.m, this.params.g, this.params.kd);
            omegadot = angular_acceleration(this, i, omega, this.params.I);

            % Euler integration
            % Advance system state and update states
            omega = omega + this.params.dt * omegadot;
            thetadot = this.omega2thetadot(omega, theta); 
            theta = theta + this.params.dt * thetadot;

            % bound theta between -2pi and 2pi
            theta = this.bound_angle(theta);

            xdot = xdot + this.params.dt * a;
            x = x + this.params.dt * xdot;
            this.state = [x; theta; xdot; thetadot];
            
            % store observation
            this.State = this.state;
            this.State(1:3) = this.State(1:3) - this.state_des(1:3);
            this.State(6) = this.State(6) - this.state_des(6);

            this.state_actual(:,end+1) = this.state;

            % check conditions before normalizing
            % Check terminal condition
            pos_condition = sum(abs(this.state(1:6)-this.state_des(1:6))) < this.pos_threshold;
            vel_condition = (sum(abs(this.state(7:12)-this.state_des(7:12))) < this.vel_threshold);
            % condition for diverging controller
            condition_div = sum(abs(this.State)) > 1000;

            %  if we are testing, run till the last iteration
            if this.mode ~= "jumbo"
                if this.waypoint_counter == size(this.waypoints,2)
                    
                    IsDone = (pos_condition && vel_condition) || condition_div ;
    
                    if IsDone
                        fprintf('final waypoint reached at %f, %f, %f, %f \n',this.state(1),this.state(2),this.state(3),this.state(6));
                    end
    
                end
            else
                IsDone = false;
            end
            
            this.waypoint_reached = false;
            if this.waypoint_counter < size(this.waypoints,2)
                % if the drone has reached desired location which is not
                % final goal
                if sum(abs(this.State([1:3 6]))) < 1e-2

                    this.waypoint_reached = true;

                    fprintf('waypoint %d reached!, location reached x: %f y: %f z: %f yaw: %f \n',this.waypoint_counter,this.state(1),this.state(2),this.state(3),this.state(6));
                    % update desired position
                    next_target = sum(this.waypoints(:,1:this.waypoint_counter+1),2);
                    rpsi_des = 0;
                    this.state_des = [next_target;0;0;rpsi_des;0;0;0;0;0;0];

                    %update State variable : state - desired
                    this.State = this.state;
                    this.State(1:3) = this.State(1:3) - this.state_des(1:3);
                    this.State(6) = this.State(6) - this.state_des(6);

                    this.waypoint_counter = this.waypoint_counter + 1;

                    fprintf('trying to reach waypoint %d at location x: %f y: %f z: %f yaw: %f \n',this.waypoint_counter,this.state_des(1),this.state_des(2),this.state_des(3),this.state_des(6));
                end

                IsDone = condition_div ;

            end
            
            IsDone = IsDone || (this.episode_steps >(this.params.waypoints_total-1)*2000);
            this.IsDone = IsDone;
            Observation= this.State;

            % Get reward
            Reward = getReward(this,condition_div);
            
            % (optional) use notifyEnvUpdated to signal that the 
            % environment has been updated (e.g. to update visualization)
            notifyEnvUpdated(this);
        end
        
        % function that implements LQR controller
        function [U] = LQR_controller(this)

            %%%%%%% Define state space matrix, x_dot = Ax + Bu %%%%%%%%%%%%%%%%%%
            
            % Linearized at hover state
        
            A = [0 0 0 0 0 0 1 0 0 0 0 0;
                 0 0 0 0 0 0 0 1 0 0 0 0;
                 0 0 0 0 0 0 0 0 1 0 0 0;
                 0 0 0 0 0 0 0 0 0 1 0 0;
                 0 0 0 0 0 0 0 0 0 0 1 0;
                 0 0 0 0 0 0 0 0 0 0 0 1;
                 0 0 0 0 this.params.g 0 0 0 0 0 0 0;
                 0 0 0 -this.params.g 0 0 0 0 0 0 0 0;
                 0 0 0 0 0 0 0 0 0 0 0 0;
                 0 0 0 0 0 0 0 0 0 0 0 0;
                 0 0 0 0 0 0 0 0 0 0 0 0;
                 0 0 0 0 0 0 0 0 0 0 0 0];
            
            B = [0 0 0 0;
                 0 0 0 0;
                 0 0 0 0;
                 0 0 0 0;
                 0 0 0 0;
                 0 0 0 0;
                 0 0 0 0;
                 0 0 0 0;
                 1/this.params.m 0 0 0;
                 0 1/this.params.I(1,1) 0 0;
                 0 0 1/this.params.I(2,2) 0;
                0 0 0 1/this.params.I(3,3)];

            Td = 3;
            Kp = 3;
        
            PD = [ Kp 0 0 0 0 0 Td 0 0 0 0 0;
                   0 Kp 0 0 0 0 0 Td 0 0 0 0;
                   0 0 Kp 0 0 0 0 0 Td 0 0 0;
                   0 0 0 0 0 Kp 0 0 0 0 0 Td];

            Ac = A-B*PD;

            % variable that stores all state values
            this.state_storage(:,end+1) = this.state - this.state_des;
            
            % check if we have more than 100 values and take rolling window
            if size(this.state_storage,2)>100
                storage_rolling= this.state_storage(:,end-100:end);
            else
                storage_rolling = this.state_storage;
            end
            % maximum square of state error.
            max_state_sq = max(storage_rolling.^2,[],2);
            max_state_sq(max_state_sq<1e-6) = 1;

            max_state_sq(4:end) = 1;
            max_state_sq(7:end) = 1;
            
        
            Q = eye(12,12)./(max_state_sq);
        
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
            % Get K matrix using LQR function
            R = eye(4,4);
            [K,~,~] = lqr(Ac,B,Q,R);

            % desired control (hover state)
            u_des = [this.params.m*this.params.g ; 0; 0; 0];

            xe = this.state - this.state_des;
            U = -K*xe + u_des - PD*xe;

            k = this.params.d/this.params.b;
            true_inputs_mat = [1 1 1 1;
                               0 -this.params.l 0 this.params.l;
                               -this.params.l 0 this.params.l 0;
                               -k k -k k];
            omega2 = (1/this.params.b).* inv(true_inputs_mat)*U;

            for omega2_itr=1:size(omega2,1)
                if omega2(omega2_itr) > this.max_omega2(omega2_itr)
                    this.max_omega2(omega2_itr) = omega2(omega2_itr);
                end
            end
        
        end
    end
end

