% all 4 on same plot
% make sure you start from velocities at previous waypoints
% go to around 40 m
% keep everything positive
% mention units

classdef DroneEnvironment_BaseClass < rl.env.MATLABEnvironment
    %DRONEENVIRONMENT: Template for defining custom environment in MATLAB.
    % This class includes properties and methods necessary to simulate the
    % drone.
    
    %% Properties (set properties' attributes accordingly)
    properties

        % parameters for simulation
        params = sys_params;
        Q_value_threshold = 1e-3; % lowest allowd value in Q matrix
        pos_threshold = 1e-4; % threhshold to check stopping condition
        vel_threshold = 1e-4;% threhshold to check stopping condition
    end
    
    properties
        % Initialize system state [x,dx,theta,dtheta]'
        % Initial system state.
        state = zeros(12,1); % drone state
        prev_state = zeros(12,1); % previous drone state
        state_des = zeros(12,1); % desired goal
        State = zeros(12,1) % goal + drone state ( input for actor and critic networks )
        waypoints = zeros(3,40); % x,y,z waypoint positions
        waypoint_reached = false; % boolean to check if next waypoitn is reached
        episode_steps = 0; % number of steps in current episode. Used to terminate episode
        waypoint_counter = 2; % next waypoint index
        state_actual = zeros(12,1)
        mode;
        
    end
    
    properties(Access = protected)
        % Initialize internal flag to indicate episode termination
        IsDone = false        
    end

    %% Necessary Methods
    methods              
        % Contructor method creates an instance of the environment
        % Change class name and constructor name accordingly
        function this = DroneEnvironment_BaseClass(mode)
            
            % Initialize Observation settings
            ObservationInfo = rlNumericSpec([12 1]);
            ObservationInfo.Name = 'Drone States';
            ObservationInfo.Description = 'state of drone, desired state of drone';
            
            % Initialize Action settings
            ActionInfo = rlNumericSpec([4 1],'LowerLimit',1e-2,'UpperLimit',1e2); % values in Q matrix
            ActionInfo.Name = 'Q matrix values: penalty on states';
             
            % The following line implements built-in functions of RL env
            this = this@rl.env.MATLABEnvironment(ObservationInfo,ActionInfo);

            this.mode = mode;
%             disp(this.params);
            
            % use random seed 0 to train
            if strcmp(mode,'GetNormfactors')
                rng(2)
            end

            if strcmp(mode,'train')
                rng(0)
            end
            
            if strcmp(mode,'test')
                rng(1)
            end

            this.params.waypoints_total = 10;
            disp("total waypoints")
            disp(this.params.waypoints_total)
        end
    
        % Reset environment to initial state and output initial observation
        function InitialObservation = reset(this)

            this.waypoint_counter = 2;
            this.episode_steps = 0;

            % random initial state
            [wps,rstate] = this.randomState();

            % store all waypoints
            this.waypoints = wps;
            this.state = [wps(:,1);rstate(1:end)];
            
            % random goal
            % random position
            rpsi_des = 0;
            this.state_des = [wps(1:3,2);0;0;rpsi_des;0;0;0;0;0;0];

            this.State = this.state;
            this.State(1:3) = this.State(1:3) - this.state_des(1:3);
            this.State(6) = this.State(6) - this.state_des(6);
          
            InitialObservation= this.State;

            % (optional) use notifyEnvUpdated to signal that the 
            % environment has been updated (e.g. to update visualization)
            notifyEnvUpdated(this); 
        end

        % function to generate random state
        function [wps, rstate] = randomState(this)

            % generate waypoints
            wps = zeros(3,this.params.waypoints_total);
            % start position should be zero
            % set inital positoin

%             if this.params.episodenum >1
%                 wps(:,1) = 10*rand(3,1) - 5;
%             end
%             
%             for wpc=2:this.params.waypoints_total
%                 wps(:,wpc) = wps(:,wpc-1) + 2*rand(3,1) - 1; % each waypoint within -1 to +1
%             end

            if this.params.episodenum > 1
                wps(:,1) = 1*rand(3,1);
            end
            
            for wpc=2:this.params.waypoints_total
                upper_bound = 1;
%                 lower_bound = max(-sum(wps(:,1:wpc-1),2) + 0.1,-upper_bound);% zeros(3)
                lower_bound = zeros(3);
                
%                 disp(lower_bound(1));
                rand_num_x = rand()*(upper_bound-lower_bound(1))+lower_bound(1);
                rand_num_y = rand()*(upper_bound-lower_bound(2))+lower_bound(2);
                rand_num_z = rand()*(upper_bound-lower_bound(3))+lower_bound(3);
%                 disp(rand_num_x);

                wps(:,wpc) = [rand_num_x;rand_num_y;rand_num_z]; % each waypoint within -1 to +1
            end

%             disp(cumsum(wps,2));

            % initialise initial angular positions
            rphi = 0;%2*pi*rand - pi;
            rtheta = 0;%2*pi*rand - pi;
            rpsi = 0;%pi*rand;

            % initialise initial velocities
            rxdot =0; %1* rand - 0.5;
            rydot = 0;%1* rand - 0.5;
            rzdot = 0;%1* rand - 0.5;

            % initialise initial angular velocities
            rphidot = 0;%0.1* rand - 0.05;
            rthetadot = 0;%0.1* rand - 0.05;
            rpsidot = 0;%0.1* rand - 0.05;

            this.params.episodenum = this.params.episodenum + 1;

            rstate = [rphi;rtheta;rpsi;rxdot;rydot;rzdot;rphidot;rthetadot;rpsidot];
        end

        % Reward function
        function Reward = getReward(this,condition_div)
            if ~this.IsDone
                if this.waypoint_reached == true
                    Reward = 600;
                    disp("collecting reward")
                else
                    Reward = -sum(abs(this.State([1:3 6])));
%                     dist_now = norm(this.state(1:3)-this.state_des(1:3));
%                     dist_prev = norm(this.prev_state(1:3)-this.state_des(1:3));
%                     Reward = dist_prev-dist_now;
                end
                                
                % reward based on projection
%                 disp(this.prev_state(1:3))
%                 disp(this.state(1:3))
%                 disp(this.state_des(1:3))
%                 dist_now = norm(this.state(1:3)-this.state_des(1:3));
%                 dist_prev = norm(this.prev_state(1:3)-this.state_des(1:3));
%                 Reward = dist_prev-dist_now;
            else
                disp("agent reached set point at episode end: ")
                disp(this.waypoint_counter)
                if condition_div
                    disp(sum(abs(this.State)))
                    Reward = 0;%-300*this.params.waypoints_total;
                else
                    Reward = 600;
                end
            end
        end

    end

    %% Optional Methods (set methods' attributes accordingly)
    methods
        % Helper methods to create the environment

        % Compute acceleration in inertial reference frame
        % Parameters:
        %   g: gravity acceleration
        %   m: mass of quadcopter
        %   k: thrust coefficient
        %   kd: global drag coefficient
        function a = acceleration(this,inputs, angles, vels, m, g, kd)
            gravity = [0; 0; -g];
             
            R = this.rotation(angles);
            % thrust in body frame
            thrust_ = this.thrust(inputs);
            % rotate from body frame to earth frame
            T = R * thrust_;
        %     Fd = -kd * vels;
            a = gravity + 1 / m * T ;%+ Fd;
        end

        % Compute angular acceleration in body frame
        % Parameters:
        %   I: inertia matrix
        function omegad = angular_acceleration(this,inputs, omega, I)
            tau = this.torques(inputs);
            omegad = inv(I) * (tau - cross(omega, I * omega));
        end  
        
        % (optional) Visualization method
        function plot(this)
            % Initiate the visualization
            
            % Update the visualization
            envUpdatedCallback(this)
        end
    end


    methods(Static)

        function theta = bound_angle(theta)
            for i=1:size(theta)
                if theta(i)>2*pi
                    theta(i) = mod(theta(i),2*pi);
                end
                if theta(i)<-2*pi
                    theta(i) = mod(theta(i),-2*pi);
                end
            end
        end

        % Compute thrust given current inputs and thrust coefficient.
        function T = thrust(inputs)
            T = [0; 0; inputs(1)];
        end
        
        % Compute torques, given current inputs, length, drag coefficient, and thrust coefficient.
        function tau = torques(inputs)
            tau = [
                inputs(2)
                inputs(3)
                inputs(4)
            ];
        end

        % Convert derivatives of roll, pitch, yaw to omega.
        function omega = thetadot2omega(thetadot, angles)
            phi = angles(1);
            theta = angles(2);
            psi = angles(3);
            W = [
                1, sin(phi)*tan(theta), cos(phi)*tan(theta)
                0, cos(phi), -sin(phi)
                0, sin(phi)/cos(theta), cos(phi)/cos(theta)
            ];
            omega = inv(W) * thetadot;
        end
        
        % Convert omega to roll, pitch, yaw derivatives
        function thetadot = omega2thetadot(omega, angles)
            phi = angles(1);
            theta = angles(2);
            psi = angles(3);
            W = [
                1, sin(phi)*tan(theta), cos(phi)*tan(theta)
                0, cos(phi), -sin(phi)
                0, sin(phi)/cos(theta), cos(phi)/cos(theta)
            ];
            thetadot = W * omega;
        end

        % Compute rotation matrix for a set of angles.
        function R = rotation(angles)
            phi = angles(1);
            theta = angles(2);
            psi = angles(3);
        
            R = zeros(3);
            R(:, 1) = [
                cos(psi) * cos(theta)
                sin(psi) * cos(theta)
                - sin(theta)
            ];
            R(:, 2) = [
                cos(psi) * sin(theta) * sin(phi) - cos(phi) * sin(psi)
                sin(psi) * sin(theta) * sin(phi) + cos(phi) * cos(psi)
                cos(theta) * sin(phi)
            ];
            R(:, 3) = [
                cos(psi) * sin(theta) * cos(phi) + sin(psi) * sin(phi)
                cos(phi) * sin(psi) * sin(theta) - cos(psi) * sin(phi)
                cos(theta) * cos(phi)
            ];
        end
    end
end
