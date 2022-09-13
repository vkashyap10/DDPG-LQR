% define critic of the actor-critic architecture
classdef critic
    properties
        critic_name;
        layer_number; % number of layers, 1 for no hierarchical
        goal_dim;  % dimension of goal
        params = sys_params
        layer_action_dim;
        goal_ph;
        state_ph;
        features_ph;
        action_ph;
        criticNetwork;
        action_space_upper_limit = 1e2;
        action_space_lower_limit = 1e-2;
        Critic; % actor defined using rlContinuousDeterministicActor()
    end
    
    methods
        % constructor method to initialise objects
        function obj = critic(layer_number)
            obj.layer_number = layer_number;
            obj.critic_name = 'critic_' + string(layer_number);
            obj.params.state_dim = 12; % x,y,z,phi,theta, psi
         
            % Dimensions of action and goal will depend on layer level
            if obj.layer_number == 1
                obj.layer_action_dim = obj.params.action_dim; % action dimensinon
%                 obj.goal_dim = 12; % x,y,z,phi,theta, psi
            else
                obj.layer_action_dim = obj.params.action_dim; % subgoal action dimension
%                 obj.goal_dim = 3; % depending on subgoal
            end
            
            % initialise a goal vector
%             obj.goal_ph = zeros(obj.goal_dim,1);
            obj.state_ph = zeros(obj.params.state_dim,1);
            obj.action_ph = zeros(obj.layer_action_dim,1);

            % initialise feature to be fed into actor neural network
            obj.features_ph = [obj.state_ph;obj.action_ph];

            % define observation and action info for actor network
            observationInfo = rlNumericSpec([size(obj.features_ph,1)-obj.layer_action_dim 1]); % dimension of features
            actionInfo = rlNumericSpec([obj.layer_action_dim 1],'LowerLimit',obj.action_space_lower_limit,'UpperLimit',obj.action_space_upper_limit); % action space

            % initialise neural network for actor
            obj.criticNetwork = create_nn(obj,obj.features_ph);
            obj.criticNetwork = dlnetwork(obj.criticNetwork);
            figure
            plot(layerGraph(obj.criticNetwork))
            obj.Critic = rlQValueFunction(obj.criticNetwork,observationInfo,actionInfo);

        end

        % function to define neural network to learn policy
        function criticNetwork = create_nn(obj,features) 
            name = obj.critic_name;
            input_size = size(features,1) - obj.layer_action_dim;
            input_layer_obs = featureInputLayer(input_size,'Normalization','none','Name','input_state_and_goal');
            input_sLayer = scalingLayer('Scale',1./obj.params.std_norm,'Bias',0,'Name','scale_input');
            concat_layer = concatenationLayer(1,2,'Name','concat');


            [fc1,output_size_fc1] = layer(input_size+obj.layer_action_dim,64,name+'fc_1',false);
            [fc2,output_size_fc2] = layer(output_size_fc1,64,name+'fc_2',false);
            [fc3,output_size_fc3] = layer(output_size_fc2,64,name+'fc_3',false);
            [fc4,~] = layer(output_size_fc3,1,name+'fc_4',true);
            % remember to add q_offset, TODO
            % A q_offset is used to give the critic function an optimistic initialization near

%             statePath = cat(2,input_layer_obs,concat_layer,fc1,fc2,fc3,fc4,sigmoidLayer('Name','sigmoid'));
            
            sLayer = scalingLayer('Scale',obj.params.q_limit,'Bias',obj.params.q_limit_bias,'Name','scale_critic');
            statePath = cat(2,input_layer_obs,input_sLayer,concat_layer,fc1,fc2,fc3,fc4,tanhLayer('Name','tanh'),sLayer);
%             statePath = cat(2,input_layer_obs,concat_layer,fc1,tanhLayer('Name','tanh'),sLayer);

            actionPath = [featureInputLayer(obj.layer_action_dim,'Normalization','none','Name','action'),scalingLayer('Scale',1e-3,'Bias',0,'Name','scale_action')];

            criticNetwork = layerGraph(statePath);
            criticNetwork = addLayers(criticNetwork,actionPath);    
            criticNetwork = connectLayers(criticNetwork,'scale_action','concat/in2');

        end
    end
end


