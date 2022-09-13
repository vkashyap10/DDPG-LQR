% define actor of the actor-critic architecture
classdef actor
    properties
        layer_number; % number of layers, 1 for no hierarchical
        actor_name;
        layer_action_dim ;
        goal_dim;  % dimension of goal
        params = sys_params;
        goal_ph;
        state_ph;
        features_ph;
        action_space_upper_limit = 1e2;
        action_space_lower_limit = 1e-2;
        scaling_bias = 1e-2;
        Actor; % actor defined using rlContinuousDeterministicActor()
    end
    
    methods
        % constructor method to initialise objects
        function obj = actor(layer_number)
            obj.layer_number = layer_number;
            obj.actor_name = 'actor_' + string(layer_number);
%             obj.params.state_dim = 12; % x,y,z,phi,theta, psi
         
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

            % initialise feature to be fed into actor neural network
%             obj.features_ph = [obj.state_ph;obj.goal_ph];
            obj.features_ph = [obj.state_ph];

            % define observation and action info for actor network
            observationInfo = rlNumericSpec([size(obj.features_ph,1) 1]); % dimension of features
%             rlNumericSpec([12 1]);
            actionInfo = rlNumericSpec([obj.layer_action_dim 1],'LowerLimit',obj.action_space_lower_limit,'UpperLimit',obj.action_space_upper_limit); % action space

            % initialise neural network for actor
            actorNetwork = create_nn(obj, obj.features_ph);
            actorNetwork = dlnetwork(actorNetwork);
            figure
            plot(layerGraph(actorNetwork))
            obj.Actor = rlContinuousDeterministicActor(actorNetwork,observationInfo,actionInfo);

        end

        % function to define neural network to learn policy
        function actorNetwork = create_nn(obj,features)
            name = obj.actor_name;
            input_size = size(features,1);
            input_layer = featureInputLayer(input_size,'Normalization','none','Name','input_state_and_goal');
            input_sLayer = scalingLayer('Scale',1./obj.params.std_norm,'Bias',0,'Name','scale_input');
            [fc1,output_size_fc1] = layer(input_size,64,name+'fc_1',false);
            [fc2,output_size_fc2] = layer(output_size_fc1,64,name+'fc_2',false);
            [fc3,output_size_fc3] = layer(output_size_fc2,64,name+'fc_3',false);
            [fc4,~] = layer(output_size_fc3,obj.layer_action_dim,name+'fc_4',true);
            sLayer = scalingLayer('Scale',obj.action_space_upper_limit,'Bias',obj.scaling_bias,'Name','scale');
            actorNetwork = cat(2,input_layer,input_sLayer,fc1,fc2,fc3,fc4,sigmoidLayer,sLayer);

        end
      
    end

end


