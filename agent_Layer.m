classdef agent_Layer
    % combines the actor critic to create actor-critic model

    properties

        actorOptions; % used to create DDPG agent
        criticOptions; % used to create DDPG agent
        critic_learning_rate;
        actor_learning_rate;
        
        % Parameter determines degree of noise added to actions during training
        noise_perc;
        DDPGagent;
        Actor;
        Critic;
        params = sys_params;
        
    end
    
    methods
        function obj = agent_Layer()
            
            % actor and critic options
            obj.criticOptions = rlOptimizerOptions('LearnRate',obj.params.critic_learning_rate,'GradientThreshold',obj.params.critic_GradientThreshold);
            obj.actorOptions = rlOptimizerOptions('LearnRate',obj.params.actor_learning_rate,'GradientThreshold',obj.params.actor_GradientThreshold);

            % DDPG agent
            agentOptions = rlDDPGAgentOptions(...
                'SampleTime',obj.params.dt,...
                'ActorOptimizerOptions',obj.actorOptions,...
                'CriticOptimizerOptions',obj.criticOptions,...
                'ExperienceBufferLength',1e6,...
                'MiniBatchSize',256);
            agentOptions.NoiseOptions.StandardDeviation = obj.params.actorNoiseSD;
            agentOptions.NoiseOptions.StandardDeviationDecayRate = obj.params.actorSDDecayRate;
            
            obj.Actor = actor(1).Actor;
            obj.Critic = critic(1).Critic;
            obj.DDPGagent = rlDDPGAgent(obj.Actor,obj.Critic,agentOptions);

        end
        
    end
end

