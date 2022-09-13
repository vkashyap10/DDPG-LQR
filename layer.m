% implements a fully connected layer used in actor and critic networks.
function [layers,num_next_neurons] = layer(num_previous_neurons,num_next_neurons,layer_name,is_output)
    
%     if is_output == true
%         a = -3e3;
%         b = 3e3;
%     else
%         a = -1/(sqrt(num_previous_neurons));
%         b = 1/(sqrt(num_previous_neurons));
%     end
% 
%     % define fully connected layer
%     fc = fullyConnectedLayer(num_next_neurons,'Name',layer_name,...
%         'WeightsInitializer',@(sz) unifrnd(a,b,num_next_neurons,num_previous_neurons),...
%         'BiasInitializer',@(sz) unifrnd(a,b,num_next_neurons,1));

    % define fully connected layer
    fc = fullyConnectedLayer(num_next_neurons,'Name',layer_name);

    layers = [fc];
    
%     if is_output == true
%         layers = [fc];
%     else
%         layers = [fc reluLayer];
%     end
end