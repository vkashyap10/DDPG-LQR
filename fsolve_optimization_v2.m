params = sys_params;
mean_norm = abs(params.mean_norm);
% disp(mean_norm)
x0 = [1/mean_norm(1); 1/mean_norm(2); 1/mean_norm(3); 1/mean_norm(6)];
disp("initial value")
disp(x0)

options = optimoptions('fsolve','Display','iter','MaxFunctionEvaluations',numepisodes);

fun = @(x)fsolve_opt(x);
x = fsolve(fun,x0);

function score_all = fsolve_opt(x)

    environment = DroneEnvironment_fsolve_LQR('train');
    numepisodes = 20;
    disp("running fsolve function with value")
    disp(x);
    score = zeros(4,1);
    dt = 0.005; % time step

    diag_Q = diag(ones(12,1));
    diag_Q(1,1) = x(1);
    diag_Q(2,2) = x(2);
    diag_Q(3,3) = x(3);
    diag_Q(6,6) = x(4);
    environment.diag_Q = diag_Q;

    try chol(diag_Q)
        disp('Matrix is symmetric positive definite.')
        % predict using LQR
        for i=1:numepisodes
            disp("episode number")
            disp(i)
            IsDone = false;
            num_itr = 0;
            observations = environment.reset();
            disp("initial start state")
            disp(observations(1:3))
        
            while IsDone==false && num_itr < 2000
                [Observation,~,IsDone,~] = environment.step();
                score = score - (abs(Observation([1:3 6])).^2).*dt;
                num_itr = num_itr + 1;
            end
            disp("total number of iterations in episode")
            disp(num_itr)
        end
    catch ME
        disp('Matrix is not symmetric positive definite')
        score = -ones(4,1).*(10/4);
    end
        

    score_all = sum(score);
    disp("score in this iteration")
    disp(score_all)


    
end

