% environment = DroneEnvironment_fsolve_LQR('train');
numepisodes = 400;
x0 = [1; 1; 1; 1];
options = optimoptions('fsolve','Display','iter','MaxFunctionEvaluations',numepisodes,'MaxIterations',400);

fun = @(x)fsolve_opt(x);
x = fsolve(fun,x0);

function score_all = fsolve_opt(x)
    disp("eval function")
    score_all = sum(x.*x);
    disp("score")
    disp(score_all)
end

