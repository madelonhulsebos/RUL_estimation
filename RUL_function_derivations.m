%% Derivation of system dynamics functions

load('RUL_data.mat')

%% Derivations of nonlinear state transition functions:

% rul_1 = f(sensor_1)
% rul_t = f(rul_t-1, sensor)

first_cycle = (sensordata_train(:,1)==1);
second_cycle = (sensordata_train(:,1)==2);
third_cycle = (sensordata_train(:,1)==3);
fourth_cycle = (sensordata_train(:,1)==4);

RUL_true = sensordata_train_RUL(first_cycle,1);
RUL_true_tminus1(1,1) = inf;
RUL_true_tminus1(2:size(RUL_true,1)+1,1) = RUL_true;
RUL_true_tminus1 = RUL_true_tminus1(1:size(RUL_true));
sensor_obs = sensordata_train(first_cycle,9);

%%For initial state estimation I fitted models to the data s.t. the RUL can
% be predicted from the observations of sensor 4 only.

[exp_fit, exp_gof, exp_output] = fit(sensordata_train(first_cycle,9), sensordata_train_RUL(first_cycle,1),'exp1');
figure; plot(sensordata_train(first_cycle,9), sensordata_train_RUL(first_cycle,1)); hold on;
plot(exp_fit);
exp_formula = formula(exp_fit);
exp_norm_resid = norm(exp_output.residuals);
% %1.0287e+40 * exp(-0.0623*sensor4)
% %This fitted curve results in a norm of the residuals of: 447.5295

% This function is used for the EKF implementation
[quad_fit, quad_gof, quad_output] = fit(sensordata_train(first_cycle,9), sensordata_train_RUL(first_cycle,1),'poly2');
figure; plot(sensordata_train(first_cycle,9), sensordata_train_RUL(first_cycle,1)); hold on;
plot(quad_fit);
quad_formula = formula(quad_fit);
quad_norm_resid = norm(quad_output.residuals);
% 1.6369e5 + (-226.5514) * sensor4 + 0.0784 * sensor4^2 
% This fitted curve results in a norm of the residuals of: 426.4505

figure; plot(sensordata_train(first_cycle,9), sensordata_train_RUL(first_cycle,1)); hold on;
plot(quad_fit); hold on;
plot(exp_fit);

%%For the function capturing the system dynamics of RUL(t) *starting from 
% t=2) given RUL(t-1) 
% and the sensor_obs I fitted a function taking 2 inputs using the curve 
% fitting app of Matlab, and is extracted from there.
figure; plot3(sensor_obs, RUL_true, RUL_true_tminus1);


% To check if the functions indeed construct the true RUL, I have computed
% RUL_t from them, given the observations of the training data.

% t = 1
RUL_t(1,1) = 1.6369e5 + (-226.5514) * sensor_obs(1,1) + 0.0784 * sensor_obs(1,1)^2;

% t = 2:192
RUL_t(2:size(RUL_true),1) = (-1) - (3.23e-13 * sensor_obs(2:192,1)) + (1 * ...
RUL_true_tminus1(2:192,1)) + (1.135e-16 * (sensor_obs(2:192,1).^2)) + (2.978e-17 * ...
sensor_obs(2:192,1) .* RUL_true_tminus1(2:192,1)) + (2.579e-10 * (RUL_true_tminus1(2:192,1).^2));
figure; plot(RUL_t);


%% Derivation of measurement function:
% sensor_t = h(rul_t)

[exp_fit_sens, exp_gof_sens, exp_output_sens] = fit(RUL_true, sensor_obs,'exp2');
figure; plot(RUL_true, sensor_obs); hold on;
plot(exp_fit_sens);
exp_formula_sens = formula(exp_fit_sens)
exp_norm_resid_sens = norm(exp_output_sens.residuals)
a = exp_fit_sens.a;
b = exp_fit_sens.b;
c = exp_fit_sens.c;
d = exp_fit_sens.d;

% sensor_obs = a*exp(b*rul_t) + c*exp(d*rul_t)
% sensor_obs = (35.2447*exp(-0.0155*rul_t)) + (1.3937e3*exp(1.7718e-5))