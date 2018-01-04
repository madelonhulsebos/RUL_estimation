%% Extended Kalman Filter implementation: estimation on training run!

load('RUL_data.mat')

% Data to use (EKF and evaluation)
sensor_obs = sensordata_train(sensordata_train(:,1)==1,9);
RUL_true = sensordata_train_RUL(sensordata_train(:,1)==1,1)';

% Matrix to hold intermediate state estimation on t and RUL estimates of
% the run
mean_KF = [];
st_matrix = [];

% Model variables and coefficients
% Assumed observation noise
varn = 5;
% Assumed process noise
varw = 5;
% Part of Process Noise Covariance matrix
Q = [2 0.5 0.5; 0.1 1 0.1; 0.1 0.1 0.5];

% Coefficients
a = 35.2447;
b = -0.0155;
c = 1.3937e3;
d = 1.7718e-5;
p1 = -1;
p2 = 3.23e-13;
p3 = 1;
p4 = 1.135e-16;
p5 = 2.978e-17;
p6 = 2.579e-10;

% Number of cycles
k = size(sensor_obs,1);

for t=1:k

   if(t==1)

       % First prediction
       st = [(1.6369e5 + (-226.5514) * sensor_obs(1,1) + 0.0784 * ...
           sensor_obs(1,1)^2); (1.6369e5 + (-226.5514) * sensor_obs(1,1) + 0.0784 * ...
           sensor_obs(1,1)^2)+1; 0 ];
       st_matrix(:,t) = st;

       % Initial covariance 
       Pt = diag([10 10 5]);

   else

       % KF updates: in first cycle, it makes the updates after initial
       % prediction.

       % jacobian of h in t
       H = [(b*a*exp(b*st(1,1))), 0, 0];
       % jacobian of f in t
       F = [0, (p3 + (p5 * sensor_obs(t,1)) + (2 * p6 * st(1,1))) , ...
           (p2  + (2 * p4 * (sensor_obs(t,1))) + (p5 * st(1,1))) ; ...
           0, 1, 0; ...
           1 , -1 , 0];          

       % innovation covariance
       meas_cov = H * Pt * H' + varn;
       % Kalman gain
       K = Pt * H' * inv(meas_cov);

       % measurement residual
       meas_res = sensor_obs(t,1) - ((a*exp(b*st(1,1))) + (c*exp(d)));
       % Updated state estimate
       st = st + K * meas_res;
       % Updated covariance estimate
       Pt = (eye(3) - K * H) * Pt;


       % KF predictions
       % Entry 1: st_t, entry 2: st_t-1, and entry 3: st_t - st_t-1
       st = [p1 - (p2*sensor_obs(t,1)) + (p3*st(1,1)) + ...
            (p4*(sensor_obs(t,1)^2)) + (p5*sensor_obs(t,1) * ...
            st(1,1)) + (p6*(st(1,1).^2)) ;  st(1,1)]; 
       % st has been changed here already! 
       st = [st ; st(1,1) - st(2,1)];
       st_matrix(:,t) = st;

       Pt = F * Pt * F'; 
       Pt = Pt + Q * varw;

   end

   mean_KF(:,t) = st;

end

estimation_residual = RUL_true - mean_KF(1,:);

figure; plot(mean_KF(1,:)); hold on;
plot(RUL_true); hold on;
plot(estimation_residual);

RMSE_training = (1/size(RUL_true,2)) * sum(estimation_residual.^2);
% RMSE = 802.5471


%% Evaluation of EKF by means of the test data

n = size(sensordata_test,1);

RUL_estimated = [];
st_matrix = [];

for i = 1:sensordata_test(n,1)
    
    % Data to use (EKF and evaluation)
    sensor_obs = sensordata_test(sensordata_test(:,1)==i,9);
    
    % Matrix to hold intermediate RUL estimates of cycle
    mean_KF = [];
    
    % Model variables and coefficients
    
    % Assumed observation noise
    varn = 5;
    % Assumed process noise
    varw = 5;
    % Part of Process Noise Covariance matrix
    Q = [2 0.5 0.5; 0.1 1 0.1; 0.1 0.1 0.5];
    
    % Coefficients
    a = 35.2447;
    b = -0.0155;
    c = 1.3937e3;
    d = 1.7718e-5;
    
    p1 = -1;
    p2 = 3.23e-13;
    p3 = 1;
    p4 = 1.135e-16;
    p5 = 2.978e-17;
    p6 = 2.579e-10;

    % Number of measurements
    k = size(sensor_obs,1);

    for t=1:k

       if(t==1)
           
           % First prediction
           st = [(1.6369e5 + (-226.5514) * sensor_obs(1,1) + 0.0784 * ...
               sensor_obs(1,1)^2); (1.6369e5 + (-226.5514) * sensor_obs(1,1) + 0.0784 * ...
               sensor_obs(1,1)^2)+1; 0 ];
           st_matrix(:,t) = st;
           
           % Initial covariance 
           Pt = diag([10 10 5]);
           
       else

           % KF updates: in first cycle, it makes the updates after initial
           % prediction.
           
           % jacobian of h in t
           H = [(b*a*exp(b*st(1,1))), 0, 0];
           % jacobian of f in t
           F = [0, (p3 + (p5 * sensor_obs(t,1)) + (2 * p6 * st(1,1))) , ...
               (p2  + (2 * p4 * (sensor_obs(t,1))) + (p5 * st(1,1))) ; ...
               0, 1, 0; ...
               1 , -1 , 0];          
                      
           % innovation covariance
           meas_cov = H * Pt * H' + varn;
           % Kalman gain
           K = Pt * H' * inv(meas_cov);
           
           % measurement residual
           meas_res = sensor_obs(t,1) - ((a*exp(b*st(1,1))) + (c*exp(d)));
           % Updated state estimate
           st = st + K * meas_res;
           % Updated covariance estimate
           Pt = (eye(3) - K * H) * Pt;
     
           
           % KF predictions
           % Entry 1: st_t, entry 2: st_t-1, and entry 3: st_t - st_t-1
           st = [p1 - (p2*sensor_obs(t,1)) + (p3*st(1,1)) + ...
                (p4*(sensor_obs(t,1)^2)) + (p5*sensor_obs(t,1) * ...
                st(1,1)) + (p6*(st(1,1).^2)) ;  st(1,1)]; 
           % st has been changed here already! 
           st = [st ; st(1,1) - st(2,1)];
           st_matrix(:,t) = st;

           Pt = F * Pt * F'; 
           Pt = Pt + Q * varw;

       end

       mean_KF(:,t) = st;

    end
    
    RUL_estimated(i,1) = mean_KF(1,k);

end

estimation_residual =  RUL_estimated - sensordata_test_RUL;

figure; stem(sensordata_test_RUL); hold on;
stem(RUL_estimated); hold on;
stem(estimation_residual);

% Full evaluation by means of metrics 

score = 0;

for j = 1:size(estimation_residual,1)

    res = estimation_residual(j,1);
    
    if res < 0
    
        score = score + exp(-res/13) - 1;
        
    else
    
    	score = score + exp(res/10) - 1;
        
    end
    
end
% Score = 4.3788e+03

% Equal weights to early and late detections RMSE:
RMSE = (1/size(sensordata_test_RUL,1)) * sum(estimation_residual.^2);
% RMSE = 1.2708e+03
