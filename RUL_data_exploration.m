%% Data processing and exploration

% sensordata_struct_train = tdfread('train_FD001.txt');
% sensordata_train = struct2array(sensordata_struct_train);
% 
% sensordata_struct_test = tdfread('test_FD001.txt', 'tab');
% sensordata_test = struct2array(sensordata_struct_test);
% 
% sensordata_struct_RUL = tdfread('RUL_FD001.txt');
% sensordata_RUL = struct2array(sensordata_struct_RUL);

load('RUL_data.mat')

figure; plot(sensordata_train(1:191,25)); hold on;
plot(sensordata_train(192:478,25)); hold on;
plot(sensordata_train(479:657,25)); hold on;
plot(sensordata_train(658:846,25)); 

% Compute RUL for training data per cycle
sensordata_train_RUL = [];
for i = 1:100
    cycle_length = size(sensordata_train(sensordata_train(:,1)==i,1),1);
    inv_rul = 1:cycle_length;
    rul = abs(inv_rul' - cycle_length)+1;
    sensordata_train_RUL = [sensordata_train_RUL; rul];
end

figure; plot(sensordata_train_RUL(1:191,1));

% Plot multiple cycles of system behavior as observed by sensor 20 (clear
% degradation)
first_cycle = (sensordata_train(:,1)==1);
second_cycle = (sensordata_train(:,1)==2);
third_cycle = (sensordata_train(:,1)==3);
fourth_cycle = (sensordata_train(:,1)==4);

figure; plot(sensordata_train(first_cycle,25), sensordata_train_RUL(first_cycle,1)); hold on;
plot(sensordata_train(second_cycle,25), sensordata_train_RUL(second_cycle,1)); hold on;
plot(sensordata_train(third_cycle,25), sensordata_train_RUL(third_cycle,1)); hold on;
plot(sensordata_train(fourth_cycle,25), sensordata_train_RUL(fourth_cycle,1)); 

% Plot multiple cycles of system behavior as observed by sensor 2 (clear
% degradation)
figure; plot(sensordata_train(first_cycle,7), sensordata_train_RUL(first_cycle,1)); hold on;
plot(sensordata_train(second_cycle,7), sensordata_train_RUL(second_cycle,1)); hold on;
plot(sensordata_train(third_cycle,7), sensordata_train_RUL(third_cycle,1)); hold on;
plot(sensordata_train(fourth_cycle,7), sensordata_train_RUL(fourth_cycle,1)); 

% Plot multiple cycles of system behavior as observed by sensor 4 (clear
% degradation)
% Continue with this sensor as it shows quite stable relation with RUL
figure; plot(sensordata_train(first_cycle,9), sensordata_train_RUL(first_cycle,1)); hold on;
plot(sensordata_train(second_cycle,9), sensordata_train_RUL(second_cycle,1)); hold on;
plot(sensordata_train(third_cycle,9), sensordata_train_RUL(third_cycle,1)); hold on;
plot(sensordata_train(fourth_cycle,9), sensordata_train_RUL(fourth_cycle,1)); 

