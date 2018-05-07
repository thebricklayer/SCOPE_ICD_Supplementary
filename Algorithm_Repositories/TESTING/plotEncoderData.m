% Plot Encoder Data
close all; clear all; clc;

encoder_data = dlmread('test1.csv');
time = encoder_data(:,1);
theta = encoder_data(:,2);
w = encoder_data(:,3);
wnew = zeros(length(encoder_data),1);
t = linspace(0,floor(length(encoder_data)/2),length(encoder_data));

for i=1:length(encoder_data)-2
    wnew(i) = (encoder_data(i+1) - encoder_data(i))/(t(i+1) - t(i));
end

%t = t(1:end-2);
%wnew = wnew(1:end-2);

modelt = [0 72]; %sec
modeltheta = [0 360];
modelw = [5 5];

figure
hold on
subplot(2,1,1)
plot(t, theta, 'LineWidth', 2);
hold on
plot(modelt,modeltheta, 'LineWidth', 1);
title('Angular Position of TARGET vs. Time')
xlabel('Time Elapsed (s)')
ylabel('Angular Position (^o)')
set(gca, 'FontSize', 14);
legend('Measured','Model')

%t = t(1:end-2);

subplot(2,1,2)
plot(t, w, 'LineWidth', 1);
hold on
plot(t, mean(w)*ones(length(w),1), '--m', 'Linewidth', 2)
hold on
plot(modelt, modelw, '--k', 'LineWidth', 2);
title('Angular Speed of TARGET vs. Time - 5^o/s')
xlabel('Time Elapsed (s)')
ylabel('Angular Speed (^o/s)')
%plot(t, w, 'LineWidth', 1)
legend('Raw Rotation Rate Data', 'Mean', 'Model')
set(gca, 'FontSize', 14);

fprintf('Mean of angular rotation rate is: %.2f ^o/s\n', mean(wnew))