%Author: Mattia Astarita
%Date: 03/20/18
%Purpose: Compare the truth LRF data with the Actual data. Plot the two
%curves on top of each other, call Smooth function to smooth the data. This
%is preliminary post-processing of the data, the actual post-processing
%will be done later by the statistics lead. This program is used to
%visually confirm the validity of the data right after testing.

close all
clear all
%parameters to change based on set up
d = 15.8369; %[m] distance between the two LRF
x = 0.444445;%[m]  distance between front and back

%read in and assign variables
M= dlmread('distance.txt','',1,0);
M_t = dlmread('truth_distance.txt','',1,0);
distance = M(:,1);  

Velocity_LRF = M(:,3);

%truth values
distance_t = M_t(:,1);  

%adjust times
if M_t(1,2)> M(1,2)
    time_start = M(1,2);
else
    time_start = M_t(1,2);
end

time_t = M_t(:,2)- time_start;
time = M(:,2)-time_start;

%adjust truth distances
distance_t = d-(M_t(:,1)+x);
%initialize a count
count = 2;
%smooth data
[time,distance] = sMooth(time,distance);
%smooth velocity
[time,Velocity_LRF] = sMooth(time,Velocity_LRF);
%Smooth truth
[time_t,distance_t] = sMooth(time_t,distance_t);
%take dimensions of matrix
sz = size(time);
%calculate velocity from data

for i= 1:(sz(2)-1)
    current_d = distance(i);
    next_d = distance(i+1);
    current_t = time(i);
    next_t = time(i+1);   
    velocity(i) = (next_d-current_d)/(next_t-current_t);

end
 
%take dimensions of matrix truth
sz_t = size(time_t);
%calculate velocity from data

for i= 1:(sz_t(2)-1)
    current_d_t = distance_t(i);
    next_d_t = distance_t(i+1);
    current_t_t = time_t(i);
    next_t_t = time_t(i+1);   
    velocity_t(i) = (next_d_t-current_d_t)/(next_t_t-current_t_t);

end
 


%calculate velocity if data not smooth
% for i= 1:(sz(1)-1)
%     current_d = distance(i,1);
%     next_d = distance(i+1,1);
%     current_t = time(i,1);
%     next_t = time(i+1,1);
%     
%     if current_d ~= next_d
%         velocity(count,1) = (next_d-current_d)/(next_t-current_t);
%         
%         
%         real_time(count,1) = time(i);
%         count= count+1;
%     end
%     
%     
% end



%plot distance
figure;

subplot(3,1,1)
plot(time, distance)
hold on
plot(time_t,distance_t)
title('distance vs time')
legend('measured','truth')
xlabel('[s]')
ylabel('[m]')
hold off
%plot velocity

subplot(3,1,2)
plot(time(1:(end-1)), velocity)
hold on
plot(time_t(1:(end-1)), velocity_t)
hold off
title('velocity vs time')
legend('measured','truth')
xlabel('[s]')
ylabel('[m/s]')

%plot velocity LRF

subplot(3,1,3)
plot(time, Velocity_LRF)
title('velocity_LRF vs time')
xlabel('[s]')
ylabel('[m/s]')


