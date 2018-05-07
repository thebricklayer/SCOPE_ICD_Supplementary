%Author: Mattia Astarita
%Date: 03/20/18
%Purpose: look the the Acuity Laser Rangefinder data, smooth it and plot it
%to visually verify that nothing went wrong during the test. This is a preliminary
%data post-processing, and it will be updated and changed for the actual
%post-processing later. 

close all
clear all

%open fileiD

[d,t] = textread('test1 copy.txt','%s %s') 
cnt = 1;

for i = 2:length(d)
    
    if strcmp(d{i}, 'E15') == 1 
    
    else
       distance_t(cnt,1) =  str2num(d{i,1});
       time_t(cnt,1) = str2num(t{i,1});
       cnt = cnt+1;
    end
    
    
end



%adjust times

time_start = time_t(1)
time_t = time_t- time_start;



%initialize a count
count = 2

%Smooth truth
[time_t,distance_t] = sMooth(time_t,distance_t);

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
subplot(2,1,1)
hold on
plot(time_t,distance_t)
title('distance vs time')
legend('measured','truth')
xlabel('[s]')
ylabel('[m]')
hold off
%plot velocity

subplot(2,1,2)
plot(time_t(1:(end-1)), velocity_t)
title('velocity vs time')
legend('measured','truth')
xlabel('[s]')
ylabel('[m/s]')



