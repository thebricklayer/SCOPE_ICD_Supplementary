%Author: Mattia Astarita
%Date: 03/20/18
%Purpose: smooth the data finding the center point and smoothing 0.25
%seconds ahead and 0.25 seconds before. This function is called by multiple
%others to smooth data.

function [ time, distance ] = sMooth( t,d )
%find max time and min time
t_max = max(t);
t_min = min(t);
%time to smooth over
t_smooth = 0.5;
%compute at zero using the next quarter second
t_0 = 0;
indices = find(t>=0 & t< 0.25);
d_vector = d(indices);
distance_std= std(d_vector); 
distance_mean = mean(d_vector);
indices = find(d_vector<=(distance_std+distance_mean) & d_vector>=(-distance_std+distance_mean));
distance_0 = mean(d_vector(indices));
% time intervals array
t_int = (t_min+0.25):t_smooth:(t_max);

for i = 1:(length(t_int)-1)
    %find times between the intervals
    indices = find(t>=t_int(i) & t<= t_int(i+1));
    d_vector = d(indices);
    distance_std= std(d_vector); 
    distance_mean = mean(d_vector);
    indices = find(d_vector<=(distance_std+distance_mean) & d_vector>=(-distance_std+distance_mean));
    distance(i) = mean(d_vector(indices));
    time(i) = t_int(i)+0.25; 
    
end

time = [t_0 time];
distance = [distance_0 distance];


% %distance = d(1);
% %time = t(1);
% %initialie count
% count= 2;
% for i = 1:(length(t_int)-1)
%     indices = find(t>=t_int(i) & t<= t_int(i+1));
%     distance(i) = mean(d(indices));
%     time(count) = t_int(i+1);
%     count = count+1;
% 
%     
%     
% end
% 
