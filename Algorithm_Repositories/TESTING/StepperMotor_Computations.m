%% Connor Kerry
% 2/19/2018
% Stepper motor duty cycle calculation code

% Housekeeping
clear all; close all; clc;

%% constants/givens
ppr = 51200;
D_spool = 0.1397;%m
D_cable = 0.000;%m
linV = linspace(0.1,1,10);%m/s


%% Calcs
D_total = D_spool + D_cable;
%w = linV.*180/(pi*D_total/2);%deg/s
w = 5.*ones(1,length(linV));%deg/s
%w = 82.027.*ones(1,length(linV));
deg_per_pulse = 360/ppr;
sec_per_pulse = deg_per_pulse./(2*w);%half period
microsec_per_pulse = (sec_per_pulse.*1e6)';
linV = linV';
w = w';
%w = w./360;%Revs/second
%% Print results
table(linV, microsec_per_pulse, w)
fprintf('Note: Linear Velocities are in m/s\n');

