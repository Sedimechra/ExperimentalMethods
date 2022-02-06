%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CODE CHALLENGE 9 - Guided Template Script
%
% The purpose of this challenge is to propagate an orbit in a two body
% system for one period, and to plot it's specific energy over time.
%
% To complete the challenge, execute the following steps:
% 1) Set an initial condition vector
% 2) Propagate for exactly period of the orbit
% 3) Calculate the specific energy of the s/c vs. time
% 4) Plot the trajectory, include points for where the trajectory starts,
% ends, and the where the Earth is.
% 5) Plot the change in specific energy vs. time
%
% NOTE: DO NOT change any variable names already present in the code.
% 
% Upload your team's script to Gradescope when complete.
% 
% NAME YOUR FILE AS Challenge9_Sec{section number}_Group{group breakout #}.m 
% ***Section numbers are 1 or 2*** 
% EX File Name: Challenge9_Sec1_Group15.m 
%
% STUDENT TEAMMATES
% 1) 
% 2) 
% 3) 
% 4)
% 5) 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Housekeeping
clear variables; close all; clc;


%% Set up
mu = ; % GM of Earth [km^3/s^2]
r = ; % initial r vetor [km]
v = ; % initial v vetor [km/s]
a = ; % calculating a [km]
T = ; % calculating T [s]
IC = ; % initial condition vector
t = ; % time domain [s]

%% Propagate w/ ode45


%% Calculate specific energy
energy = ;

%% Plotting
figure(1)
plot3(); % plot starting point
hold on; grid minor;
plot3(); % plot earth
xlabel(); 
ylabel();
zlabel();
title();
plot3(); % plot trajectory
plot3(); % plot ending point

figure(2)
plot(); %plot specific energy vs. time
grid minor;
xlabel();
ylabel();
title();

