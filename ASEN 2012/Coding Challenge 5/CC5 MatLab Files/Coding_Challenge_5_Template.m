%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CODE CHALLENGE 5 - Template Script
%
% The purpose of this challenge is to predict whether or not the Boulder
% Reservior will have to close due to a major leak.
%
% To complete the challenge, execute the following steps:
% Part 1:
% 1) Read in the data file
% 2) Set values to any constants
% 3) Perform a trapazoid integration on the data w/r.t. x
% 4) Perform a simpson's 1/3 integration on the data w/r.t. x
% 5) Display which volume measurement is more accurate and why
%
% Part 2:
% 1) Define which delta t will be used in the Euler integration
% 2) Set values to any constants and initial conditions
% 3) Propagate h with t using Euler integration
% 4) Repeat steps 1-4 with different delta t values
% 5) Display which delta t gives a more accurate result and why.
% 
%
% NOTE: DO NOT change any variable names already present in the code.
% 
% Upload your team's script to Gradescope to complete the challenge.
% 
% NAME YOUR FILE AS Challenge5_Sec{section number}_Group{group breakout #}.m 
% ***Section numbers are 1 or 2*** 
% EX File Name: Challenge5_Sec1_Group15.m 
%
%
% 1) 
% 2) 
% 3) 
% 4) 
% 5)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Housekeeping
% don't "clear variables", it makes things easier to grade
close all;   % Close all open figure windows
clc;         % Clear the command window


%% Part 1

%% Set up
data = nan; % read in .csv
x = nan; % [ft]
d = nan; % [ft]
L = nan; % length of reservior [ft]

%% Trapazoid - Calculate Volume
Vol_trap = nan; % [ft^3]

%% Simpson 1/3 - Calculate Volume
V_simp = nan; % [ft^3]


%% Part 2

%% Set up
del_t = nan; % various delta t values to test [days]

figure(1) % create figure

h0 = nan; % initial depth
alpha = nan; % relating volume out per day to depth [ft^2/day]
dV_in = nan; % volume in rate per day
t = nan; % allocate time vector [days]
h = nan; % allocate depth vector [ft]
h(1)= nan; % set initial value in h vector

for i = 1:(length(t)-1) % Euler method
    dhdt = nan; % get dh/dt at this depth
    h(i+1) = nan; %compute next depth value
end

% plot results

% labels for plot




