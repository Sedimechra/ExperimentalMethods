%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CODE CHALLENGE 2 - Monte Carlo Analysis
%
% The purpose of this challenge is to perform a Monte-Carlo analysis on the
% lift generated by an aircraft.  The aircraft has the following characteristics:
%   Wing surface area, S = 80 m^2
%   Lift coefficient, C_L = 0.90 +- 0.03
%
% And is flying under the following conditions
%   Air density, rho = 0.653 kg/m^3
%   Airspeed, V = 100 +- 10 m/s
%
% ---------------------------------------------------------------------------------
%
% To complete the challenge, execute the following steps:
% 1) Sample S, C_L, rho, and V 10,000 times.
% 2) Calculate lift in kilonewtons for each of the 10,000 samplings/simulations.
% 3) Calculate the best estimate and error for lift and report it to the
% command window using appropriate significant figures.
% 4) Plot a histogram of L.
% Bonus 1) Calculate drag in kilonewtons for each of the 10,000
% samplings/simulations.
% Bonus 2) Make a scatterplot of Lift vs Drag.
%
% NOTE: DO NOT change any variable names already present in the code.
% 
% Upload your team's script to Canvas to complete the challenge.
% 
% NAME YOUR FILE AS Challenge2_Sec{section number}_Group{group breakout #}.m 
% ***Section numbers are 1 or 2*** 
% EX File Name: Challenge2_Sec1_Group15.m 
%
%
% 1) 
% 2) 
% 3) 
% 4) 
% 5)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Housekeeping 
% (Please don't "clear all" or "clearvars", it makes grading difficult)
close all   % Close all open figure windows
clc         % Clear the command window

%% 1) Sample S, C_L, rho, and V 10,000 times
% (i.e. the S variable should contain 10000 samples of the wing surface area)
N = 1e04;
S = 
C_L = 
rho = 
V = 

%% 2) Calculate lift in kilonewtons for each of the 10,000 samplings/simulations.
% Given that the equation for lift is:
%       L = 0.5 * rho * V^2 * C_L * S (Newtons)
L = 

%% 3) Calculate the best estimate and error for lift
% Report it to the command window using appropriate significant figures.
L_best = 
L_err  = 

%% 4) Plot a histogram (use the "histogram" command) of L with 30 bins.  
% Add annotations and labels for style points!


%% Bonus 1) Calculate drag in kilonewtons 
% For each of the 10,000 samplings/simulations, given that the equation for drag is:
%       D = 0.5 * rho * V^2 * C_D * S (Newtons)
% and that C_D = 0.070 +- 0.005
C_D = 
D = 

%% Bonus 2) Make a scatterplot of Lift vs Drag.
%
% Think about the following (no work to do):
%     - Why do you think the points are spread into an ellipse and not a
%     circle?
%     - What is the significance of the general trend/slope of the data?
%     - How could this sort of analysis be useful when dealing with more
%     complicated systems and equations?





















