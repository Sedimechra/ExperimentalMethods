%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CODE CHALLENGE 4 - Linear Least-Squares Fit
%
% The purpose of this program is to calculate the equation of the best fit
% line for a data set using linear least-squares fitting.
%
% To complete the challenge, finish the code below to:
% 1) load data from csv file
% 2) find linear best fit coefficients and associated uncertainty
% 3) plot the original data along with the best fit line 
% 4) add errorbars for fit uncertainty to this plot from the data and from
%    the linear regression parameters
%
% NOTE: DO NOT change any variable names already present in the code.
% 
% Upload your team's script to Gradescope when complete.
% 
% NAME YOUR FILE AS Challenge4_Sec{section number}_Group{group breakout #}.m 
% ***Section numbers are 1 or 2*** 
% EX File Name: Challenge4_Sec1_Group15.m 
%
% STUDENT TEAMMATES
% 1) 
% 2) 
% 3) 
% 4)
% 5) 
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Housekeeping (Please don't "clear all" or "clearvars", it makes grading difficult)
close all   % Close all open figure windows
clc           % Clear the command window

%% Load and extract the time and velocity vectors from the data
data = 
t =     % [s]
v =     % [m/s]

%% Calculations
% Find number of data points in the vectors
N = 

% Find linear best fit coefficients A and B
% Create H matrix
H = 

% Create y matrix
y = 

% Create W matrix (hint: type <help diag> in command line)
W = 

% Solve for P matrix
P = 

% Solve for x_hat matrix and extract A and B parameters
x_hat = 
A = 
B = 

% extract uncertainty in A and uncertainty in B from P matrix
A_error = 
B_error = 

%% Display acceleration with associated uncertainty and the intial velocity with associated uncertainty
%  Make sure to use and display with CORRECT SIG FIGS


%% Find predicted velocity values using your linear fit equation
v_predicted = 

%% Ploting and Error Calculations
% On the same plot, do the following:
% 1. plot the velocity data vs time as a scatter plot 
% 2. plot predicted velocity vs time as a line 
% 3. title your plot so that it indicates what celestial body this data
%    simulates
% 4. Add measured velocity error bars and predicted velocity error bars to 
%    the plot (hint - this will involve error propagation calculations
v_err =
v_predicted_error = 