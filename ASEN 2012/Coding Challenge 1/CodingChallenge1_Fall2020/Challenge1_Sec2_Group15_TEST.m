%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CODE CHALLENGE 1 - 
%
% The purpose of this challenge is to estimate atmospheric pressure in
% Boulder CO using a pressure model and measurements, and compare the two
% through error analysis and statistics. 
%
% To complete the challenge, execute the following steps:
% 1) Load the given dataset
% 2) Extract altitude and pressure data
% 3) Determine standard deviation, variance, mean, and 
%    standard error of the mean of the pressure data
% 4) Using information given about the instrument, find uncertainty associated
%    with altitude measurements
% 5) Use the model to predict pressure measurements at each altitude in the
%    data set, along with propagated uncertainty
% 6) Compare results, discuss, and print answers to the command window. 
% Bonus) Repeat for larger measurement uncertainty in altitude
%
% NOTE: DO NOT change any variable names already present in the code.
% 
% Upload your team's script to Canvas to complete the challenge.
% 
% NAME YOUR FILE AS Challenge1_Sec{section number}_Group{group breakout #}.m 
% ***Section numbers are 1 or 2*** 
% EX File Name: Challenge1_Sec1_Group15.m 
%
%
% 1) Autumn Martinez
% 2) Thomas O'Connor
% 3) Siyang Liu
% 4) 
% 5)
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Housekeeping
clear all   % Clear all variables in workspace
close all   % Close all open figure windows
clc           % Clear the command window
%% 1) Load data from given file
%% 2) Extract just the altitude and station pressure data columns to meaningfully named variables

% Create a table which imports the data directly from the csv file for
% usability
boulderData = readtable("PressureInBoulder.csv")
% extract altitude and pressure data into individual arrays
AltitudeData = boulderData.Altitude_m_;
PressureData = boulderData.StationPressure_kPa_;
%% 3) Determine Statistics and Error
% the standard deviation, variance, mean, and standard error of the mean (sem) 
% of the pressure data

% calculate each of the statistics using standard matlab functions
StdevPressure = std(PressureData)

VarPressure = var(PressureData)

MeanPressure = mean(PressureData)
% standard error of the mean is standard deviation of the dataset divided
% by the square root of the number of values in the dataset
Sem_Pressure = StdevPressure / sqrt(length(PressureData))
%% 4) Uncertainty
%%
% 
%    The altitude measurements were taken using an instrument that displayed
%    altitude to the nearest tenth of a meter. 
%

%    What is the associated absolute uncertainty with these measurements? 

AltitudeUncertainty = 0.1; % [m]
%% 5) Pressure Predictions
%%
% 
%    Using the altitude measurements and uncertainty, predict pressure with the follwing model:
%    First, propagate uncertainty BY HAND before calculating uncertainty for each value. 
%    Then check: is it different for each calculation?
%

%               Model
%    P_est =   P_s * e^(-k*h)               
%    Assume P_s is 101.7 ± 0.4 kPa and k is 1.2*10^(-4) [1/m] 

P_s = 101.7;       % ± 0.4 [kPa]
k = 1.2*10^(-4);   % [1/m]

P_s_uncertainty = 0.4;

P_est = P_s * exp(-k*AltitudeData) % (vector of pressure estimates for each) (atmosphere pressure
% estimate)
% general method
P_sig = sqrt((exp(-k.*AltitudeData).*P_s_uncertainty).^2+(-k.*P_s*exp(-k.*AltitudeData)*AltitudeUncertainty).^2) % (uncertainty using general rule)

% p_est and p_sig vary slightly
%% 6) Print Results
%%
% 
%    Display the predicted pressure from the model with it's associated uncertainty and
%    the average pressure with the it's standard error of the mean from the data.
%

results  = table(P_est,P_sig);
P_data = [num2str(MeanPressure) '  ±  ' num2str(Sem_Pressure) '   kPa'];
disp(results);
disp(P_data);

% Disucss the accuracy of the model and whether or not you think the
% model agrees with the measurements 

% disp('Model Discussion: (The difference between the top end of our model and the bottom end of the measured data is ~0.2078 kPa, which could be significant depending on the application of the model. In my opinion, the model is close enough to the measurement to be considered accurate.) ')
%% Bonus
%%
% 
%   Repeat steps 4-6, but assume the altitude measurements were taken on a
%   lower precision instrument that only displayed altitude to nearest 10
%   meters
%   How does this change the results and comparison ? 
%

% new altitude uncertainty
altitude_uncertainty_new = 10    % [m]
% changing the uncertainty in p_sig
p_sig_new = sqrt((exp(-k.*AltitudeData).*P_s_uncertainty).^2+(-k.*P_s*exp(-k.*AltitudeData)*altitude_uncertainty_new).^2) % (uncertainty using general rule)

results  = table(P_est,p_sig_new);
P_data = [num2str(MeanPressure) '  ±  ' num2str(Sem_Pressure) '   kPa'];
disp(results);
disp(P_data);

percent_change = mean((p_sig_new - P_sig)./P_sig .* 100)

% An increase in the error of the altitude measurement by a factor of 100
% increases the error of pressure measurement by ~4.6 percent