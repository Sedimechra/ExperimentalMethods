%% Cleaning
clear all;
clc;
close all;

%% Initializing Variables
% Create conversion functions
psi2pa = @(P) P * (1/14.696) * 101325; % Convert from psi to Pa
areaD = @(d) pi * (d/2)^2;             % Calculate area given diameter

% Values given by verification case
g = 9.81;                  % Acceleration of gravity [m/s^2] acting in -Z direction
cDischarge = 0.8;          % Discharge coefficient
gamma = 1.4;               % Ratio of specific heats for air
R = 287;                   % Gas constant of air [J/kgK]

denAirAmb = 0.961;         % Ambient air density [kg/m^3]
pAmb = psi2pa(12.1);       % Atmospheric pressure [Pa]
tempAirI = 300;            % Initial temperature of air [K]

volB = 0.002;              % Volume of empty bottle [m^3]
dThroat = 2.1 / 100;       % Diameter of throat [m]
At = areaD(dThroat);       % Area of throat [m^2]
dBottle = 10.5 / 100;      % Diameter of bottle [m]
Ab = areaD(dBottle);       % Area of bottle [m^2]
mBottle = 0.15;            % Mass of empty bottle with cone and fins [kg]

velI = 0;                  % Initial magnitude of velocity [m/s]
velIX = 0;                 % Initial velocity in x [m/s]
velIZ = 0;                 % Initial velocity in z [m/s]
x0 = 0;                    % Initial horizontal distance [m]
z0 = 0.25;                 % Initial vertical height [m]
lenS = 0.5;                % Length of test stand [m]

%% Fluctuating Parameters
VerificationCase = 0;                         % If wanting to output verification case, set to 1, otherwise set to 0

if VerificationCase                           % Run verification case
    pAirI = psi2pa(50)+pAmb;                  % Initial gage pressure of air in bottle added to ambient pressure [Pa]
    cDrag = .5;                               % Coefficient of drag
    angleLaunch = 60;                         % Launch angle [deg]
    volWI = 0.001;                            % Initial volume of water in bottle [m^3]
else                                          % Trail and error by hand
    % These parameters to be varied by hand
    pAirI = psi2pa(70)+pAmb;                  % Initial gage pressure of air in bottle added to ambient pressure [Pa]
    cDrag = .45;                              % Coefficient of drag
    angleLaunch = 46.5;                         % Launch angle [deg]
    volWI = 0.001;                            % Initial volume of water in bottle [m^3]
end

denWater = 1000;                          % Density of water [kg/m^3]
mWaterI = denWater * volWI;               % Initial mass of water [kg]
volAI = volB - volWI;                     % Initial volume of air in bottle [m^3]
mAirI = (pAirI * volAI) / (R * tempAirI); % Initial mass of air in the bottle [kg]
mRocket = mAirI + mWaterI + mBottle;      % Initial total mass of rocket [kg]

%% Calling ode45

% Initialize initial vectors
h = [g, cDischarge, gamma, R, denAirAmb, pAmb, tempAirI, volB, dThroat, ...
    At, dBottle, Ab, mBottle, velI, x0, z0, lenS];                                % Vector of standard measurements
c = [pAirI, cDrag, angleLaunch, denWater, volWI, mWaterI, volAI, mAirI, mRocket]; % Vector of variable measurements
X0 = [x0, z0, velIX, velIZ, mRocket, mAirI, volAI];                                 % Vector of initial state vector values
tSpan = [0, 5];                                                                   % Time span

option = odeset('Events', @stopEvent);                                            % Sets condition for stopping oder45 call
[t, X] = ode45(@(t,x) odeFunction(t, x, h, c), tSpan, X0, option);

% Calculate indexes to keep track of phases
pEnd = pAirI * ((volAI / volB) ^ gamma);                    % Calculate pEnd to be used in pCurrVec
pCurrVec = pEnd .* (X(:, 6) ./ mAirI) .^ gamma;             % Create vector of pressure measurements
idx1 = find(X(:, 7) < volB);                                % Index containing data within phase 1
idx2 = find(pCurrVec(idx1(end)+1:end) > pAmb) + idx1(end);  % Index containing data within phase 2
idx3 = find(pCurrVec(idx1(end)+1:end) <= pAmb) + idx1(end); % Index containing data within phase 3

% Calculate thrust of rocket
aX = gradient(X(:, 3)) ./ gradient(t(:));                                            % Take derivative of x velocity to find x acceleration
aZ = gradient(X(:, 4)) ./ gradient(t(:));                                            % Take derivative of z velocity to find z acceleration
Fx = aX .* X(:, 5);                                                                  % Find total force in x
Fz = aZ .* X(:, 5);                                                                  % Find total force in z
drag = .5 .* denAirAmb .* (X(:, 3).^2 + X(:, 4).^2) .* cDrag .* Ab;                  % Find vector of drag values
% Heading vector
hx = X(:, 3) ./ sqrt(X(:, 3).^2 + X(:, 4).^2);                                       % Heading vector in x
hz = X(:, 4) ./ sqrt(X(:, 3).^2 + X(:, 4).^2);                                       % Heading vector in z
headIdx = X(:, 1) < lenS*cosd(angleLaunch)+x0 & X(:, 2) < lenS*sind(angleLaunch)+z0; % Index highlighting time when rocket on launch stand
hx(headIdx) = cosd(angleLaunch);                                                     % Set launch stand angle while on launch stand
hz(headIdx) = sind(angleLaunch);                                                     % Set launch stand angle while on launch stand
% Calculate thrust
Tx = (Fx + drag.*hx) ./ hx;                                                          % Calculate force of thrust in x
Tz = (Fz + drag.*hz + X(:, 5).*g) ./ hz;                                             % Calculate force of thrust in z
Tx(1) = Tx(2);                                                                       % Set initial value to avoid weird graph
Tz(1) = Tz(2);                                                                       % Set initial value to avoid weird graph
ThrustTot = mean([Tx, Tz], 2);                                                       % Calculate average between x and z calculated thrust to get average total thrust
%% Plotting
% Plot of trajectory
subplot(2,2,1);
plot(X(:, 1), X(:, 2));
hold on;
yline(0, 'g');
xline(0, 'r');
xline(X(idx1(end), 1), 'b');
xline(X(idx2(end), 1), 'b');
xline(X(idx3(end), 1), 'b');
scatter(X(idx1(end), 1), X(idx1(end), 2), 'b');
scatter(X(idx2(end), 1), X(idx2(end), 2), 'b');
scatter(X(idx3(end), 1), X(idx3(end), 2), 'b');
hold off;
grid on; grid minor;
title("Rocket Trajectory");
xlabel("X Distance (m)");
ylabel("Z Distance (m)");

% Plot of thrust profile
subplot(2,2,2);
plot(t(t(:) < .45), ThrustTot(t(:) < .45));
grid on; grid minor;
title("Plot of Thrust over Time");
xlabel("Time (s)");
ylabel("Force of Thrust (N)");

% Plot of x velocity
subplot(2,2,3);
plot(t, X(:, 4));
grid on; grid minor;
title("Z-Velocity Evolution");
xlabel("Time (s)");
ylabel("Velocity in Z (m/s)");

% Plot of z velocity
subplot(2,2,4);
plot(t, X(:, 3));
grid on; grid minor;
title("X-Velocity Evolution");
xlabel("Time (s)");
ylabel("Velocity in X (m/s)");

% figure();
% volIdx = find(X(:, 7) > .002, 1);  % Calculates point when water fully expelled
% plot(t(1:volIdx), X(1:volIdx, 7));
% grid on; grid minor;
% title("Bottle Rocket Air Volume Evolution");
% xlabel("Time (s)");
% ylabel("Volume of Air in Bottle (m^3)");

% Output maximum height and distance
max_height = max(X(:, 2));
max_dist = max(X(:, 1));
fprintf("Maximum height reached: %f\n", max_height);
fprintf("Maximum distance reached: %f\n", max_dist);

%% Functions
function [value, isterminal, direction] = stopEvent(t, X)
% Function sets event that will stop ode45

value = (X(2) <= 0); % Set event to z position hitting ground
isterminal = 1;      % Stop the integration
direction = 0;

end

function dXdt = odeFunction(t, X, h, c)
% Function calculates rocket state vector throughout flight. Takes in
% starting parameters. Is able to differentiate between thrust phases

% X = [
%     x - x position
%     z - z position
%     Vx - x component of velocity
%     Vz - z component of velocity
%     RM - rocket mass
%     AM - mass of air
%     AV - volume of air
% ]
% 
% Xdot = [
%     vx - velocity in x
%     vz - velocity in z
%     ax - acceleration in x
%     az - acceleration in z
%     RMdot - change in rocket mass
%     AMdot - change in mass of air
%     AVdot - change in volume of air
% ]

%% Reassign variables
% Assign constant parameters
g = h(1);           % Gravity [m/s^2]
cDischarge = h(2);  % Coefficient of discharge
gamma = h(3);       % Ratio of specific heats of air
R = h(4);           % Gas constant of air [J/kgK]
denAirAmb = h(5);   % Ambient air density [kg/m^3]
pAmb = h(6);        % Atmospheric pressure [kg/m*s^2]
tempAirI = h(7);    % Initial temperature of air [K]
volB = h(8);        % Volume of empty bottle [m^3]
dThroat = h(9);     % Diameter of throat [m]
At = h(10);         % Area of throat [m^2]
dBottle = h(11);    % Diameter of bottle [m]
Ab = h(12);         % Area of bottle [m^2]
mBottle = h(13);    % Mass of empty bottle with cone and fins [kg]
velI = h(14);       % Initial velocity [m/s]
x0 = h(15);         % Initial horizontal position [m]
z0 = h(16);         % Initial vertical position [m]
lenS = h(17);       % Length of test stand [m]

% Assign changing parameters
pAirI = c(1);       % Initial gage pressure of air in bottle [kg/m*s^2]
cDrag = c(2);       % Coefficient of drag
angleLaunch = c(3); % Launch angle [deg]
denWater = c(4);    % Density of water [kg/m^3]
volWI = c(5);       % Initial volume of water in bottle [m^3]
mWaterI = c(6);     % Initial mass of water [kg]
volAI = c(7);       % Initial volume of air in bottle [m^3]
mAirI = c(8);       % Initial mass of air in bottle [kg]
mRocket = c(9);     % Initial total mass of rocket [kg]

% Current array values
xpos = X(1);        % Current x position [m]
zpos = X(2);        % Current z position [m]
currVx = X(3);      % Current x velocity [m/s]
currVz = X(4);      % Current z velocity [m/s]
currRM = X(5);      % Current rocket mass [kg]
currAM = X(6);      % Current air mass [kg]
currAV = X(7);      % Current air volume [m^3]

%% Phase 1
if (currAV < volB) % Volume air less than volume of bottle
    
    % Pressure in bottle
    pCurr = pAirI * (volAI ./ currAV).^gamma;
    
    % Exhaust velocity
    Ve = sqrt((2*(pCurr - pAmb)) / denWater);
    
    % Rate of change of volume with air
    dvdt = cDischarge * At * sqrt((2/denWater) * ((pAirI * ((volAI / currAV)^gamma)) - pAmb));
    
    % Mass flow rate of water through throat
    mWdot = cDischarge * denWater * At * Ve;
    
    % Calculate thrust
    F = 2 * cDischarge * At * (pCurr - pAmb);
    
    % Assign output variables
    dRMdt = -1 * cDischarge * At * sqrt(2*denWater* (pCurr - pAmb)); % Rocket mass changes as water mass exits bottle
    dAMdt = 0;                                                       % Air does not exit bottle during first phase
    dAVdt = dvdt;                                                    % Air volume changes according to given equation
    
end

%% Needed for differentiating between stages 2 and 3
if (currAV >= volB)
    % Pressure and temperature at time water is expelled
    pEnd = pAirI * ((volAI / volB) ^ gamma);
    tEnd = tempAirI * ((volAI / volB) ^ (gamma - 1));

    % Find pressure inside bottle after water is expelled
    pCurr = pEnd * (currAM / mAirI) ^ gamma;
end

%% Phase 2
if (currAV >= volB && pCurr > pAmb)   % Volume of air equal to volume of bottle and gage pressure greater than atmospheric pressure
    denCurr = currAM / volB;          % Corresponding density
    tempCurr = pCurr / (denCurr * R); % Corresponding temperature
    
    % Calculate critical pressure of air at exit
    pCrit = pCurr * ((2/(gamma+1))^(gamma/(gamma-1)));
    
    if (pCrit > pAmb)                   % Calculate condition for choked flow
        MachE = 1;                      % Mach number        
        tempE = (2/(gamma+1)) * tempCurr;   % Temperature of airflow at exit
        denE = pCrit / (R * tempE);     % Density of airflow at exit
        pE = pCrit;                     % Pressure of airflow at exit
        
    elseif (pCrit <= pAmb)                                                    % Calculate condition for un-choked flow
        MachE = sqrt((2/(gamma-1)) * ((pCurr / pAmb)^((gamma-1)/gamma) - 1)); % Mach un-choked therefore < 1
        tempE = tempCurr / (1 + (((gamma-1)/2)*(MachE^2)));                   % Temperature of airflow at exit
        denE = pAmb / (R * tempE);                                            % Density of airflow at exit
        pE = pAmb;                                                            % Pressure of airflow at exit
                
    end
    
    % Calculate exhaust velocity
    velE = MachE * sqrt(gamma * R * tempE); % Velocity at exit
    
    % Calculate mass flow rate of air
    mAirDot = cDischarge * denE * At * velE; % Mass flow rate of air
    
    % Calculate thrust
    F = (mAirDot * velE) + ((pAmb - pE) * At);
    
    % Assign output variables
    dRMdt = -1 * mAirDot;
    dAMdt = -1 * mAirDot;
    dAVdt = 0;            % Air volume does not change during phase 2
end
    
%% Phase 3
if (currAV >= volB && pCurr <= pAmb) % Volume of air equal to volume of bottle and gage pressure equal to atmospheric pressure
    % Calculate thrust force (is 0 for ballistic phase)
    F = 0;
    
    % Assign output variables
    dRMdt = 0;
    dAMdt = 0;
    dAVdt = 0;
end

%% Compute outputs
% Set condition for rocket being on test stand
if xpos < lenS*cosd(angleLaunch)+x0 && zpos < lenS*sind(angleLaunch)+z0
    Hz = sind(angleLaunch);
    Hx = cosd(angleLaunch);
else
    Hz = currVz / sqrt(currVx^2 + currVz^2); % Find heading vector in z-direction
    Hx = currVx / sqrt(currVx^2 + currVz^2); % Find heading vector in x-direction
end

% Calculate drag (not dependent on phase of rocket)
D = .5 * denAirAmb * (currVx^2 + currVz^2) * cDrag * Ab;

% Calculate sum of forces
sumForcesZ = F*Hz - D*Hz - currRM*g; % Sum of the forces in the z-direction
sumForcesX = F*Hx - D*Hx;            % Sum of the forces in the x-direction

% F = ma --> a = F/m
accX = sumForcesX / currRM; % Acceleration in x-direction given sum of forces in x
accZ = sumForcesZ / currRM; % Acceleration in z-direction given sum of forces in z

dXdt = [currVx; currVz; accX; accZ; dRMdt; dAMdt; dAVdt];

end