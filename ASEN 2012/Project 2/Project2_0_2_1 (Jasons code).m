%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                         %
%                 Project #2 - ASEN 2012                  %
%                     Bottle Rocket                       %
%                        V 0.21                           %
%                                                         %
%                     Jason Popich                        %
%                      109409863                          %
%                                                         %
%                      11/13/2020                         %
%                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Set Environment
clear
clc

% Determine whether to use verification constants that were given (true)
% or custom constants (false)
verification_Plots = true;

%% Set Variables
if (verification_Plots == true)
    R = 287;                        % Gas Constant [J*kg^-1*K^-1]
    g = 9.81;                       % Acceleration of Gravity [m/s^2]
    c_d = 0.8;                      % Discharge Coefficient
    C_D = 0.5;                      % Drag Coefficient
    y = 1.4;                        % Specific Heat Ratio of Air [Unitless?]
    temperature_air_initial = 300;  % [degrees K]

    % Measurements
    d_throat = 2.1;                 % Diameter of throat [cm]
    d_bottle = 10.5;                % Diameter of bottle [cm]

    % Pressure
    p_ambient = 12.1;               % Ambient pressure [psi]
    p_air_gage_initial = 50;        % Gage pressure of inside of bottle [psi]

    % Volume
    vol_water_initial = 0.001;      % Volume of water in bottle [m^3]
    vol_bottle = 0.002;             % Vol of bottle [m^3]

    % Density
    rho_water = 1000;               % Density of Water [kg/m^3]
    rho_amb_air = 0.961;            % Density of Ambient Air [kg/m^3]

    % Flight Variables
    V_Z_0 = 0;                      % Initial Velocity of Rocket in Z Direction [m/s]
    V_X_0 = 0;                      % Initial Velocity of Rocket in X Direction [m/s]
    X_0 = 0;                        % Initial Horizontal Distance of Rocket [m]
    Z_0 = 0.25;                     % Initial Vertical Distance of Rocket [m]
    theta = 45;                     % Initial Angle of Rocket [Degrees]
    l_s = 0.5;                      % Initial Length of Test Stand [m]
else
    R = 287;                        % Gas Constant [J*kg^-1*K^-1]
    g = 9.81;                       % Acceleration of Gravity [m/s^2]
    c_d = 0.8;                      % Discharge Coefficient
    C_D = 0.5;                      % Drag Coefficient
    y = 1.4;                        % Specific Heat Ratio of Air [Unitless?]
    temperature_air_initial = 300;  % [degrees K]

    % Measurements
    d_throat = 2.1;                 % Diameter of throat [cm]
    d_bottle = 10.5;                % Diameter of bottle [cm]

    % Pressure
    p_ambient = 12.1;               % Ambient pressure [psi]
    p_air_gage_initial = 150;       % Gage pressure of inside of bottle [psi]

    % Volume
    vol_water_initial = 0.001;      % Volume of water in bottle [m^3]
    vol_bottle = 0.002;             % Vol of bottle [m^3]

    % Density
    rho_water = 1000;               % Density of Water [kg/m^3]
    rho_amb_air = 0.961;            % Density of Ambient Air [kg/m^3]

    % Flight Variables
    V_Z_0 = 0;                      % Initial Velocity of Rocket in Z Direction [m/s]
    V_X_0 = 0;                      % Initial Velocity of Rocket in X Direction [m/s]
    X_0 = 0;                        % Initial Horizontal Distance of Rocket [m]
    Z_0 = 0.25;                     % Initial Vertical Distance of Rocket [m]
    theta = 45;                     % Initial Angle of Rocket [Degrees]
    l_s = 0.5;                      % Initial Length of Test Stand [m]
end

%% Calculate Initial Values

% Initial Angle of Rocket in Radians
theta_rad = deg2rad(theta);

% Pressure inside Rocket
p_air_initial = (p_air_gage_initial*6894.76) + (p_ambient*6894.76); % [N/m^2]

% Initial Volume of Air
vol_air_initial = vol_bottle - vol_water_initial; % volume of air in bottle [m^3]

% Initial Mass of Rocket
m_air_initial = (p_air_initial*vol_air_initial)/(R*temperature_air_initial); % Mass of Air [kg]
m_water_initial = rho_water*(vol_bottle - vol_air_initial); % Mass of Water [kg]
m_bottle = 0.15; % Mass of Bottle [kg]
mass_initial = m_bottle + m_water_initial + m_air_initial;  % Total Mass [kg]

% Area of Throat
A_t = pi*((d_throat*0.01)/2)^2; % m^2

% Area of Bottle
A_b = pi*((d_bottle*0.01)/2)^2; % m^2

% Time Range for Solver; 0-5 Seconds
tspan = [0 5];

% Variable Declaration for Solver
vars = [rho_water rho_amb_air c_d C_D A_t A_b p_air_initial vol_air_initial y p_ambient vol_bottle l_s g temperature_air_initial m_air_initial R theta_rad Z_0 X_0];

% Initial State of Rocket
initial_state = [Z_0 X_0 mass_initial m_air_initial vol_air_initial V_Z_0 V_X_0];

%% ODE SOLVER

% Set Terminal Condition (i.e. Rocket hit ground)
terminalCond = odeset('Events', @hitGround);

% Propagate Flight
[t,state] = ode45(@(t,s) flightProp(t,s,vars), tspan, initial_state, terminalCond);

%% Verification Plots
if (verification_Plots == true)
    F_thrust = f_thrust(state,vars);
    % Force of Thrust Plot
    f1 = figure;
    plot(t(F_thrust(t < 0.45) < 1500),F_thrust(F_thrust(t < 0.45) < 1500));
    xlabel("Time (s)");
    ylabel("Thrust (N)");
    title("Force of Thrust Curve");
end

%% Plot
% Position Plot
f2 = figure;
hold on
plot(state(:,2),state(:,1));
plot(state(:,2),zeros(length(state(:,2)),1),'g--') % Ground Line (Set at Z=0)
xlabel("X Position");
ylabel("Z Position");
title("Z with respect to X");
hold off

% Z Velocity Plot
f3 = figure;
plot(t,state(:,6));
xlabel("Time (s)");
ylabel("V_z");
title("Velocity in the Z Direction");

% X Velocity Plot
f4 = figure;
plot(t,state(:,7));
xlabel("Time (s)");
ylabel("V_x");
title("Velocity in the X Direction");

fprintf('Max Height: %f\n', max(state(:,1)));
fprintf('Max Distance (X): %f\n', max(state(:,2)));


%% Function Definitions

% Flight Propagation
%
% The function that is recursively called by ODE45 and propagates flight
% parameters according to the initial condition passed through by ODE45.
%
% @param t The input time 
% @param state_i The input state matrix
% @param vars The constants vector that holds all relevant constants
%
function state = flightProp(t,state_i,vars) 
    % Define Variables
    rho_water = vars(1);            % Density of Water [kg/m^3]
    rho_amb = vars(2);              % Density of Air [kg/m^3]
    c_d = vars(3);                  % Discharge Coefficient
    C_D = vars(4);                  % Drag Coefficient
    A_t = vars(5);                  % Area of the Throat [m^2]
    A_b = vars(6);                  % Area of the Throat [m^2]
    p_air_initial = vars(7);        % Initial Pressure of Air [N/m^2]
    vol_air_initial = vars(8);      % Initial Volume of Air [m^3]
    y = vars(9);                    % Specific Heat Ratio of Air
    p_ambient = vars(10);           % Ambient Pressure [psi]
    p_ambient = (p_ambient*6894.76); % Convert to [N/m^2]
    vol_bottle = vars(11);          % Volume of Bottle [m^3]
    l_s = vars(12);                 % Length of Test [m]
    g = vars(13);                   % Gravitational Acceleration Constant [m/s^2]
    temperature_air_initial = vars(14); % Temperature of Air [degrees K]
    m_air_initial = vars(15);       % Initial Mass of the Air [kg]
    R = vars(16);                   % Gas Constant
    launch_angle = vars(17);        % Angle in Radians of Rocket Initially
    Z_0 = vars(18);                 % Initial Z Position of Rocket
    X_0 = vars(19);                 % Initial X Position of rocket
    
    % Define State Vector
    state = zeros(7,1);             % Vector of 7 elements
    Z = state_i(1);                 % Vertical Distance of Rocket [m]
    X = state_i(2);                 % Horizontal Distance of Rocket [m]
    m_r = state_i(3);               % Mass of Rocket [kg]
    m_air = state_i(4);             % Mass of the Air [kg]
    v = state_i(5);                 % Volume of Air [m^3]
    V_z = state_i(6);               % Velocity in the Z direction [m/s]
    V_x = state_i(7);               % Velocity in the X direction [m/s]
    
    % Water Expulsion Phase
    if (v < vol_bottle)
        % Calculate rate of change of Volume over time
        v_dot = c_d*A_t*sqrt((2/rho_water)*(p_air_initial*((vol_air_initial/v)^y)-p_ambient));
        % Calculate the pressure in the bottle over time
        p_air = p_air_initial*(vol_air_initial./v).^y;
        % Calculate the force of thrust from water expulsion over time
        F_thrust = 2*c_d*A_t*(p_air-p_ambient);
        % Calculate the change in mass of the rocket over time
        m_dot_r = (-c_d)*A_t*sqrt(2*rho_water*(p_air-p_ambient));
        % Calculate the change in mass of the air over time
        m_dot_a = 0;
    else
        % Pressure of Air at end of Water Expulsion Phase
        p_end = p_air_initial*(vol_air_initial/vol_bottle)^y;
        % Temperature of Air at end of Water Expulsion Phase
        t_end = temperature_air_initial*(vol_air_initial/vol_bottle)^(y-1);
        % Calculate Pressure
        p_air = p_end*(m_air./m_air_initial).^y;
    end
    
    % Gas Expulsion Phase
    if (v >= vol_bottle) && (p_air > p_ambient)
        % Change in Volume
        v_dot = 0;
        % Calculate Critical Pressure
        p_crit = p_air*(2/(y+1))^(y/(y-1));
        % Determine Air Density
        rho_air = m_air / vol_bottle;
        % Determine Gas Temperature
        T = p_air / (rho_air*R);

        % Determine Flow Type
        if (p_crit > p_ambient) % CHOKED FLOW
            % Determine Exit Temperature
            T_e = (2/(y+1))*T;
            % Determine Exit Velocity
            V_e = sqrt(y*R*T_e);
            % Determine Exit Density of Air
            rho_e = p_crit / (R*T_e);
            % Calculate rate of change of mass of air
            m_dot_a = -c_d*rho_e*A_t*V_e;
            % rate of change of mass of rocket
            m_dot_r = m_dot_a;
            % Define Exit Pressure
            p_e = p_crit;
        elseif (p_crit <= p_ambient) % NOT CHOKED FLOW
            % Determine Exit Mach Number
            M_e = sqrt((((p_air/p_ambient)^((y-1)/y))-1)/((y-1)/2));
            % Determine Exit Temperature
            T_e = T / (1+(((y-1)/2)*(M_e^2)));
            % Determine Exit Velocity
            V_e = M_e*sqrt(y*R*T_e);
            % Determine Exit Density of Air
            rho_e = p_ambient * (R*T_e);
            % Calculate rate of change of mass of air
            m_dot_a = -c_d*rho_e*A_t*V_e;
            % rate of change of mass of rocket
            m_dot_r = m_dot_a;
            % Define Exit Pressure
            p_e = p_ambient;
        end
        % Calculate the force of thrust from air expulsion over time
        F_thrust = (c_d*rho_e*A_t*V_e*V_e) + (p_ambient-p_e)*A_t;
    end
   
    % Ballistic Phase
    if (v >= vol_bottle) && (p_air < p_ambient)
        % Set everything to zero because we have no prop left
        F_thrust = 0;
        m_dot_r = 0;
        m_dot_a = 0;
        v_dot = 0;
    end
    
    % Calculate Velocity of Rocket
    V = sqrt((V_x^2)+(V_z^2));
    
    % Calculate Drag on Rocket
    drag = (1/2)*rho_amb*(V^2)*C_D*A_b;
    
    % Calculate Angle of Rocket
    % Check to see if still on stand...
    if (sqrt((X-X_0)^2+(Z-Z_0)^2) <= l_s)
        % At launch pad heading if still on stand
        H_x = cos(launch_angle);
        H_z = sin(launch_angle);
    else
        H_x = V_x/sqrt(V_x^2+V_z^2);
        H_z = V_z/sqrt(V_x^2+V_z^2);
    end
    
    % Calculate Acceleration
    accel_z = (F_thrust*H_z - drag*H_z - (m_r*g))/m_r;        % Acceleration in Z Direction
    accel_x = (F_thrust*H_x - drag*H_x + 0)/m_r;              % Acceleration in X Direction
    
    % Output State 
    state(1) = V_z;                                 % Velocity in Z direction
    state(2) = V_x;                                 % Velocity in X Direction
    state(3) = m_dot_r;                             % Rate of Change of Mass of Rocket
    state(4) = m_dot_a;                             % Rate of Change of Mass of Air
    state(5) = v_dot;                               % Rate of Change of Volume of Air
    state(6) = accel_z;                             % Rate of Change of Velocity in the Z Direction
    state(7) = accel_x;                             % Rate of Change of Velocity in the X Direction
end

% F_thrust
%
% The function takes the ODE45 Flight Propagation state matrix that was output
% and calculates the force of thrust for plotting.
%
% @param state The state matrix from the ODE45 Flight Propagation call
% @param vars The constants vector that holds all relevant constants
%
function out = f_thrust(state,vars)
    % Define Variables
    c_d = vars(3);                  % Discharge Coefficient
    A_t = vars(5);                  % Area of the Throat [m^2]
    p_air_initial = vars(7);        % Initial Pressure of Air [N/m^2]
    vol_air_initial = vars(8);      % Initial Volume of Air [m^3]
    y = vars(9);                    % Specific Heat Ratio of Air
    p_ambient = vars(10);           % Ambient Pressure [psi]
    p_ambient = (p_ambient*6894.76); % Convert to [N/m^2]
    vol_bottle = vars(11);          % Volume of Bottle [m^3]
    m_air_initial = vars(15);       % Initial Mass of the Air [kg]
    R = vars(16);                   % Gas Constant
    
    % Iterate through state matrix data
    for i = 1:size(state,1)
        % Get important values from state matrix
        m_air = state(i,4);             % Mass of the Air [kg]
        v = state(i,5);                 % Volume of Air [m^3]
        
        % Water Expulsion Phase
        if (v < vol_bottle)
            % Calculate the pressure in the bottle over time
            p_air = p_air_initial*(vol_air_initial./v).^y;
            % Calculate the force of thrust from water expulsion over time
            F_thrust = 2*c_d*A_t*(p_air-p_ambient);
        else
            % Pressure of Air at end of Water Expulsion Phase
            p_end = p_air_initial*(vol_air_initial/vol_bottle)^y;
            % Calculate Pressure
            p_air = p_end*(m_air./m_air_initial).^y;
        end

        % Gas Expulsion Phase
        if (v >= vol_bottle) && (p_air > p_ambient)
            % Calculate Critical Pressure
            p_crit = p_air*(2/(y+1))^(y/(y-1));
            % Determine Air Density
            rho_air = m_air / vol_bottle;
            % Determine Gas Temperature
            T = p_air / (rho_air*R);

            % Determine Flow Type
            if (p_crit > p_ambient) % CHOKED FLOW
                % Determine Exit Temperature
                T_e = (2/(y+1))*T;
                % Determine Exit Velocity
                V_e = sqrt(y*R*T_e);
                % Determine Exit Density of Air
                rho_e = p_crit / (R*T_e);
                % Define Exit Pressure
                p_e = p_crit;
            elseif (p_crit <= p_ambient) % NOT CHOKED FLOW
                % Determine Exit Mach Number
                M_e = sqrt((((p_air/p_ambient)^((y-1)/y))-1)/((y-1)/2));
                % Determine Exit Temperature
                T_e = T / (1+(((y-1)/2)*(M_e^2)));
                % Determine Exit Velocity
                V_e = M_e*sqrt(y*R*T_e);
                % Determine Exit Density of Air
                rho_e = p_ambient * (R*T_e);
                % Define Exit Pressure
                p_e = p_ambient;
            end
            % Calculate the force of thrust from air expulsion over time
            F_thrust = (c_d*rho_e*A_t*V_e*V_e) + (p_ambient-p_e)*A_t;
        end

        % Ballistic Phase
        if (v >= vol_bottle) && (p_air < p_ambient)
            % Set everything to zero because we have no prop left
            F_thrust = 0;
        end
        % Output the Force of Thrust
        out(i) = F_thrust;
    end
end

% hitGround
%
% The conditional stop function that checks the state matrix such that if
% the condition Z < 0ft is met the integration is terminated (i.e. hit the ground)
%
% @param t The input time
% @param state The input state matrix 
%
function [value, isterminal, direction] = hitGround(t, state)
    value      = (state(1) < 0);
    isterminal = 1;   % Stop the integration
    direction  = 0;
end
