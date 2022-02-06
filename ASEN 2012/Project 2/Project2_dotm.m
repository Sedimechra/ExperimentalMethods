%% ASEN 2012 Project 2
% Tristan Workman, ID: 109326637, last modified 12/6/2020

% Using thermodynamic and aerodynamic relations, and equations of motion, determine 
% the trajectory of a bottle rocket with specified initial properties, and tweak 
% those properties to guarantee an 80m flight.

% Assumptions
% Steady, uniform 1D flow, only 2D required, Ideal weather + Gas,
% Compressible within the bottle (two densities)

g = 9.81; % [m/s^2]
C_discharge = 0.8; % discharge coefficient
rho_amb = 0.961; % kg/m^3 ambient air density
Vol_bottle = 0.002; % m^3 volume of empty bottle
P_amb = 83426.56; % Pa atmospheric pressure
gamma = 1.4; % ratio of specific heats for air
rho_water = 1000; % kg/m^3 density of water
D_throat = 2.1; % cm diameter of the throat
D_bottle = 10.5; % cm diameter of bottle
R = 287; % J/kgK gas constant of air
M_bottle = 0.15; % kg mass of empty 2 litre bottle with cones and fins
T_airI = 300; % K initial temperature of air
v0 = 0.0; % m/s initial velocity of rocket
x0 = 0.0; % m initial horizontal displacement
z0 = 0.25; % m initial vertical displacement
l_s = 0.5; % m length of test stand
tspan = [0 5]; % integration time

A_t = pi * (D_throat/200)^2; % m^2 area of throat
A_B = pi * (D_bottle/200)^2; % m^2 cross sectional area of bottle

% After meeting the verification requirements, I begin to try test cases
% make 1 true and all others false to see associated graph 
% Verification case on by default
verification = true;
testcase = false;
pressureChange = false;
volChange = false;
dragChange = false;
thetaChange = false;
if (verification == true)
    P_gage = 344738; % initial gage pressure of air in bottle
    Vol_waterI = 0.001; % m^3 initial volume of water inside bottle
    C_drag = 0.5; % drag coefficient
    theta = deg2rad(45); % radians initial angle of rocket
    
    % determining rocket mass
    P_airI = (P_gage + P_amb);
    rho_air_bottleI = P_airI ./ (R .* T_airI);
    M_waterI = (Vol_waterI .* rho_water); 
    Vol_airI = (Vol_bottle - Vol_waterI);
    M_airI = (P_airI .* Vol_airI)./(R .* T_airI);
    RMI = M_bottle + M_waterI + M_airI;
    
    X0 = [x0,z0,v0,v0,RMI,M_airI,Vol_airI];
    
    opt = odeset('Events', @stopEvent);
    [t, X] = ode45(@(t,x) odeFun(t,x,g,C_discharge,rho_amb,Vol_bottle,P_amb,gamma,rho_water,A_t,R,C_drag,T_airI,theta,l_s,Vol_airI,P_airI,M_airI,A_B,x0,z0), tspan, X0, opt);
    Force_t = thrust(X,C_discharge,A_t,P_airI,Vol_airI,gamma,P_amb,Vol_bottle,M_airI,R);
    dist = max(X(:,1))
    height = max(X(:,2))
elseif (testcase == true)
    P_gage = 344738 + 80000; % initial gage pressure of air in bottle
    Vol_waterI = 0.0008; % m^3 initial volume of water inside bottle
    C_drag = 0.5; % drag coefficient
    theta = deg2rad(40); % radians initial angle of rocket
    
    % determining rocket mass
    P_airI = (P_gage + P_amb);
    rho_air_bottleI = P_airI ./ (R .* T_airI);
    M_waterI = (Vol_waterI .* rho_water); 
    Vol_airI = (Vol_bottle - Vol_waterI);
    M_airI = (P_airI .* Vol_airI)./(R .* T_airI);
    RMI = M_bottle + M_waterI + M_airI;
    
    opt = odeset('Events', @stopEvent);
    [t, X] = ode45(@(t,x) odeFun(t,x,g,C_discharge,rho_amb,Vol_bottle,P_amb,gamma,rho_water,A_t,R,C_drag,T_airI,theta,l_s,Vol_airI,P_airI,M_airI,A_B,x0,z0), tspan, X0, opt);
    Force_t = thrust(X,C_discharge,A_t,P_airI,Vol_airI,gamma,P_amb,Vol_bottle,M_airI,R);
    dist = max(X(:,1))
    height = max(X(:,2))
elseif (pressureChange == true)
    P_gageV = [243413, 344738, 446063, 547388]; % initial gage pressure of air in bottle
    Vol_waterI = 0.001; % m^3 initial volume of water inside bottle
    C_drag = 0.5; % drag coefficient
    theta = deg2rad(45); % radians initial angle of rocket
    hold off
    for i = 1:length(P_gageV)
        % determining rocket mass
        P_airI = (P_gageV(1,i) + P_amb);
        rho_air_bottleI = P_airI ./ (R .* T_airI);
        M_waterI = (Vol_waterI .* rho_water); 
        Vol_airI = (Vol_bottle - Vol_waterI);
        M_airI = (P_airI .* Vol_airI)./(R .* T_airI);
        RMI = M_bottle + M_waterI + M_airI;
        
        % graphing trajectory with change in initial pressure
        opt = odeset('Events', @stopEvent);
        [t, X] = ode45(@(t,x) odeFun(t,x,g,C_discharge,rho_amb,Vol_bottle,P_amb,gamma,rho_water,A_t,R,C_drag,T_airI,theta,l_s,Vol_airI,P_airI,M_airI,A_B,x0,z0), tspan, X0, opt);
        % Force_t = thrust(X,C_discharge,A_t,P_airI,Vol_airI,gamma,P_amb,Vol_bottle,M_airI,R);
        plot(X(:,1),X(:,2),"LineWidth",1.5)
        title("Bottle Rocket Trajectory with Changing Initial Pressure")
        set(0,'defaultTextInterpreter','latex')
        set(gca,'FontSize',11)
        ylabel("Height [m]")
        xlabel("Distance [m]")
        ylim([0 30])
        grid on
        hold on
    end
    legend("2.4 atm","3.4 atm","4.4 atm","5.4 atm")
elseif (volChange == true)
    P_gage = 344738; % initial gage pressure of air in bottle
    Vol_waterIV = [0.0005 0.0007 0.0009 0.0011]; % m^3 initial volume of water inside bottle
    C_drag = 0.5; % drag coefficient
    theta = deg2rad(45); % radians initial angle of rocket
    hold off
    for i = 1:length(Vol_waterIV)
        % determining rocket mass
        P_airI = (P_gage + P_amb);
        rho_air_bottleI = P_airI ./ (R .* T_airI);
        M_waterI = (Vol_waterIV(1,i) .* rho_water); 
        Vol_airI = (Vol_bottle - Vol_waterIV(1,i));
        M_airI = (P_airI .* Vol_airI)./(R .* T_airI);
        RMI = M_bottle + M_waterI + M_airI;
        
        % graphing trajectory with change in initial water volume
        opt = odeset('Events', @stopEvent);
        [t, X] = ode45(@(t,x) odeFun(t,x,g,C_discharge,rho_amb,Vol_bottle,P_amb,gamma,rho_water,A_t,R,C_drag,T_airI,theta,l_s,Vol_airI,P_airI,M_airI,A_B,x0,z0), tspan, X0, opt);
        % Force_t = thrust(X,C_discharge,A_t,P_airI,Vol_airI,gamma,P_amb,Vol_bottle,M_airI,R);
        plot(X(:,1),X(:,2),"LineWidth",1.5)
        title("Bottle Rocket Trajectory with Changing Initial Water Volume")
        set(0,'defaultTextInterpreter','latex')
        set(gca,'FontSize',11)
        ylabel("Height [m]")
        xlabel("Distance [m]")
        ylim([0 35])
        grid on
        hold on
    end
    legend("0.5 L","0.7 L","0.9 L","1.1 L")
elseif (dragChange == true)
    P_gage = 344738; % initial gage pressure of air in bottle
    Vol_waterI = 0.001; % m^3 initial volume of water inside bottle
    C_drag = [0.3 0.4 0.5 0.6]; % drag coefficient
    theta = deg2rad(45); % radians initial angle of rocket
    hold off
    for i = 1:length(C_drag)
        % determining rocket mass
        P_airI = (P_gage + P_amb);
        rho_air_bottleI = P_airI ./ (R .* T_airI);
        M_waterI = (Vol_waterI .* rho_water); 
        Vol_airI = (Vol_bottle - Vol_waterI);
        M_airI = (P_airI .* Vol_airI)./(R .* T_airI);
        RMI = M_bottle + M_waterI + M_airI;
        
        % graphing trajectory with change in initial water volume
        opt = odeset('Events', @stopEvent);
        [t, X] = ode45(@(t,x) odeFun(t,x,g,C_discharge,rho_amb,Vol_bottle,P_amb,gamma,rho_water,A_t,R,C_drag(1,i),T_airI,theta,l_s,Vol_airI,P_airI,M_airI,A_B,x0,z0), tspan, X0, opt);
        % Force_t = thrust(X,C_discharge,A_t,P_airI,Vol_airI,gamma,P_amb,Vol_bottle,M_airI,R);
        plot(X(:,1),X(:,2),"LineWidth",1.5)
        title("Bottle Rocket Trajectory with Changing Coefficient of Drag")
        set(0,'defaultTextInterpreter','latex')
        set(gca,'FontSize',11)
        ylabel("Height [m]")
        xlabel("Distance [m]")
        ylim([0 25])
        grid on
        hold on
    end
    legend("C_D = 0.3","C_D = 0.4","C_D = 0.5","C_D = 0.6")
elseif (thetaChange == true)
    P_gage = 344738; % initial gage pressure of air in bottle
    Vol_waterI = 0.001; % m^3 initial volume of water inside bottle
    C_drag = 0.5; % drag coefficient
    theta = [deg2rad(35) deg2rad(45) deg2rad(55)]; % radians initial angle of rocket
    hold off
    for i = 1:length(theta)
        % determining rocket mass
        P_airI = (P_gage + P_amb);
        rho_air_bottleI = P_airI ./ (R .* T_airI);
        M_waterI = (Vol_waterI .* rho_water); 
        Vol_airI = (Vol_bottle - Vol_waterI);
        M_airI = (P_airI .* Vol_airI)./(R .* T_airI);
        RMI = M_bottle + M_waterI + M_airI;
        
        % graphing trajectory with change in initial water volume
        opt = odeset('Events', @stopEvent);
        [t, X] = ode45(@(t,x) odeFun(t,x,g,C_discharge,rho_amb,Vol_bottle,P_amb,gamma,rho_water,A_t,R,C_drag,T_airI,theta(1,i),l_s,Vol_airI,P_airI,M_airI,A_B,x0,z0), tspan, X0, opt);
        % Force_t = thrust(X,C_discharge,A_t,P_airI,Vol_airI,gamma,P_amb,Vol_bottle,M_airI,R);
        plot(X(:,1),X(:,2),"LineWidth",1.5)
        title("Bottle Rocket Trajectory with Changing Launch Angle")
        set(0,'defaultTextInterpreter','latex')
        set(gca,'FontSize',11)
        ylabel("Height [m]")
        xlabel("Distance [m]")
        ylim([0 25])
        grid on
        hold on
    end
    legend("35 degrees","45 degrees","55 degrees")
end

% X = [
%   x - x position
%   z - z position
%   Vx - x component of velocity
%   Vz - z component of velocity
%   RM - rocket mass
%   AM - mass of air
%   AV - volume of air
% ]

% Xdot = [
%   vx - velocity in x
%   vz - velocity in z
%   ax - acceleration in x
%   az - acceleration in z
%   RMdot - change in rocket mass
%   AMdot - change in mass of air
%   AVdot - change in volume of air
% ]

if (verification == true || testcase == true)
    % Finding where the phases lie on the trajectory graph
    [~,ind1] = min(abs(t-0.204625));
    [~,ind2] = min(abs(t-0.2544));
    X(ind1,1);
    X(ind2,1);
    
    figure(1)
    plot(X(:,1),X(:,2),"LineWidth",1.5)
    hold on
    title("Bottle Rocket Trajectory")
    set(0,'defaultTextInterpreter','latex')
    set(gca,'FontSize',11)
    % xline(X(ind1,1),"Linewidth",1.5)
    % xline(X(ind2,1),"Linewidth",1.5)
    if (verification == true)
        ylim([0 20]) % for verification case
    elseif (testcase == true)
        ylim([0 25])
    end
    ylabel("Height [m]")
    xlabel("Distance [m]")
    grid on
    hold off 
    
    figure(2)
    plot(t,X(:,4),"LineWidth",1.5)
    hold on
    title("Bottle Rocket Z-Velocity")
    set(0,'defaultTextInterpreter','latex')
    set(gca,'FontSize',11)
    ylabel("Z-Velocity [m/s]")
    xlabel("Time [s]")
    grid on
    hold off
    
    figure(3)
    plot(t,X(:,3),"LineWidth",1.5)
    hold on
    title("Bottle Rocket X-Velocity")
    set(0,'defaultTextInterpreter','latex')
    set(gca,'FontSize',11)
    ylabel("X-Velocity [m/s]")
    xlabel("Time [s]")
    grid on
    hold off
    
    figure(4)
    plot(t((t(:,1) < .25),1),X((t(:,1) < 0.25),7),"LineWidth",1.5)
    hold on
    title("Bottle Rocket Air Volume Evolution")
    set(0,'defaultTextInterpreter','latex')
    set(gca,'FontSize',11)
    ylabel("Air Volume in Bottle Rocket [m$^3$]")
    xlabel("Time [s]")
    ylim([1 * 10^-3, 2 * 10^-3])
    grid on
    hold off
    
    figure(5)
    plot(t((t(:,1) < 1),1),Force_t(1,(t(:,1) < 1)),"LineWidth",1.5)
    hold on
    if (verification == true)
        xline(0.204625,"Linewidth",1.5)
        xline(0.2544,"Linewidth",1.5)
    elseif (testcase == true)
        xline(0.152,"Linewidth",1.5)
        xline(0.197,"Linewidth",1.5)
    end
    title("Thrust vs Time for Bottle Rocket")
    set(0,'defaultTextInterpreter','latex')
    set(gca,'FontSize',11)
    ylabel("Thrust [N]")
    xlabel("Time [s]")
    xlim([0 .45])
    grid on
    hold off
end
%% Function propogation

function [value, isterminal, direction] = stopEvent(t,X)
    value = (X(2) <= 0);
    isterminal = 1;
    direction = 0;
end
function dx = odeFun(t,x,g,C_discharge,rho_amb,Vol_bottle,P_amb,gamma,rho_water,A_t,R,C_drag,T_airI,theta,l_s,Vol_airI,P_airI,M_airI,A_B,x0,z0)
    % must split into three parts, as there are three sections of flight
    % Part 1 is powered (water exhaust), so drag + thrust(water) + gravity
    % Part 2 is also powered (air exhaust), so drag + thrust(air) + gravity
    % Part 3 is unpowered, so just drag + gravity
    xpos = x(1);
    zpos = x(2);
    Vx = x(3);
    Vz = x(4);
    RM = x(5);
    AM = x(6);
    AV = x(7);
    
    % velocity vector is just combination of components
    V = sqrt((Vx.^2) + (Vz.^2));
    
    % Part 1
    if (AV < Vol_bottle) 
        % calculating change in volume of air over time
        AVdot = C_discharge .* A_t .* sqrt((2./rho_water) .* ((P_airI .* (Vol_airI./AV).^gamma) - P_amb));
        % calculating current pressure based on volume
        P_air = P_airI .* (Vol_airI./AV).^gamma;
        % calculating change in mass of the rocket over time
        RMdot = -C_discharge .* A_t .* sqrt(2 .* rho_water .* (P_air - P_amb));
        % with P_air we can now calculate the force from thrust
        F_t = 2 .* C_discharge .* A_t .* (P_air - P_amb);
        % change in air mass is zero because of phase of flight
        AMdot = 0;
    else
        % set pressure and temperature to fixed values after all water is
        % expelled (AV >= Vol_bottle) && (P_air > P_amb)
        % calculating pressure at end of part 1
        P_air_end = P_airI .* (Vol_airI./Vol_bottle).^gamma;
        % calculating temperature at the end of part 1
        T_air_end = T_airI .* (Vol_airI./Vol_bottle).^(gamma-1);
        % Air pressure now changes (no water left in the bottle)
        % calculating new pressure
        P_air = P_air_end .* (AM./M_airI).^gamma;
    end
    
    % Part 2
    if (AV >= Vol_bottle) && (P_air > P_amb)
        % Calculating air density/temperature
        rho_air_bottle = AM./Vol_bottle;
        T_air = P_air ./ (rho_air_bottle .* R);
        % determining whether choked flow, need critical pressure
        P_crit = P_air .* (2 ./ (gamma + 1)).^(gamma./(gamma-1));
        if (P_crit > P_amb)
            % flow is choked, determine exit velocity
            % need temperature at exit
            T_exit = T_air .* (2./(gamma+1));
            % calculating exit velocity
            V_exit = sqrt(gamma .* R .* T_exit);
            % air pressure at exit is just critical pressure
            P_exit = P_crit;
            % calculating air density at exit
            rho_air_exit = P_exit ./ (R .* T_exit);
        elseif (P_crit < P_amb)
            % solve for exit mach speed through pressure relation
            Mach_exit = sqrt((((P_air./P_amb).^((gamma-1)./gamma))-1)./((gamma-1)./2));
            % calculating temperature of air at exit
            T_exit = T_air ./ (1 + (((gamma-1)./2) .* (Mach_exit.^2)));
            % air pressure at exit is just ambient pressure
            P_exit = P_amb;
            % calculating density at exit
            rho_air_exit = P_exit ./ (R .* T_exit);
            % now we can calculate exit velocity
            V_exit = Mach_exit .* sqrt(gamma .* R .* T_exit);
        end
        % thrust, air mass, and rocket mass change by the same functions
        % calculating change in air mass
        AMdot = -C_drag .* rho_air_exit .* A_t .* V_exit;
        % calculating thrust
        F_t = (-AMdot) .* V_exit + ((P_amb - P_exit) .* A_t);
        % Calculating change in rocket mass (same as change in air mass)
        RMdot = AMdot;
        % Air volume is unchanging during this phase
        AVdot = 0;
    end
    
    % Part 3
    if (AV >= Vol_bottle) && (P_air <= P_amb)
        % no water or air exhaust thrust
        F_t = 0;
        AVdot = 0;
        % Change in air mass is zero, as pressure has equalized
        AMdot = 0;
        % change in mass is zero (again no exhaust and pressure equalized)
        RMdot = 0;
    end
    
    % initial heading is determined by test stand
    % so we have to check whether its cleared the stand yet (using trig)
    % if (sqrt(((zpos - z0)^2) + ((xpos - x0)^2)) <= l_s)
    if (xpos < (l_s .* cos(theta) + x0) && zpos < (l_s .* sin(theta) + z0))
        hx = cos(theta);
        hz = sin(theta);
    else
        % once it has cleared the stand, heading is determined by velocity
        % components
        hx = (Vx)./sqrt((Vx.^2) + (Vz.^2));
        hz = (Vz)./sqrt((Vx.^2) + (Vz.^2));
    end
    
    % Since drag force is a simple function of area and velocity, it can
    % be calculated outside of the individual parts
    
    % Calculating drag:
    F_d = (1/2) .* rho_amb .* V^2 .* C_drag .* A_B;
    
    % Now, we have all forces responsible for motion 
    % We can now determine the acceleration components which govern
    % trajectory:
    accelX = ((F_t .* hx) - (F_d .* hx)) ./ RM; 
    accelZ = ((F_t .* hz) - (F_d .* hz) - (RM .* g)) ./ RM;
    
    % final outputs for state vector
    dx = [Vx; Vz; accelX; accelZ; RMdot; AMdot; AVdot];
end
function T = thrust(X,C_discharge,A_t,P_airI,Vol_airI,gamma,P_amb,Vol_bottle,M_airI,R)
    % use state matrix data (X) to extract force of thrust, so it can be graphed and verified
    for k = 1:size(X,1)
        % thrust equations require AM and AV
        AM = X(k,6); % air mass
        AV = X(k,7); % air volume
        % rest of code is copied from ODE function
        % Part 1
        if (AV < Vol_bottle) 
            % calculating current pressure based on volume
            P_air = P_airI .* (Vol_airI./AV).^gamma;
            % with P_air we can now calculate the force from thrust
            F_t = 2 .* C_discharge .* A_t .* (P_air - P_amb);
        else
            % set pressure and temperature to fixed values after all water is
            % expelled (AV >= Vol_bottle) && (P_air > P_amb)
            % calculating pressure at end of part 1
            P_air_end = P_airI .* (Vol_airI./Vol_bottle).^gamma;
            % Air pressure now changes (no water left in the bottle)
            % calculating new pressure
            P_air = P_air_end .* (AM./M_airI).^gamma;
        end
        
        % Part 2
        if (AV >= Vol_bottle) && (P_air > P_amb)
            % Calculating air density/temperature
            rho_air_bottle = AM./Vol_bottle;
            T_air = P_air ./ (rho_air_bottle .* R);
            % determining whether choked flow, need critical pressure
            P_crit = P_air .* (2 ./ (gamma + 1)).^(gamma./(gamma-1));
            if (P_crit > P_amb)
                % flow is choked, determine exit velocity
                % need temperature at exit
                T_exit = T_air .* (2./(gamma+1));
                % calculating exit velocity
                V_exit = sqrt(gamma .* R .* T_exit);
                % air pressure at exit is just critical pressure
                P_exit = P_crit;
                % calculating air density at exit
                rho_air_exit = P_exit ./ (R .* T_exit);
            elseif (P_crit < P_amb)
                % solve for exit mach speed through pressure relation
                Mach_exit = sqrt((((P_air./P_amb).^((gamma-1)./gamma))-1)./((gamma-1)./2));
                % calculating temperature of air at exit
                T_exit = T_air ./ (1 + (((gamma-1)./2) .* (Mach_exit.^2)));
                % air pressure at exit is just ambient pressure
                P_exit = P_amb;
                % calculating density at exit
                rho_air_exit = P_exit ./ (R .* T_exit);
                % now we can calculate exit velocity
                V_exit = Mach_exit .* sqrt(gamma .* R .* T_exit);
            end
            % thrust, air mass, and rocket mass change by the same functions
            % calculating thrust
            F_t = (C_discharge .* rho_air_exit .* A_t .* V_exit) .* V_exit + ((P_amb - P_exit) .* A_t);
        end
    
        % Part 3
        if (AV >= Vol_bottle) && (P_air < P_amb)
            % no water or air exhaust thrust
            F_t = 0;
        end
        T(k) = F_t;
    end
end