clear all, close all, clc

%% Vehicle parameters 
%%% Given inputs
mass = 2164;        % Vehicle mass in kg
inertia = 4373;     % Inertia in kg*m^2
l_f = 1.3384;       % Distance from the center of mass to front axle
l_r = 1.6456;       % Distance from the center of mass to rear axle
C_f = 1.0745e5;     % Front cornering stiffness coefficient
C_r = 1.9032e5;     % Rear cornering stiffness coefficient

%% Simulation parameters
%%% Movement parameters
v = 30 * 1000 / 3600;       % Vehicle speed in m/s
freq = 0.5;                 % Frequency of sinusoidal steering input in Hz

%%% Time paramenters
tsim_start = 0;     % Simulation time start
tsim_end = 10;      % Simulation time end
dtsim = 0.01;       % Simulation time step




% Simulation parameters


amp_steering = deg2rad(30); % Amplitude of steering input in radians

% ODE's function for Kinematic model
xdot = v*cos(phi + beta);
ydot = v*sin(phi + beta);
phi = ((v*cos(beta)) / (l_f + l_r))*(tan(C_f) + tan(C_r));
beta = atan((l_f*tan(C_r) + l_r*tan(C_f)) / l_f + l_r);

% Kinematic model
x = [0 0 0]';       % State of the vehicle

% u = [v C_f C_r]';   % Input signal

%% Simulation
while tsim_start <= tsim_end

end