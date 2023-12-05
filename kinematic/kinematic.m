clear all, close all, clc

%% Simulation parameters
%  Movement parameteres
v = 30 * 1000 / 3600;   % Vehicle speed in m/s

%  Signal parameters
freq = 0.5;             % Frequency of sinusoidal steering input in Hz
amp_steering = 10;      % Amplitude of sinusoidal steering input in rad

%  Time parameteres
t = 0;          % Simulation time start
t_end = 10;     % Simulation time end
dt = 0.1;       % Simulation time step

%% Vehicle parameteres
%  Physical parameteres
mass = 2164;        % Vehicle mass in kg
%  Forces parameteres
inertia = 4373;     % Inertia in kg*m^2
%  Geometry parameteres
l_f = 1.3384;       % Distance from the center of mass to front axle
l_r = 1.6456;       % Distance from the center of mass to rear axle
%  Constant parameteres
C_f = 1.0745e5;     % Front cornering stiffness coefficient
C_r = 1.9032e5;     % Rear cornering stiffness coefficient

%% Vehicle model
%  Kinematic vehicle model
%  Initialization parameters

%  * Position params
X = 0;      % Position X of the vehicle in the global frame
Y = 0;      % Position Y of the vehicle in the global frame
psi = 0;    % Orientation angle of the vehicle
%  * Storage params
v_x = v;    % Longitudinal velocity in m/s
%   + The assumption is 0 slip angle at t = 0, then only the
%     front angle will update, s_r will remain at 0.
s_f = 0;    % Front slip angle, in rad
s_r = 0;    % Rear slip angle, in rad
beta = 0;   % Steering angle, in rad
course = 0; % Course angle, in rad

% State vectors
x = [X; Y; psi];        % Position state
u = [v_x, s_f, s_r];    % Input singals state
%  * Other storage support struct
global_u = [u(1)'; u(2)'; u(3)'];
global_b = [beta'];
global_c = [course'];

%% Support anonymous fun
%  Calculate the slip angle in relation of the time (t)
slip = @(t) deg2rad(30 * sin(2 * pi * freq * t));
% slip = @(t) deg2rad(30*pi/180*sin(freq*pi*2*t));
%  Load the external function
xdot = @xdot;

%% Simulation
while t <= t_end
    % Update u with the give input signal of
    % steering angle
    u = [v_x, slip(t), 0];

    % Update beta with the calculated steering
    % angle and calculate the slip angle
    beta = atan((l_f*tan(u(3))+l_r*tan(u(2)))/(l_f+l_r));

    % Kinematic model ODE's functions
    [tsol, xsol] = ode45(@(t,x) xdot(t, x, u, beta, l_f, l_r), [t t+dt], x(:,end));
    x = [x xsol(end,:)'];
    
    global_u = [global_u u'];
    global_b = [global_b beta'];

    % Update the course angle
    course = beta+x(3, end);
    global_c = [global_c, course'];
    
    % Move to the next time step
    t = t + dt; 
end

%% Plot the result
figure;

subplot(3, 2, 1);
plot(x(1,1:101),x(2,1:101), 'r-', 'LineWidth', 1.5);
title('Vehicle Position in the global frame');
xlabel('X Position (m)');
ylabel('Y Position (m)');
grid on;

