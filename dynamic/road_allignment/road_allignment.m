%% Load external path
%  * Get the current working directory
root_folder = fileparts(pwd);
disp(root_folder);

%  * Construct the new path
new_path = [root_folder, '/utils'];
disp(new_path);

%  * Add the new path to the MATLAB search path
addpath(new_path);

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

t_pos = 0;

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
%  Dynamic model
%  Initialization parameters
%  * Easy vars name for the forumals
v_x = v;
i_z = inertia;

%  * Speed dependent vars (erros => e_n) in the matrix A
e_1_dot_2 = (2*C_f + 2*C_r) / (mass*v_x);
e_1_dot_3 = (2*C_f + 2*C_r) / mass;
e_1_dot_4 = (-2*C_f*l_f + 2*C_r*l_r) / (mass*v_x);

e_2_dot_2 = (2*C_f*l_f - 2*C_r*l_r) / (i_z*v_x);
e_2_dot_3 = (2*C_f*l_f - 2*C_r*l_r) / i_z;
e_2_dot_4 = (2*C_f*(l_f^2) + 2*C_r*(l_r^2)) / (i_z*v_x);

%  * Orientations vars (delta_n) in the matrix B
delta_2 = (2*C_f) / mass;
delta_4 = (2*l_f*C_f) / i_z;

%  * ---
psides_dot_2 = ((-(2*C_f*l_f - 2*C_r*l_r) / (mass*v_x)) - v_x);
psides_dot_4 = (2*C_f*(l_f^2) + 2*C_r*(l_r^2)) / (i_z*v_x);

%  Dynamic lateral model matrix A, B
A = [0 1 0 0;
    0 -e_1_dot_2 e_1_dot_3 e_1_dot_4;
    0 0 0 1;
    0 -e_2_dot_2 e_2_dot_3 -e_2_dot_4];

B = [0; delta_2; 0; delta_4];

d = [0; psides_dot_2; 0; -psides_dot_4];

%% Support anonymous fun
%  Calculate the slip angle in relation of the time (t)
slip = @(t) deg2rad(30 * sin(2 * pi * freq * t));
% TODO: xdot
% TODO: e1e2pos

%% Simulation
while t<=t_end
end