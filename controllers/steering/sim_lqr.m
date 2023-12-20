clear all, close all, clc

% Load external paths
root_folder = fileparts(fileparts(pwd));                % project root directory
controller_folder = fileparts(pwd);                     % controllers root directory
common_sym_path = [root_folder, '/common'];             % common directory
model_folder = [root_folder, '/models/src'];            % models directory
road_aligned_folder = [model_folder, '/road_aligned'];  % road aligned directory
lqr_folder = [pwd, '/src/lqr'];                         % LQR directory

% Add the new path to the MATLAB search path
addpath(common_sym_path);           % common directory
addpath(road_aligned_folder);       % road aligned directory
addpath(lqr_folder);                % LQR directory

% Vehicle geomtry
[mass, i_z, l_f, l_r, C_f, C_r] = vehicle_geometry();

% Simulation envirorment
[v_x, freq, amp_steering, t, t_end, dt] = gen_simulation();
R = inf;        % Path radius, start with inf and at t = 10 change to R = 1000
e_start = -2;   % Simulation starting error, 2m

% Road Allignment system matrices A, B, B_d
[A, B, B_d] = road_aligned_matrices(mass, i_z, l_f, l_r, C_f, C_r, v_x);

n = size(A,2);      % number of states
p = size(B_d,1);    % number of measured outputs  ???
m = size(B,2);      % number of inputs

% Rank of the reachability matrix
r = rank(ctrb(A,B));

% Check the reachability of the system
if r ~= n
    error('Error! The system is not reachable.');
end

% Steering constraints
[Ts, overshoot_max] = steering_constraints();

% LQR
[Q, R, K, S, CLP] = lq_regolator(A, B);

disp("LQ regulator Gain");
disp(K)

disp("LQ regolator Poles");
disp(CLP);

% Initial state of the simulation
x = [-2; 0; 0; 0];

% Plot storage
t_plot = 0:dt:t_end+dt;     % Simulation time
pos_des = [0; 0];           % Desire position
pos_global = [0; -2; 0];    % Global position
heading = 0;                % Vehicle heading
slip_angle = 0;             % Slip angle
course_angle = 0;           % Course angle
steering_angle = 0;         % Steering angle
longitudinal_v = 70;        % Longitudinal velocity
acceleration = 0;           % Acceleration
commanded_accel = 0;        % Commanded accel
y_step_response = 0;        % Step response

% Simulation
while t <= t_end
    % Road Aligned integrator to calc d, psi_des(t), x_des(t) and y_des(t)
    [d, psi_des_t, x_des_t, y_des_t] = road_aligned_integrator(v_x, R, t);

end