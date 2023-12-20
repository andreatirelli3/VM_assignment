clear all, close all, clc

% Load external paths
root_folder = fileparts(fileparts(pwd));                % project root directory
controller_folder = fileparts(pwd);                     % controllers root directory
common_sym_path = [root_folder, '/common'];             % common directory
model_folder = [root_folder, '/models/src'];            % models directory
road_aligned_folder = [model_folder, '/road_aligned'];  % road aligned directory
pole_placement_folder = [pwd, '/src/pole_placement'];    % pole placement directory


% Add the new path to the MATLAB search path
addpath(common_sym_path);           % common directory
addpath(road_aligned_folder);       % road aligned directory
addpath(pole_placement_folder);     % pole placement directory

% Anonymous function
pole_placement_xdot = @pole_placement_xdot;

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

% Design steering controller K
[K, delta, S, w_n] = design_k(Ts, overshoot_max, A, B);

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
y_step_response = 0;

while t <= t_end
    % Road Aligned integrator to calc d, psi_des(t), x_des(t) and y_des(t)
    [d, psi_des_t, x_des_t, y_des_t] = road_aligned_integrator(v_x, R, t);
    
    % ODE Lateral movement with controller K
    [tsol, xsol] = ode45(@(t, x) pole_placement_xdot(x, A, B, K, B_d, d), [t t+dt], x(:,end));
    x = [x xsol(end,:)'];     % Update the state
    
    % Step response
    y = step_response(t, w_n, delta);

    % Update storages
    heading_t = x(3, end) + psi_des_t;          % PSI
    x_posg = x_des_t - x(1,end)*sin(heading_t); % X global position
    y_posg = y_des_t + x(1,end)*cos(heading_t); % Y global position
    slip_t = (1/v_x)*x(2, end) - x(3, end);     % Slip angle
    course_t = heading_t + slip_t;              % Course angle
    u = steering_angle_u(t, freq);              % Steering angle

    pos_des = [pos_des [x_des_t y_des_t]'];
    pos_global = [pos_global [x_posg y_posg heading_t]'];
    % heading = [heading heading_t'];
    slip_angle = [slip_angle slip_t'];
    course_angle = [course_angle course_t'];
    steering_angle = [steering_angle u'];
    y_step_response = [y_step_response y'];

    % Move to the next instant (t)
    t = t + dt;

    % Change the path after 10s of the simulation
    if t >= 10
        % R = 1000;   % Along a circle
    end 
end

%  Plot
plotter(t_plot, x, pos_global, heading, slip_angle, course_angle, steering_angle, pos_des);

figure
plot(t_plot, y_step_response, 'k-', 'LineWidth', 1.5);
title('Step response');
xlabel('Time (s)');
ylabel('Step response (y(t))');
grid on;
