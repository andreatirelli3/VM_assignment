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

% Vehicle geomtry
[mass, i_z, l_f, l_r, C_f, C_r] = vehicle_geometry();

% Simulation envirorment
[v_x, freq, amp_steering, t, t_end, dt] = gen_simulation();

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