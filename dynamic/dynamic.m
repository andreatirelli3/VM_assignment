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