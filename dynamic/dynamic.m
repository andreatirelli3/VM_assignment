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
%  Dynamic model
%  Initialization parameters
%  * Easy vars name for the forumals
v_x = v;
i_x = inertia;

%  * Speed dependent vars in the matrix A
Ay_dot_dot_2 = -((2*C_f + 2*C_r) / (mass*v_x));
Ay_dot_dot_4 = -v_x - ((2*C_f*l_f - 2*C_r*l_r) / (mass*v_x));

Apsi_dot_dot_2 = -((2*C_f*l_f - 2*C_r*l_r) / (i_x*v_x));
Apsi_dot_dot_4 = -((2*C_f*(l_f^2) + 2*C_r*(l_r^2)) / (i_x*v_x));

%  * Speed dependent vars in the matrix B
By_dot_dot = (2*C_f) / mass;        % We use only the coefficents and
Bpsi_dot_dot = (2*l_f*C_f) / mass;  % constants of Front tire because the
                                    % the assumtion is Rear = 0 slip angle.
                                    % In more easy term, the vehicle is not
                                    % a 4x4.

%  Dynamic model matrix A, B
A = [0 1 0 0;
    0 Ay_dot_dot_2 0 Ay_dot_dot_4;
    0 0 0 1;
    0 Apsi_dot_dot_2 0 Apsi_dot_dot_4];

B = [0; By_dot_dot; 0; Bpsi_dot_dot];

%  State vectors
%  * Support vars
y = 0;
y_dot = 0;
psi = 0;
psi_dot = 0;
delta = 0;

%  * States vectors
x = [y; y_dot; psi; psi_dot];       % Lateral position state (v_x depend)
u = delta;                          % Input singal state (steering angle)

%  * Position in the global fram
X = 0;                              % Coordinate X in the global frame
Y = 0;                              % Coordinate Y in the global frame
global_frame_pos = [X; Y];          % Global frame position (X,Y)

%  * Other storage support struct
global_u = u;
global_c = 0;
global_sf = 0;
global_sr = 0;
global_vs = 0;

%% Support anonymous fun
%  Calculate the slip angle in relation of the time (t)
slip = @(t) deg2rad(30 * sin(2 * pi * freq * t));
%  Load the external function
xdot = @xdot;

%% Simulation
while t<=t_end
    u = slip(t);
    [tsol, xsol] = ode45(@(t,x) xdot(x, u, A, B), [t t+dt], x(:,end));
    x = [x xsol(end,:)'];
    
    v_s = 
    course_f = x(3, end) +
end

