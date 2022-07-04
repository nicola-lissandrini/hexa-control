
% clc
% clear all

basic_time_step = 0.005;
visualization_time_step=0.04;

                              
alpha = 20.5*pi/180; %[rad];
beta=-10*pi/180;

% kf  = 6.3454e-04;% N/Hz^2   /   1.6073*10^(-5); % N/omega^2
% kd  = 1.1049e-05;% Nm/Hz^2  /   2.7988*10^(-7); % Nm/omega^2

kf  = 9.9016*10^(-4)*0.9; % N/Hz^2
kd  =  1.9*10^(-5); % Nm/Hz^2

g = 9.81; % m/s^2

l = 0.435; %Arm length
m = 0.395 + 1.870;%1.775; %kg meassured including small battery / 1.540 kg without battery / 1.909 kg with big battery

wrench_offset=[0.6 ; 0; 0;    0.1; 0.15; 0]; %compensate position tracking error


I_sys=[0.075 0 0;0 0.075 0;0 0 0.15];

init_vel=[0 0 0 0 0 0]; % m/s
init_pos=[0 0 0]; % m
init_orient=[1 0 0; 0 1 0;0 0 1];

w_init=[1 1 1 1 1 1]*2.6937e+05; %rad/s

IMU_PositionOffset=[0 0 0.05]; %Position offset [m] of IMU with respect to COM

motion_capture_rate=0.01;


%===========================================================%
J = diag([82973.961,81490.708,157588.499]*10^(-6)); 

init_vel = zeros(6,1); % initial condition for the integrator
p0 = [0;0;0.75]; % initial condition for the integrator
R0 = eye(3); % initial condition for the integrator

% Matrix gains for the Controller
K_p_scalar = 7;%26;
K_v_scalar = 2;%4*sqrt(K_p_scalar);

K_R_scalar = 4;
K_w_scalar = 0.5;%4*sqrt(K_R_scalar);


K_p = K_p_scalar*eye(3);
K_v = K_v_scalar*eye(3);
K_R = K_R_scalar*eye(3);
K_w = K_w_scalar*eye(3);

k_R_filt_first = 15;

e_3 = [0 0 1]';
F1 = [kf * R_z(         0) * R_x(-alpha) * R_y(beta)*e_3 ...
      kf * R_z(1 * 2*pi/6) * R_x( alpha) * R_y(beta)*e_3 ...
      kf * R_z(2 * 2*pi/6) * R_x(-alpha) * R_y(beta)*e_3 ...
      kf * R_z(3 * 2*pi/6) * R_x( alpha) * R_y(beta)*e_3 ...
      kf * R_z(4 * 2*pi/6) * R_x(-alpha) * R_y(beta)*e_3 ...
      kf * R_z(5 * 2*pi/6) * R_x( alpha) * R_y(beta)*e_3];
  
F2 = [ kd * R_z(         0)*R_x(-alpha)*R_y(beta)*e_3 + cross(R_z(         0)*[l; 0; 0], kf * R_z(         0)*R_x(-alpha)*R_y(beta)*e_3) ...
      -kd * R_z(1 * 2*pi/6)*R_x( alpha)*R_y(beta)*e_3 + cross(R_z(1 * 2*pi/6)*[l; 0; 0], kf * R_z(1 * 2*pi/6)*R_x( alpha)*R_y(beta)*e_3) ...
       kd * R_z(2 * 2*pi/6)*R_x(-alpha)*R_y(beta)*e_3 + cross(R_z(2 * 2*pi/6)*[l; 0; 0], kf * R_z(2 * 2*pi/6)*R_x(-alpha)*R_y(beta)*e_3) ...
      -kd * R_z(3 * 2*pi/6)*R_x( alpha)*R_y(beta)*e_3 + cross(R_z(3 * 2*pi/6)*[l; 0; 0], kf * R_z(3 * 2*pi/6)*R_x( alpha)*R_y(beta)*e_3) ...
       kd * R_z(4 * 2*pi/6)*R_x(-alpha)*R_y(beta)*e_3 + cross(R_z(4 * 2*pi/6)*[l; 0; 0], kf * R_z(4 * 2*pi/6)*R_x(-alpha)*R_y(beta)*e_3) ...
      -kd * R_z(5 * 2*pi/6)*R_x( alpha)*R_y(beta)*e_3 + cross(R_z(5 * 2*pi/6)*[l; 0; 0], kf * R_z(5 * 2*pi/6)*R_x( alpha)*R_y(beta)*e_3)];
  
F = [F1; F2];

function Rx=R_x(alpha)
    Rx = [1 0 0; 0 cos(alpha) -sin(alpha); 0 sin(alpha) cos(alpha)];
end

function Ry=R_y(beta)
    Ry = [cos(beta) 0 sin(beta); 0 1 0; -sin(beta) 0 cos(beta)];
end

function Rz=R_z(theta)
    Rz = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1];
end





