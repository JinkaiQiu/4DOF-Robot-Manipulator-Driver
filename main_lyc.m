% MAE 263A Project
% Simulation
clc;close all;clear;
global c
% Main
% Parameter Claim
L1 = 0.10; % m
L2 = 0.13; % m
L3 = 0.13; % m
L4 = 0.06; % m
c = [L1 L2 L3 L4]; % parameters specification

% Mode = 1; %Mode 1: Horizontal Tool
% home = [L4,0,L1+L2+L3].';
Mode = 2; %Mode 2: Vertical Tool
home = [L3,0,L1+L2-L4].';

% Trajectory Cartesian Space
switch Mode
    case 1
        N = 101;
        t = linspace(0,2*pi,N);
        
        % 3D Elipsoid
        a = 0.05; u_vec = [1 1 1]; u_unit = u_vec / norm(u_vec);
        b = 0.025; v_vec = [1 0 -1]; v_unit = v_vec / norm(v_vec);
        center = [0.15, 0, 0.08].';
        x = center(1) + a * cos(t) * u_unit(1) + b * sin(t) * v_unit(1);
        y = center(2) + a * cos(t) * u_unit(2) + b * sin(t) * v_unit(2);
        z = center(3) + a * cos(t) * u_unit(3) + b * sin(t) * v_unit(3);
        
        P_start = [x(1) y(1) z(1)].';
        [x1, y1, z1] = homegoto(P_start,home,N);
        x = [x1 x]; y = [y1 y]; z = [z1 z];
        
    case 2
        N = 101;
        t = linspace(0,2*pi,N);
        
        % 3D Line
        a = 0.05; n_vec = [1 1 -1]; n_unit = n_vec / norm(n_vec);
        center = [0.15 0 0.07];
        x = center(1) + a * sin(t) * n_unit(1);
        y = center(2) + a * sin(t) * n_unit(2);
        z = center(3) + a * sin(t) * n_unit(3);
        
        P_start = [x(1) y(1) z(1)].';
        [x1, y1, z1] = homegoto(P_start,home,N);
        x = [x1 x]; y = [y1 y]; z = [z1 z];

end

% Joint Space
theta1 = zeros(1,N); theta2 = zeros(1,N); theta3 = zeros(1,N); theta4 = zeros(1,N);
for i = 1:N
    p = [x(i) y(i) z(i)]';
    [theta1(i),theta2(i),theta3(i),theta4(i)] = IK_lyc(p, Mode);
end

theta1 = unwrap(theta1); % Remove discontinuity beyond [0,2*pi]
theta2 = unwrap(theta2);
theta3 = unwrap(theta3);
theta4 = unwrap(theta4);

joint = [theta1;theta2;theta3;theta4];
path = [x;y;z];

movie = 0; % create movie if 1
speed = 1; % frame number 1~N

figure(1)
animation_lyc(joint,path,movie,speed,Mode)
