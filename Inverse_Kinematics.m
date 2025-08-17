% ============================
% INVERSE KINEMATICS FOR YOUR ROBOT
% ============================

clc;
clear;
close all;

% Load Robotics Toolbox (Peter Corke)
% Make sure the Robotics Toolbox is installed and added to your MATLAB path

% --- Define Links using MODIFIED DH parameters
L(1) = Link('d', 60, 'a', 3, 'alpha', deg2rad(90));%, 'modified');
L(2) = Link('d', 0,  'a', 85, 'alpha', deg2rad(0));%,  'modified');
L(3) = Link('d', -43, 'a', 0,  'alpha', deg2rad(90));%, 'modified');
L(4) = Link('d', 82, 'a', 3,  'alpha', deg2rad(90));%, 'modified');
L(5) = Link('d', 20, 'a', 10, 'alpha', deg2rad(90));%, 'modified');
L(6) = Link('d', 26, 'a', 0,  'alpha', deg2rad(0));%, 'modified');

d1 = 60;    a1 = 3;     alpha1 = pi/2;
d2 = 0;     a2 = 85;    alpha2 = 0;
d3 = -43;   a3 = 0;     alpha3 = pi/2;
d4 = 82;    a4 = 3;     alpha4 = pi/2;
d5 = 20;    a5 = 10;    alpha5 = pi/2;
d6 = 26;    a6 = 0;     alpha6 = 0;

L(5).offset = pi/2;
L(6).offset = pi/2;

% --- Create the SerialLink robot
robot = SerialLink(L, 'name', 'My6DOFRobot');

% --- Plot the robot in zero position
figure(1);
robot.plot([pi/2 pi/2 pi/2 pi/2 pi/2 pi/2]);
pause;
title('Robot Initial Configuration');

% ============================
% Define Desired End-Effector Pose
% ============================

% Example: desired position (x, y, z) in mm
% and orientation (roll, pitch, yaw) in radians
x = 60;  % mm
y = 80;  % mm
z = 180;  % mm

roll = 5*(pi/180);        % rad
pitch = 63*(pi/180);    % rad
yaw = 127*(pi/180);         % rad

R4 = rpy2tr(roll, pitch, yaw);
Tran = transl(x, y, z);

% Create the transformation matrix
T_desired = Tran * R4;

disp('Desired End-Effector Pose:');
disp(T_desired);

% ============================
% Inverse Kinematics Solution
% ============================

% Solve inverse kinematics numerically
% Use mask [1 1 1 1 1 1] for full 6D pose (position + orientation)
% Provide an initial guess if needed
%initial guess should be same as that od ideal spherical manipulator with
%spherical wrist
R = [
    R4(1,1) R4(1,2) R4(1,3)
    R4(2,1) R4(2,2) R4(2,3)
    R4(3,1) R4(3,2) R4(3,3)
];
P = [
    Tran(1,4)
    Tran(2,4)
    Tran(3,4)
    ];

P_0c = P - d6*R*[0 0 1]';

r = sqrt(P_0c(1)^2 + (P_0c(2))^2);
d = abs(d3);
q1 = atan2((P_0c(2)), P_0c(1)) - atan2(d, sqrt(r^2 - d^2) + a1);
%if (q1 < 0)
%    q1 = pi + q;
%end
s = (P_0c(3) - d1);
D = (r^2  + s^2 - a2^2 - d4^2) / (2*a2*d4);
q3 = atan2(sqrt(1 - D^2), D);  %choosing the reach from below case; +ve angle
q2 = atan2(s, r) - atan2(d4*sin(q3), a2 + d4*cos(q3));

A1 = [
    cos(q1) -sin(q1)    0 
    sin(q1) cos(q1)     0
    0       0           1
    ]*[
    ];

R_03 = [
    1
    0
    ];

%initial_guess = [q1 q2 q3 q4 q5 q6];
initial_guess = [q1 q2 (pi/2 + q3) 0 0 0];
disp((180/pi)*initial_guess);

%numericall solve for inverse kinematics for our robot
q_solution = robot.ikine(T_desired, initial_guess, 'mask', [pi/4 pi/4 -pi/2 0 0 0]);

disp('Inverse Kinematics Joint Angles (in radians):');
disp(q_solution);
disp('Inverse Kinematics Joint Angles (in degrees):');
disp(rad2deg(q_solution));

disp('Inverse Kinematics Joint Angles for our Setup (in degrees):');
our_q = rad2deg(q_solution);
disp(our_q);


% ============================
% Verify Solution
% ============================

% Compute forward kinematics with the solution
%% 
T_actual = robot.fkine(q_solution);

disp('Actual End-Effector Pose from FK:');
disp(T_actual);

% Plot robot in the computed configuration with our constraints
figure(2);
robot.plot(deg2rad(our_q));
title('Robot in Computed IK Configuration with our constraints')

