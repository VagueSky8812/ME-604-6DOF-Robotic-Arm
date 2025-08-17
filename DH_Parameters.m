clear all;
clc;
%% Setting up the Manipulator
%all dimensions in mm, all angles in radians
Link1 = Link('d', 60, 'a', 3, 'alpha', pi/2);    %defining a link, link 1
%disp(Link1.a);
%disp(Link1.d);
%disp(Link1.alpha);
%disp(Link1.isrevolute);
%disp(Link1.A(pi/2));
Link2 = Link('d', 0, 'a', 85, 'alpha', 0);        %defining link 2
Link3 = Link('d', -43, 'a', 0, 'alpha', pi/2);        %defining link 2
Link4 = Link('d', 82, 'a', 3, 'alpha', pi/2);        %defining link 2
Link5 = Link('d', 20, 'a', 10, 'alpha', pi/2);        %defining link 2
Link6 = Link('d', 26, 'a', 0, 'alpha', 0);        %defining link 2

Link5.offset = pi/2;
Link6.offset = pi/2;

bot = SerialLink([Link1 Link2 Link3 Link4 Link5 Link6], 'name', 'Industrial Manipulator');  %joining these two, in order, to form a serial link robot manipulator
%disp(bot);
%disp(bot.n);    %shows number of joints in the robot

%% Plotting in Zero position
%disp(bot.fkine([0 pi/3])); %supplying the qi's, in order, to find the pose of the robot's end effector
%bot.plot([0 pi/2 0 0 pi 0]);  %plotting a stick figure of our robot in zero position
%bot.plot([pi/2 pi/2 pi/2 pi/2 pi pi]);
bot.plot([pi/2 pi/2 pi/2 pi/2 pi/2 pi/2]);
pause;%press any key to continue

%% Animation
%{
t_step = 0.25;
t_final = 5;
t = 0: t_step : t_final; %generating a time step vector/array
N_tsteps = t_final/t_step + 1;
qz = [0 pi/2 0 0 pi 0];%at zero position
qr = [0 pi/2 pi/2 0 pi 0];%at final position
%Trajectory Generation
q = jtraj(qz, qr, t);% generate joint coordinate trajectory; a series of q's

for i = 1:N_tsteps
    bot.plot(q(i, 1:6));
end
%}