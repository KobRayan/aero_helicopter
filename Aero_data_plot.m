% Aero_data_plot.m
% H. Garnier 
% 21/01/2025


close all

clear;
fontsize=12;

% Load the data file
% Change the name according to your data

load aero_resp_from_pitch_step_HG
%load aero_resp_from_yaw_step_HG


t=step_rsp(1,:);
u_p=step_rsp(2,:); % Voltage to the pitch motor
theta_p=step_rsp(4,:);
theta_y=step_rsp(5,:);
u_y=step_rsp(3,:); % Voltage to the yaw motor
theta_p_dot=step_rsp(6,:);
theta_y_dot=step_rsp(7,:);

figure
subplot(5,1,1)
plot(t,theta_p,'b','LineWidth',2),grid
legend('Pitch angle (°)')
set(gca,'FontSize',fontsize)
set(findall(gcf,'type','text'),'FontSize',fontsize)
subplot(5,1,2)
plot(t,theta_y,'r','LineWidth',2),grid
legend('Yaw angle (°)')
set(gca,'FontSize',fontsize)
set(findall(gcf,'type','text'),'FontSize',fontsize)
subplot(5,1,3)
plot(t,theta_p_dot,'b','LineWidth',2),grid
legend('Pitch angular velocity ')
set(gca,'FontSize',fontsize)
set(findall(gcf,'type','text'),'FontSize',fontsize)
subplot(5,1,4)
plot(t,theta_y_dot,'r','LineWidth',2),grid
legend('Yaw angular velocity')
set(gca,'FontSize',fontsize)
set(findall(gcf,'type','text'),'FontSize',fontsize)
subplot(5,1,5)
plot(t,u_p,t,u_y,'LineWidth',2),grid
legend('Pitch motor voltage', 'Yaw motor voltage')
xlabel('Time (sec)')
set(gca,'FontSize',fontsize)
set(findall(gcf,'type','text'),'FontSize',fontsize)
