% step_response_plot.m
% 28/09/2026
% Plot of the step response of the AERO

clear;
close all;
clc

% load data_step_AERO
load step_pitch_10V
% data0 = data0;

PitchMotor=data0(2,:)';  % Pitch Motor (A)
YawMotor=data0(3,:)'; % Yaw Motor (A)
PitchEncoder=data0(4,:)';  % Pitch Encoder (deg)
YawEncoder=data0(5,:)';  % Yaw Encoder (deg)
PitchMotorSpeed=data0(6,:)'; % Pitch Motor Speed (rad/s)
YawMotorSpeed=data0(7,:)';  % Yaw Motor Speed (rad/s)
GyroscopeX=data0(8,:)';  % gyroscope x
GyroscopeY=data0(9,:)';  % gyroscope y
GyroscopeZ=data0(10,:)';  % gyroscope z
t=data0(1,:)'; % time
Ts=t(2)-t(1);   % Sampling period in sec

theta1=cumsum(PitchEncoder)*Ts; % angular position estimate in rad
theta2=cumsum(YawEncoder)*Ts; % angular position estimate in rad

figure
subplot(7,1,1)
plot(t,PitchMotor,'r','linewidth',2)
legend('Pitch Motor (A)','location','northwest')
axis([0 t(end) min(PitchMotor) max(PitchMotor)+1])
set(gca,'FontSize',12,'FontName','helvetica');
grid
subplot(7,1,2)
plot(t,PitchEncoder,'r','linewidth',2)
legend('Yaw Motor (A)','location','northwest')
axis([0 t(end) min(PitchEncoder) max(PitchEncoder)+20])
set(gca,'FontSize',12,'FontName','helvetica');
grid
subplot(7,1,3)
plot(t,YawEncoder,'r','linewidth',2)
legend('Pitch Encoder (deg)','location','northwest')
axis([0 t(end) min(YawEncoder) max(YawEncoder)+1])
set(gca,'FontSize',12,'FontName','helvetica');
grid
subplot(7,1,4)
plot(t,YawMotor,'r','linewidth',2)
legend('Yaw Encoder (deg)','location','northwest')
axis([0 t(end) min(YawMotor) max(YawMotor)+20])
set(gca,'FontSize',12,'FontName','helvetica');
grid
subplot(7,1,5)
plot(t,PitchMotorSpeed,'r','linewidth',2)
legend('Pitch Motor Speed (rad/s)','location','northwest')
axis([0 t(end) min(PitchMotorSpeed) max(PitchMotorSpeed)+20])
set(gca,'FontSize',12,'FontName','helvetica');
grid
subplot(7,1,6)
plot(t,YawMotorSpeed,'r','linewidth',2)
legend('Yaw Motor Speed (rad/s)','location','northwest')
axis([0 t(end) min(YawMotorSpeed) max(YawMotorSpeed)+20])
set(gca,'FontSize',12,'FontName','helvetica');
grid
subplot(7,1,7)
plot(t,GyroscopeX,'r','linewidth',2)
hold on;
plot(t,GyroscopeY,'g','linewidth',2)
hold on;
plot(t,GyroscopeZ,'b','linewidth',2)
hold off;
legend('Gyroscope X', 'Gyroscope Y', 'Gyroscope Z','location','northwest')
axis([0 t(end) min(GyroscopeY)*1.2 max(GyroscopeZ)*1.2])
set(gca,'FontSize',12,'FontName','helvetica');
grid