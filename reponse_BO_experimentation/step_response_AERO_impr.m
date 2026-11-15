% step_response_AERO.m
% 30/09/2026
% Plot of an open-loop step response of the Quanser AERO (2-DOF helicopter)
%
% data0 is a 12 x N matrix: row 1 = time (added by the To File block),
% then one row per Mux input, in the order of the Mux ports (top to bottom).
%
%   row  1 : time                            (s)      Ts = 2 ms
%   row  2 : pitch motor current             (A)      HIL analog input a0
%   row  3 : yaw   motor current             (A)      HIL analog input a1
%   row  4 : pitch angle theta, encoder e2   (deg)    360/(512*4) = 0.176 deg per count
%   row  5 : yaw   angle psi,   encoder e3   (deg)    360/(1024*4) = 0.088 deg per count
%   row  6 : pitch rotor (propeller) speed   (rad/s)  tachometer o14000
%   row  7 : yaw   rotor (propeller) speed   (rad/s)  tachometer o14001
%   row  8 : gyroscope X                     (rad/s)  roll, not an axis of the model
%   row  9 : gyroscope Y                     (rad/s)  pitch rate (sign flipped by Gyro Cal)
%   row 10 : gyroscope Z                     (rad/s)  yaw rate
%   row 11 : pitch motor VOLTAGE             (V)      command after the +/-24 V saturation
%   row 12 : yaw   motor VOLTAGE             (V)      command after the +/-24 V saturation
%
% Rows 11-12 are the INPUTS of the system (what we apply).
% Rows 2-12 are measurements or commands logged at Ts = 2 ms.

clear;
close all;
clc

%% 1) Load the data and give a name to each row
fileName = 'data_step_AERO';        % name of the .mat file to plot (without .mat)
S        = load(fileName);          % the To File block saves the variable data0
data0    = S.data0;
if size(data0,1) ~= 12
    error('%s has %d rows instead of 12: check the Mux of the Simulink model.', ...
          fileName, size(data0,1));
end

t            = data0(1,:)';         % time (s)
PitchCurrent = data0(2,:)';         % pitch motor current (A)
YawCurrent   = data0(3,:)';         % yaw motor current (A)
PitchAngle   = data0(4,:)';         % pitch angle theta (deg)
YawAngle     = data0(5,:)';         % yaw angle psi (deg)
PitchRotor   = data0(6,:)';         % pitch rotor speed (rad/s)
YawRotor     = data0(7,:)';         % yaw rotor speed (rad/s)
GyroX        = data0(8,:)';         % gyroscope X (rad/s)
GyroY        = data0(9,:)';         % gyroscope Y (rad/s)
GyroZ        = data0(10,:)';        % gyroscope Z (rad/s)
PitchVoltage = data0(11,:)';        % pitch motor voltage (V)  -> input 1
YawVoltage   = data0(12,:)';        % yaw motor voltage (V)    -> input 2

Ts = t(2) - t(1);                   % sampling period (s)
N  = length(t);                     % number of samples
fprintf('%d samples, Ts = %.4f s, duration = %.2f s\n', N, Ts, t(end));

% Check that rows 11-12 really are the voltages: a command is exact and
% noise-free, so the final values should be exactly the applied voltages.
fprintf('Final pitch voltage = %.3f V, final yaw voltage = %.3f V\n', ...
        PitchVoltage(end), YawVoltage(end));

%% 2) Angular velocities of the body
% The encoders measure angles, so the angular velocities are obtained by
% differentiating them. Differentiating the raw encoder amplifies its
% quantisation steps, so the angle is first smoothed with a 0.1 s Gaussian
% window. smoothdata is zero-phase, so it adds no delay.
% (The old line theta1 = cumsum(PitchEncoder)*Ts integrated an angle,
%  which has no physical meaning, so it was removed.)
win       = round(0.1/Ts);                                        % 0.1 s = 50 samples
PitchRate = gradient(smoothdata(PitchAngle, 'gaussian', win), Ts); % deg/s
YawRate   = gradient(smoothdata(YawAngle,   'gaussian', win), Ts); % deg/s

r2d = 180/pi;                       % rad -> deg conversion for the gyroscope

%% 3) Figure 1: measured signals
figure('Name', ['AERO: signals - ' fileName], 'Color', 'w');

ax0 = subplot(5,1,1);
plot(t, PitchVoltage, 'b', t, YawVoltage, 'r', 'LineWidth', 1.5)
title(['Inputs: motor voltages  (' fileName ')'], 'Interpreter', 'none')
ylabel('Voltage (V)')
legend('Pitch motor', 'Yaw motor', 'Location', 'northeast')
ylim([min([PitchVoltage; YawVoltage]) - 1, max([PitchVoltage; YawVoltage]) + 1])
grid on

ax1 = subplot(5,1,2);
plot(t, PitchCurrent, 'b', t, YawCurrent, 'r', 'LineWidth', 1)
title('Motor currents')
ylabel('Current (A)')
legend('Pitch motor', 'Yaw motor', 'Location', 'northeast')
grid on

ax2 = subplot(5,1,3);
plot(t, PitchAngle, 'b', 'LineWidth', 1.5)
title('Pitch angle \theta (encoder)')
ylabel('\theta (deg)')
grid on

ax3 = subplot(5,1,4);
plot(t, YawAngle, 'r', 'LineWidth', 1.5)
title('Yaw angle \psi (encoder)')
ylabel('\psi (deg)')
grid on

ax4 = subplot(5,1,5);
plot(t, PitchRotor, 'b', t, YawRotor, 'r', 'LineWidth', 1)
title('Rotor (propeller) speeds')
ylabel('Speed (rad/s)')
xlabel('Time (s)')
legend('Pitch rotor', 'Yaw rotor', 'Location', 'northeast')
grid on

linkaxes([ax0 ax1 ax2 ax3 ax4], 'x')    % zooming on one subplot zooms all of them
xlim([t(1) t(end)])

%% 4) Figure 2: angular velocities, encoder derivative vs gyroscope
figure('Name', ['AERO: angular velocities - ' fileName], 'Color', 'w');

bx1 = subplot(3,1,1);
plot(t, PitchRate, 'b', 'LineWidth', 1.5)
hold on
plot(t, GyroY*r2d, 'Color', [0.1 0.6 0.2])
hold off
title('Pitch rate d\theta/dt')
ylabel('deg/s')
legend('Encoder derivative', 'Gyroscope Y', 'Location', 'northeast')
grid on

bx2 = subplot(3,1,2);
plot(t, YawRate, 'r', 'LineWidth', 1.5)
hold on
plot(t, GyroZ*r2d, 'Color', [0.1 0.6 0.2])
hold off
title('Yaw rate d\psi/dt')
ylabel('deg/s')
legend('Encoder derivative', 'Gyroscope Z', 'Location', 'southeast')
grid on

bx3 = subplot(3,1,3);
plot(t, GyroX, t, GyroY, t, GyroZ)
title('Raw gyroscope (3 axes)')
ylabel('rad/s')
xlabel('Time (s)')
legend('X', 'Y', 'Z', 'Location', 'northwest')
grid on

linkaxes([bx1 bx2 bx3], 'x')
xlim([t(1) t(end)])

%% 5) Same font on every plot
set(findall(0, 'Type', 'axes'), 'FontSize', 12, 'FontName', 'Helvetica')