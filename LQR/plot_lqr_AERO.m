% plot_lqr_AERO.m
% 30/09/2026
% Plot and analysis of a closed-loop LQR test on the Quanser AERO
%
% Content of the logged matrix (9 rows x N samples), in the order of the Mux ports:
%   row 1 : time                                        (s)      Ts = 2 ms
%   row 2 : pitch motor voltage (after the +/-20 V sat.) (V)
%   row 3 : pitch reference (signal generator)           (deg)    square wave
%   row 4 : pitch angle measured                         (deg)    see WARNING
%   row 5 : yaw motor voltage (after the +/-20 V sat.)   (V)
%   row 6 : yaw reference (signal generator)             (deg)    square wave
%   row 7 : yaw angle measured                           (deg)    see WARNING
%   row 8 : pitch motor (propeller) speed                (rad/s)
%   row 9 : yaw motor (propeller) speed                  (rad/s)
%
% WARNING - unit problem in this test
%   The Aero_RW block already outputs the angles in DEGREES. In the Simulink
%   model, they go through an R2D block before being logged, so rows 4 and 7
%   are 57.3 times too large (one encoder count reads 10.07 instead of 0.176 deg).
%   The same degree signals also feed the State X block, whose inputs expect
%   RADIANS. This is why the LQR did not work: see the analysis printed at the end.
%   Set angleLoggedTwice = false once the Simulink model is corrected.

% clear;
% close all;
% clc

%% 0) Settings
fileName         = 'aero_lqr_rsp_fixed';  % name of the .mat file (without .mat)
angleLoggedTwice = false;            % true for this test: rows 4 and 7 were converted to deg twice
satV             = 20;              % saturation of the voltages in the model (V)
zoomWindow       = [10 11];         % time window (s) for the zoom on the voltages

%% 1) Load the data and give a name to each row
S    = load(fileName);
fn   = fieldnames(S);
data = S.(fn{1});                   % the To File block saves a single matrix
if size(data,1) > size(data,2)
    data = data';                   % make sure rows = signals
end
if size(data,1) ~= 9
    error('%s has %d rows instead of 9: check the Mux.', fileName, size(data,1));
end

t          = data(1,:)';            % time (s)
PitchVolt  = data(2,:)';            % pitch motor voltage (V)
PitchRef   = data(3,:)';            % pitch reference (deg)
PitchAngle = data(4,:)';            % pitch angle (deg, see WARNING)
YawVolt    = data(5,:)';            % yaw motor voltage (V)
YawRef     = data(6,:)';            % yaw reference (deg)
YawAngle   = data(7,:)';            % yaw angle (deg, see WARNING)
PitchRotor = data(8,:)';            % pitch propeller speed (rad/s)
YawRotor   = data(9,:)';            % yaw propeller speed (rad/s)

Ts = t(2) - t(1);                   % sampling period (s)
fprintf('%s: %d samples, Ts = %.4f s, duration = %.1f s\n\n', fileName, length(t), Ts, t(end));

% Undo the double deg conversion: divide by 180/pi = 57.3
if angleLoggedTwice
    PitchAngle = PitchAngle * pi/180;
    YawAngle   = YawAngle   * pi/180;
end

% Encoder resolution (deg per count), useful to read the zoomed plots
resPitch = 360/2048;                % 0.176 deg
resYaw   = 360/4096;                % 0.088 deg

% What the controller really tried to reach in this test:
% it compared a reference in RADIANS with a measurement in DEGREES, so it
% stopped when (angle in deg) = (reference in rad), i.e. at ref*pi/180 degrees.
PitchSeen = PitchRef * pi/180;      % e.g. 10 deg -> 0.175 deg
YawSeen   = YawRef   * pi/180;      % e.g. 45 deg -> 0.785 deg

%% 2) Figure 1: reference tracking
figure('Name', 'LQR: reference tracking', 'Color', 'w');

ax1 = subplot(2,1,1);
plot(t, PitchRef, 'k--', t, PitchAngle, 'b', 'LineWidth', 1.5)
title('Pitch: reference vs measured')
ylabel('\theta (deg)')
legend('Reference', 'Measured', 'Location', 'northeast')
grid on

ax2 = subplot(2,1,2);
plot(t, YawRef, 'k--', t, YawAngle, 'r', 'LineWidth', 1.5)
title('Yaw: reference vs measured')
ylabel('\psi (deg)')
xlabel('Time (s)')
legend('Reference', 'Measured', 'Location', 'northeast')
grid on

linkaxes([ax1 ax2], 'x')
xlim([t(1) t(end)])

%% 3) Figure 2: zoom on the measured angles (only meaningful with the unit error)
if angleLoggedTwice
    figure('Name', 'LQR: what the controller actually regulated', 'Color', 'w');

    bx1 = subplot(2,1,1);
    stairs(t, PitchAngle, 'b', 'LineWidth', 1.2)
    hold on
    plot(t, PitchSeen, 'm--', 'LineWidth', 1.5)
    hold off
    title('Pitch: measured angle vs reference read in the wrong unit')
    ylabel('\theta (deg)')
    legend('Measured', 'Reference(rad) read as deg', 'Location', 'northeast')
    yticks(resPitch * (-5:5))       % one grid line per encoder count
    grid on

    bx2 = subplot(2,1,2);
    stairs(t, YawAngle, 'r', 'LineWidth', 1.2)
    hold on
    plot(t, YawSeen, 'm--', 'LineWidth', 1.5)
    hold off
    title('Yaw: measured angle vs reference read in the wrong unit')
    ylabel('\psi (deg)')
    xlabel('Time (s)')
    legend('Measured', 'Reference(rad) read as deg', 'Location', 'northeast')
    grid on

    linkaxes([bx1 bx2], 'x')
    xlim([t(1) t(end)])
end

%% 4) Figure 3: control signals and propeller speeds
figure('Name', 'LQR: voltages and propeller speeds', 'Color', 'w');

cx1 = subplot(3,1,1);
plot(t, PitchVolt, 'b')
title('Pitch motor voltage (controller output)')
ylabel('V_p (V)')
ylim([-satV-2, satV+2])
grid on

cx2 = subplot(3,1,2);
plot(t, YawVolt, 'r')
title('Yaw motor voltage (controller output)')
ylabel('V_y (V)')
ylim([-satV-2, satV+2])
grid on

cx3 = subplot(3,1,3);
plot(t, PitchRotor, 'b', t, YawRotor, 'r')
title('Propeller speeds')
ylabel('\omega (rad/s)')
xlabel('Time (s)')
legend('Pitch rotor', 'Yaw rotor', 'Location', 'northeast')
grid on

linkaxes([cx1 cx2 cx3], 'x')
xlim([t(1) t(end)])

%% 5) Figure 4: zoom on one second, to see the switching of the voltages
k = t >= zoomWindow(1) & t <= zoomWindow(2);

figure('Name', 'LQR: zoom on the voltages', 'Color', 'w');

dx1 = subplot(2,1,1);
plot(t(k), PitchVolt(k), 'b', t(k), YawVolt(k), 'r', 'LineWidth', 1.2)
title(sprintf('Voltages between %g and %g s', zoomWindow(1), zoomWindow(2)))
ylabel('Voltage (V)')
legend('Pitch motor', 'Yaw motor', 'Location', 'northeast')
ylim([-satV-2, satV+2])
grid on

dx2 = subplot(2,1,2);
plot(t(k), PitchRotor(k), 'b', t(k), YawRotor(k), 'r', 'LineWidth', 1.2)
title('Propeller speeds: they never have time to reach full speed')
ylabel('\omega (rad/s)')
xlabel('Time (s)')
legend('Pitch rotor', 'Yaw rotor', 'Location', 'northeast')
grid on

linkaxes([dx1 dx2], 'x')

%% 6) Same font on every plot
set(findall(0, 'Type', 'axes'), 'FontSize', 12)

%% 7) Numerical summary in the command window
fprintf('--- Voltages ---\n');
fprintf('Pitch: saturated %4.1f %% of the time, switches sign about %.1f times/s\n', ...
        100*mean(abs(PitchVolt) >= 0.99*satV), signChanges(PitchVolt, t));
fprintf('Yaw  : saturated %4.1f %% of the time, switches sign about %.1f times/s\n\n', ...
        100*mean(abs(YawVolt) >= 0.99*satV), signChanges(YawVolt, t));

fprintf('--- Angles ---\n');
fprintf('Pitch: reference +/-%.0f deg, measured between %.2f and %.2f deg (%d encoder counts)\n', ...
        max(abs(PitchRef)), min(PitchAngle), max(PitchAngle), ...
        round((max(PitchAngle) - min(PitchAngle))/resPitch));
fprintf('Yaw  : reference +/-%.0f deg, measured between %.2f and %.2f deg (%d encoder counts)\n\n', ...
        max(abs(YawRef)), min(YawAngle), max(YawAngle), ...
        round((max(YawAngle) - min(YawAngle))/resYaw));

fprintf('--- Mean angle in each reference step (first second skipped) ---\n');
phaseTable(t, PitchRef, PitchAngle, 'Pitch', angleLoggedTwice);
phaseTable(t, YawRef,   YawAngle,   'Yaw',   angleLoggedTwice);

%% ======================= LOCAL FUNCTIONS =======================
% (in a script, the functions must be at the end of the file)

function f = signChanges(V, t)
% Number of sign changes of V per second
    f = sum(diff(sign(V)) ~= 0) / (t(end) - t(1));
end

function phaseTable(t, ref, meas, name, bug)
% For each constant part of the reference, mean of the measured angle,
% compared with the reference and (if the unit error is present) with
% the value the controller actually aimed at (reference in rad read as deg).
    edges = [1; find(diff(ref) ~= 0) + 1; length(t) + 1];
    for s = 1:length(edges) - 1
        idx = edges(s):edges(s+1) - 1;
        idx = idx(t(idx) >= t(idx(1)) + 1);         % skip the first second
        if isempty(idx), continue; end
        if bug
            fprintf('%-5s %5.1f-%5.1f s: ref %+6.1f deg | measured %+7.3f deg | aimed at %+7.3f deg\n', ...
                    name, t(idx(1)) - 1, t(idx(end)), ref(idx(1)), mean(meas(idx)), ref(idx(1))*pi/180);
        else
            fprintf('%-5s %5.1f-%5.1f s: ref %+6.1f deg | measured %+7.3f deg\n', ...
                    name, t(idx(1)) - 1, t(idx(end)), ref(idx(1)), mean(meas(idx)));
        end
    end
end