% plot_lqr_AERO.m
% 30/09/2026
% Plot and analysis of a closed-loop LQR test on the Quanser AERO
%
% Logged matrix (9 rows x N samples), in the order of the Mux ports:
%   row 1 : time                                          (s)      Ts = 2 ms
%   row 2 : pitch motor voltage, after the +/-20 V sat.   (V)
%   row 3 : pitch reference (signal generator)            (deg)
%   row 4 : pitch angle measured                          (deg)
%   row 5 : yaw motor voltage, after the +/-20 V sat.     (V)
%   row 6 : yaw reference (signal generator)              (deg)
%   row 7 : yaw angle measured                            (deg)
%   row 8 : pitch motor (propeller) speed                 (rad/s)
%   row 9 : yaw motor (propeller) speed                   (rad/s)
%
% Figures
%   1 : tracking of the references and tracking errors
%   2 : voltages (controller outputs) and propeller speeds
%   3 : zoom on a time window, to see the coupling between the axes
% Command window
%   unit check, saturation, characteristics of every step, coupling

clear;
close all;
clc

%% 0) Settings
fileName   = 'aero_lqr_rsp_fixed2';  % .mat file (without .mat)
satV       = 20;                            % saturation of the voltages in the model (V)
zoomWindow = [12 15];                       % s: here a yaw step while the pitch reference is constant

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
PitchAngle = data(4,:)';            % pitch angle (deg)
YawVolt    = data(5,:)';            % yaw motor voltage (V)
YawRef     = data(6,:)';            % yaw reference (deg)
YawAngle   = data(7,:)';            % yaw angle (deg)
PitchRotor = data(8,:)';            % pitch propeller speed (rad/s)
YawRotor   = data(9,:)';            % yaw propeller speed (rad/s)

Ts = t(2) - t(1);                   % sampling period (s)
fprintf('%s: %d samples, Ts = %.4f s, duration = %.1f s\n\n', fileName, length(t), Ts, t(end));

PitchErr = PitchRef - PitchAngle;   % tracking errors (deg)
YawErr   = YawRef   - YawAngle;

%% 2) Unit check: the smallest change of an angle must be one encoder count
resPitch = 360/2048;                % 0.1758 deg per count
resYaw   = 360/4096;                % 0.0879 deg per count
fprintf('--- Unit check ---\n');
fprintf('Smallest pitch change: %.4f deg (one count = %.4f deg)\n', minStep(PitchAngle), resPitch);
fprintf('Smallest yaw change  : %.4f deg (one count = %.4f deg)\n', minStep(YawAngle),   resYaw);
if abs(minStep(YawAngle) - 2*resYaw) < 1e-6
    fprintf(['  (!) The yaw never changes by a single count: check the yaw gain in Aero_RW.\n' ...
             '      It must use 1024 (2*pi/1024/4), not 512.\n']);
end
fprintf('\n');

%% 3) Figure 1: tracking
figure('Name', 'LQR: tracking', 'Color', 'w');

ax1 = subplot(3,1,1);
plot(t, PitchRef, 'k--', t, PitchAngle, 'b', 'LineWidth', 1.5)
title('Pitch: reference vs measured')
ylabel('\theta (deg)')
legend('Reference', 'Measured', 'Location', 'northeast')
grid on

ax2 = subplot(3,1,2);
plot(t, YawRef, 'k--', t, YawAngle, 'r', 'LineWidth', 1.5)
title('Yaw: reference vs measured')
ylabel('\psi (deg)')
legend('Reference', 'Measured', 'Location', 'northeast')
grid on

ax3 = subplot(3,1,3);
plot(t, PitchErr, 'b', t, YawErr, 'r', 'LineWidth', 1.2)
title('Tracking errors (reference - measured)')
ylabel('Error (deg)')
xlabel('Time (s)')
legend('Pitch', 'Yaw', 'Location', 'northeast')
ylim([-15 15])                      % the large errors right after each step are cut on purpose
grid on

linkaxes([ax1 ax2 ax3], 'x')
xlim([t(1) t(end)])

%% 4) Figure 2: voltages and propeller speeds
figure('Name', 'LQR: voltages and propeller speeds', 'Color', 'w');

bx1 = subplot(3,1,1);
plot(t, PitchVolt, 'b')
title('Pitch motor voltage (controller output)')
ylabel('V_p (V)')
ylim([-satV-2, satV+2])
grid on

bx2 = subplot(3,1,2);
plot(t, YawVolt, 'r')
title('Yaw motor voltage (controller output)')
ylabel('V_y (V)')
ylim([-satV-2, satV+2])
grid on

bx3 = subplot(3,1,3);
plot(t, PitchRotor, 'b', t, YawRotor, 'r')
title('Propeller speeds')
ylabel('\omega (rad/s)')
xlabel('Time (s)')
legend('Pitch rotor', 'Yaw rotor', 'Location', 'northeast')
grid on

linkaxes([bx1 bx2 bx3], 'x')
xlim([t(1) t(end)])

%% 5) Figure 3: zoom on a time window (coupling between the axes)
k = t >= zoomWindow(1) & t <= zoomWindow(2);

figure('Name', 'LQR: zoom', 'Color', 'w');

cx1 = subplot(4,1,1);
plot(t(k), PitchRef(k), 'k--', t(k), PitchAngle(k), 'b', 'LineWidth', 1.5)
title(sprintf('Zoom between %g and %g s', zoomWindow(1), zoomWindow(2)))
ylabel('\theta (deg)')
grid on

cx2 = subplot(4,1,2);
plot(t(k), YawRef(k), 'k--', t(k), YawAngle(k), 'r', 'LineWidth', 1.5)
ylabel('\psi (deg)')
grid on

cx3 = subplot(4,1,3);
plot(t(k), PitchVolt(k), 'b')
ylabel('V_p (V)')
ylim([-satV-2, satV+2])
grid on

cx4 = subplot(4,1,4);
plot(t(k), YawVolt(k), 'r')
ylabel('V_y (V)')
xlabel('Time (s)')
ylim([-satV-2, satV+2])
grid on

linkaxes([cx1 cx2 cx3 cx4], 'x')

%% 6) Same font on every plot
set(findall(0, 'Type', 'axes'), 'FontSize', 12)

%% 7) Numerical summary
fprintf('--- Voltages ---\n');
fprintf('Pitch: saturated %4.1f %% of the time\n', 100*mean(abs(PitchVolt) >= 0.99*satV));
fprintf('Yaw  : saturated %4.1f %% of the time\n\n', 100*mean(abs(YawVolt) >= 0.99*satV));

fprintf('--- Steps (tr = 10-90 %% rise time, error = mean over the last second) ---\n');
stepTable(t, PitchRef, PitchAngle, 'Pitch');
stepTable(t, YawRef,   YawAngle,   'Yaw');

fprintf('\n--- Coupling: how much one axis moves when the OTHER reference steps ---\n');
couplingTable(t, YawRef,   PitchRef, PitchAngle, 'Yaw',   'pitch');
couplingTable(t, PitchRef, YawRef,   YawAngle,   'Pitch', 'yaw');

%% ======================= LOCAL FUNCTIONS =======================
% (in a script, the functions must be at the end of the file)

function q = minStep(x)
% Smallest non-zero change between two samples
    d = abs(diff(x));
    q = min(d(d > 1e-9));
end

function stepTable(t, ref, y, name)
% Rise time, overshoot and steady-state error of every step of the reference
    Ts    = t(2) - t(1);
    edges = [find(diff(ref) ~= 0) + 1; length(t) + 1];
    for s = 1:length(edges) - 1
        k0  = edges(s);
        idx = (k0:edges(s+1) - 1)';
        if t(idx(end)) - t(k0) < 1
            continue                            % step too close to the end of the record: skip
        end
        pre = max(1, k0 - round(0.2/Ts)):k0 - 1;
        y0  = mean(y(pre));                     % angle just before the step
        r1  = ref(k0);                          % new reference
        z   = (y(idx) - y0) / (r1 - y0);        % normalised response: goes from 0 to 1
        tt  = t(idx) - t(k0);
        i10 = find(z >= 0.1, 1);
        i90 = find(z >= 0.9, 1);
        tr  = NaN;
        if ~isempty(i10) && ~isempty(i90)
            tr = tt(i90) - tt(i10);
        end
        OS  = max(0, 100*(max(z) - 1));
        err = r1 - mean(y(idx(tt >= tt(end) - 1)));
        fprintf('%-5s step at %5.1f s: %+5.0f -> %+5.0f deg | tr = %.2f s | overshoot = %4.1f %% | error = %+5.2f deg\n', ...
                name, t(k0), ref(k0-1), r1, tr, OS, err);
    end
end

function couplingTable(t, refA, refB, yB, nameA, nameB)
% Largest deviation of axis B after each step of reference A,
% only when reference B stays constant during that time.
    Ts     = t(2) - t(1);
    stepsA = find(diff(refA) ~= 0) + 1;
    stepsB = find(diff(refB) ~= 0) + 1;
    for k0 = stepsA'
        if any(stepsB == k0)
            continue                            % both references change together: skip
        end
        kEnd = min([stepsA(stepsA > k0); stepsB(stepsB > k0); length(t) + 1]) - 1;
        if t(kEnd) - t(k0) < 1
            continue                            % window too short (end of the record): skip
        end
        pre  = max(1, k0 - round(0.2/Ts)):k0 - 1;
        dev  = yB(k0:kEnd) - mean(yB(pre));
        [~, i] = max(abs(dev));
        stepSize = abs(refA(k0) - refA(k0-1));
        fprintf('%-5s step of %3.0f deg at %5.1f s -> %s moves by up to %+6.2f deg (%.2f deg per deg of step)\n', ...
                nameA, stepSize, t(k0), nameB, dev(i), abs(dev(i))/stepSize);
    end
end