% tune_lqr_AERO.m
% 30/09/2026, updated 05/10/2026 (performance metrics, model error, results saved to .mat,
%                                  second Bryson design)
%             updated 06/10/2026 (iUsed read from the file name; step markers of
%                                  figure 4 work with ramp references)
% Helps tuning the LQR weights Q and R BEFORE trying them on the AERO.
%
% For each (Q, R) design below, the script:
%   1) computes K = lqr(A, B, Q, R) on the Quanser grey-box model
%   2) simulates the closed loop u = K*(xd - x), with the +/-20 V saturation,
%      using the SAME references as a measured test
%   3) computes the same performance metrics for every simulated design AND
%      for the measured test (computeMetrics.m)
%   4) measures how far the simulation is from the measurement (modelError.m)
%   5) saves everything to a .mat file and plots the comparisons
%
% Two kinds of metrics, not to be mixed:
%   - performance (reference vs angle, voltages): works for ANY controller,
%     simulated or measured, because it only uses the logged signals
%   - model error (simulated vs measured): only meaningful for the design that
%     was actually used on the AERO (row iUsed of the designs list)
% Metrics can only be compared between runs that use the SAME references.
%
% Needs in the same folder:
%   quanser_aero_parameters.m, quanser_aero_state_space.m (without "clear"),
%   computeMetrics.m, modelError.m (function files shared with the other controllers)
%
% Limits of the simulation: the model is linear (no V^2 thrust, no friction)
% and it uses the exact velocities, while the real loop differentiates the
% encoders. Always confirm on the AERO with small references first.

clear;
close all;
clc

%% 0) Settings
%dataFile = '../reponse_BF_experimentation/aero_lqr_rsp_fixed2';
dataFile  = 'aero_lqr_rsp_fixed3_sys1';        % measured test: gives the references (9 rows)
                                          % must end with sysN (N = row of the design used)
resFile   = ['results_tune_lqr_' dataFile '.mat'];   % one results file per test, named after it
satV      = 20;                           % voltage saturation (V)
epsSettle = 0.05;                         % settling band: +/- 5 % of the step size
d2r       = pi/180;

% Designs to compare, one per row: {name, Q, R}
% State order: x = [theta, psi, theta_dot, psi_dot]  (rad, rad/s)
% Bryson's rule: Q(i,i) = 1/(largest acceptable x_i)^2, R(j,j) = 1/(largest acceptable u_j)^2
%   Bryson values used: errors 5 deg (pitch) and 10 deg (yaw), velocities 60 or
%   40 deg/s, voltage 15 V. Only the ratios between the values change K.
%   The velocity terms are ~100 times smaller than the angle terms, so going
%   from 60 to 40 deg/s only adds a little damping.
designs = {
    'Current (test)',    diag([200 75 0 0]),   0.005*eye(2)
    'Softer (R x4)',     diag([200 75 0 0]),   0.02*eye(2)
    'More damping',      diag([200 75 20 10]), 0.005*eye(2)
    'Bryson (60 deg/s)', diag([1/(5*d2r)^2, 1/(10*d2r)^2, 1/(60*d2r)^2, 1/(60*d2r)^2]), eye(2)/15^2
    'Bryson (40 deg/s)', diag([1/(5*d2r)^2, 1/(10*d2r)^2, 1/(40*d2r)^2, 1/(40*d2r)^2]), 0.005*eye(2)
    };
% Row of the design used for the measured test, read from the end of the file
% name: ..._sys1 -> row 1 (Current), ..._sys4 -> row 4 (Bryson 60 deg/s), etc.
% (06/10/2026: before, set by hand, easy to forget when changing dataFile)
tok = regexp(dataFile, 'sys(\d+)$', 'tokens', 'once');
if isempty(tok)
    error('No "sysN" at the end of dataFile: set iUsed by hand.')
end
iUsed = str2double(tok{1});
nD     = size(designs,1);
colors = lines(nD);

%% 1) Model (Quanser grey-box)
quanser_aero_state_space;                 % runs quanser_aero_parameters, defines A, B, C = eye(2,4), D

%% 2) Measured test (references, measured angles and voltages)
S    = load(dataFile);
fn   = fieldnames(S);
data = S.(fn{1});
if size(data,1) > size(data,2)
    data = data';
end
t          = data(1,:)';
PitchVolt  = data(2,:)';
PitchRef   = data(3,:)';            % deg
PitchAngle = data(4,:)';            % deg
YawVolt    = data(5,:)';
YawRef     = data(6,:)';            % deg
YawAngle   = data(7,:)';            % deg
Ts         = t(2) - t(1);

% Discrete model with zero-order hold, like the real digital loop at Ts = 2 ms
% (only sysd.A and sysd.B are used by the simulation)
sysd = c2d(ss(A, B, C, D), Ts);

%% 3) Simulate every design and compute its metrics
% Every run (simulated or measured) is stored with the SAME fields, so that
% runs from other scripts (LQI, PID...) can later be put in the same list:
%   name, source ('simulated' or 'measured'), ctrl (controller settings),
%   t, ref [pitch yaw] (deg), y [pitch yaw] (deg), u [Vp Vy] (V), m (metrics)
results = struct('name', {}, 'source', {}, 'ctrl', {}, 't', {}, ...
                 'ref', {}, 'y', {}, 'u', {}, 'm', {});

for d = 1:nD
    K = lqr(A, B, designs{d,2}, designs{d,3});
    [Y, U] = simulateLQR(sysd.A, sysd.B, K, t, PitchRef*d2r, YawRef*d2r, satV);

    results(d).name   = designs{d,1};
    results(d).source = 'simulated';
    results(d).ctrl   = struct('type', 'LQR', 'Q', designs{d,2}, 'R', designs{d,3}, ...
                               'K', K, 'poles', eig(A - B*K));
    results(d).t      = t;
    results(d).ref    = [PitchRef, YawRef];
    results(d).y      = Y;
    results(d).u      = U;
    results(d).m      = computeMetrics(t, [PitchRef, YawRef], Y, U, satV, epsSettle);
end

%% 4) Measured test: same metrics, then model error
iM = nD + 1;
results(iM).name   = 'MEASURED';
results(iM).source = 'measured';
results(iM).ctrl   = results(iUsed).ctrl;   % the measured test was run with design iUsed
results(iM).t      = t;
results(iM).ref    = [PitchRef, YawRef];
results(iM).y      = [PitchAngle, YawAngle];
results(iM).u      = [PitchVolt, YawVolt];
results(iM).m      = computeMetrics(t, [PitchRef, YawRef], [PitchAngle, YawAngle], ...
                                    [PitchVolt, YawVolt], satV, epsSettle);
nR = numel(results);

% Model error: simulation of the design used on the AERO vs measurement
modelErr = modelError(results(iUsed).y, results(iUsed).u, results(iM).y, results(iM).u);

% Same metrics as a table (each variable has 2 columns: [pitch, yaw];
% for the coupling: [pitch moved by the yaw steps, yaw moved by the pitch steps])
M     = [results.m];
names = {results.name}';
metricsTable = table(vertcat(M.rmse), vertcat(M.tr), vertcat(M.os), vertcat(M.ts), ...
                     vertcat(M.ess), vertcat(M.rmsU), vertcat(M.sat), vertcat(M.tv), vertcat(M.cpl), ...
                     'VariableNames', {'RMSE_deg', 'tr_s', 'OS_pct', 'ts_s', 'ess_deg', ...
                                       'RMSu_V', 'sat_pct', 'TVu_Vps', 'coupling_deg'}, ...
                     'RowNames', names);

%% 5) Tables in the Command Window
fprintf('--- Designs: slowest closed-loop pole and largest velocity gain ---\n');
for d = 1:nD
    p = results(d).ctrl.poles;
    [~, iSlow] = max(real(p));              % pole closest to the imaginary axis = slowest
    kVel = max(max(abs(results(d).ctrl.K(:,3:4))));   % large velocity gains amplify derivative noise
    fprintf('%-15s | slowest pole %+6.2f %+6.2fj rad/s | max K_vel %5.1f V.s/rad\n', ...
            results(d).name, real(p(iSlow)), abs(imag(p(iSlow))), kVel);
end

fprintf('\n--- Tracking (step metrics: median over the steps; RMSE: whole test) ---\n');
fprintf('%-15s | %6s %5s %5s %5s %5s | %6s %5s %5s %5s %5s\n', 'Run', ...
        'RMSE_p', 'tr_p', 'OS_p', 'ts_p', '|e_p|', 'RMSE_y', 'tr_y', 'OS_y', 'ts_y', '|e_y|');
fprintf('%-15s | %6s %5s %5s %5s %5s | %6s %5s %5s %5s %5s\n', '', ...
        '(deg)', '(s)', '(%)', '(s)', '(deg)', '(deg)', '(s)', '(%)', '(s)', '(deg)');
for r = 1:nR
    m = results(r).m;
    fprintf('%-15s | %6.2f %5.2f %5.1f %5.2f %5.2f | %6.2f %5.2f %5.1f %5.2f %5.2f\n', results(r).name, ...
            m.rmse(1), m.tr(1), m.os(1), m.ts(1), m.ess(1), m.rmse(2), m.tr(2), m.os(2), m.ts(2), m.ess(2));
end

fprintf('\n--- Voltages and coupling ---\n');
fprintf('%-15s | %6s %6s | %5s %5s | %6s %6s | %10s %10s\n', 'Run', ...
        'RMS_Vp', 'RMS_Vy', 'sat_p', 'sat_y', 'TV_p', 'TV_y', 'pitch<-yaw', 'yaw<-pitch');
fprintf('%-15s | %6s %6s | %5s %5s | %6s %6s | %10s %10s\n', '', ...
        '(V)', '(V)', '(%)', '(%)', '(V/s)', '(V/s)', '(deg)', '(deg)');
for r = 1:nR
    m = results(r).m;
    fprintf('%-15s | %6.2f %6.2f | %5.1f %5.1f | %6.1f %6.1f | %10.2f %10.2f\n', results(r).name, ...
            m.rmsU(1), m.rmsU(2), m.sat(1), m.sat(2), m.tv(1), m.tv(2), m.cpl(1), m.cpl(2));
end

fprintf('\n--- Model error: "%s" simulated vs measured ---\n', results(iUsed).name);
fprintf('Fit                 : pitch %5.1f %%,  yaw %5.1f %%\n', modelErr.fit);
fprintf('RMS of angle error  : pitch %5.2f deg, yaw %5.2f deg\n', modelErr.rmseY);
fprintf('RMS of voltage error: Vp    %5.2f V,   Vy  %5.2f V\n\n', modelErr.rmseU);

% Gain matrices
for d = 1:nD
    fprintf('K (%s) =\n', results(d).name);
    disp(results(d).ctrl.K)
end

%% 6) Save the results
% To reload later:  R = load('results_tune_lqr_aero_lqr_rsp_fixed2.mat');  R.metricsTable
save(resFile, 'results', 'metricsTable', 'modelErr', 'dataFile', 'satV', 'epsSettle');
fprintf('Results saved to %s\n', resFile);

%% 7) Figure 1: simulated angles vs measured
figure('Name', 'LQR tuning: angles', 'Color', 'w');
ax1 = subplot(2,1,1);  hold on;  grid on
ax2 = subplot(2,1,2);  hold on;  grid on
plot(ax1, t, PitchRef,   'k--', 'LineWidth', 1)
plot(ax1, t, PitchAngle, 'k',   'LineWidth', 2)
plot(ax2, t, YawRef,     'k--', 'LineWidth', 1)
plot(ax2, t, YawAngle,   'k',   'LineWidth', 2)
for d = 1:nD
    plot(ax1, t, results(d).y(:,1), 'Color', colors(d,:), 'LineWidth', 1.2)
    plot(ax2, t, results(d).y(:,2), 'Color', colors(d,:), 'LineWidth', 1.2)
end
title(ax1, 'Pitch: measured (black) vs simulated designs')
ylabel(ax1, '\theta (deg)')
title(ax2, 'Yaw: measured (black) vs simulated designs')
ylabel(ax2, '\psi (deg)')
xlabel(ax2, 'Time (s)')
legend(ax1, [{'Reference', 'Measured'}, designs(:,1)'], 'Location', 'eastoutside')
legend(ax2, [{'Reference', 'Measured'}, designs(:,1)'], 'Location', 'eastoutside')
linkaxes([ax1 ax2], 'x')
xlim(ax1, [t(1) t(end)])

%% 8) Figure 2: simulated voltages vs measured
figure('Name', 'LQR tuning: voltages', 'Color', 'w');
bx1 = subplot(2,1,1);  hold on;  grid on
bx2 = subplot(2,1,2);  hold on;  grid on
plot(bx1, t, PitchVolt, 'Color', [0.6 0.6 0.6])
plot(bx2, t, YawVolt,   'Color', [0.6 0.6 0.6])
for d = 1:nD
    plot(bx1, t, results(d).u(:,1), 'Color', colors(d,:), 'LineWidth', 1.2)
    plot(bx2, t, results(d).u(:,2), 'Color', colors(d,:), 'LineWidth', 1.2)
end
title(bx1, 'Pitch motor voltage: measured (grey) vs simulated')
ylabel(bx1, 'V_p (V)')
title(bx2, 'Yaw motor voltage: measured (grey) vs simulated')
ylabel(bx2, 'V_y (V)')
xlabel(bx2, 'Time (s)')
legend(bx1, [{'Measured'}, designs(:,1)'], 'Location', 'eastoutside')
legend(bx2, [{'Measured'}, designs(:,1)'], 'Location', 'eastoutside')
ylim(bx1, [-satV-2, satV+2])
ylim(bx2, [-satV-2, satV+2])
linkaxes([bx1 bx2], 'x')
xlim(bx1, [t(1) t(end)])

%% 9) Figure 3: metrics of every run, one panel per metric
% Two bars per run: pitch (left) and yaw (right). Lower is better everywhere.
metricList = {
    'rmse', 'Tracking RMSE (deg)'
    'tr',   'Rise time 10-90 % (s)'
    'os',   'Overshoot (%)'
    'ts',   'Settling time, 5 % band (s)'
    'ess',  'Steady-state error (deg)'
    'rmsU', 'RMS voltage (V)'
    'sat',  'Time saturated (%)'
    'tv',   'Voltage activity sum|dV|/T (V/s)'
    'cpl',  'Moved by the other axis step (deg)'
    };
figure('Name', 'Metrics', 'Color', 'w');
for i = 1:size(metricList,1)
    cx = subplot(3, 3, i);  hold on;  grid on
    vals = vertcat(M.(metricList{i,1}));      % nR x 2: [pitch, yaw]
    bar(cx, vals)
    set(cx, 'XTick', 1:nR, 'XTickLabel', names)
    xtickangle(cx, 30)
    title(cx, metricList{i,2})
end
legend(cx, {'Pitch', 'Yaw'}, 'Location', 'best')

%% 10) Figure 4: model error = measured - simulated, for the design used on the AERO
% Where these curves leave zero, the model is wrong. Dotted lines = reference steps.
errTheta = PitchAngle - results(iUsed).y(:,1);
errPsi   = YawAngle   - results(iUsed).y(:,2);
errVp    = PitchVolt  - results(iUsed).u(:,1);
errVy    = YawVolt    - results(iUsed).u(:,2);
% Start of each move of the references (a ramp counts once, not once per sample:
% same rule as moveStarts in computeMetrics.m)
chgP = [false; abs(diff(PitchRef)) > 1e-6];          % pitch reference changing
chgY = [false; abs(diff(YawRef))   > 1e-6];          % yaw reference changing
stepTimes = t([find(chgP & ~[false; chgP(1:end-1)]); find(chgY & ~[false; chgY(1:end-1)])]);

figure('Name', 'Model error', 'Color', 'w');
ex1 = subplot(4,1,1);  hold on;  grid on
plot(ex1, t, errTheta, 'k')
ylabel(ex1, '\Delta\theta (deg)')
title(ex1, sprintf('Measured - simulated ("%s"): fit pitch %.1f %%, yaw %.1f %%', ...
                   results(iUsed).name, modelErr.fit(1), modelErr.fit(2)))
ex2 = subplot(4,1,2);  hold on;  grid on
plot(ex2, t, errPsi, 'k')
ylabel(ex2, '\Delta\psi (deg)')
ex3 = subplot(4,1,3);  hold on;  grid on
plot(ex3, t, errVp, 'Color', [0.4 0.4 0.4])
ylabel(ex3, '\DeltaV_p (V)')
ex4 = subplot(4,1,4);  hold on;  grid on
plot(ex4, t, errVy, 'Color', [0.4 0.4 0.4])
ylabel(ex4, '\DeltaV_y (V)')
xlabel(ex4, 'Time (s)')
for ex = [ex1 ex2 ex3 ex4]
    for k = 1:length(stepTimes)
        plot(ex, [stepTimes(k) stepTimes(k)], ylim(ex), ':', 'Color', [0.5 0.5 0.5])
    end
end
linkaxes([ex1 ex2 ex3 ex4], 'x')
xlim(ex1, [t(1) t(end)])

set(findall(0, 'Type', 'axes'), 'FontSize', 12)

%% ======================= LOCAL FUNCTIONS =======================
% (in a script, the functions must be at the end of the file)
% simulateLQR stays here: it is specific to the LQR. The metrics are in
% computeMetrics.m and modelError.m, shared with the other controllers.

function [Y, U] = simulateLQR(Ad, Bd, K, t, refP, refY, satV)
% Discrete simulation of the closed loop:
%   u(k)   = sat( K*(xd(k) - x(k)) ),   xd = [refP; refY; 0; 0]  (rad)
%   x(k+1) = Ad*x(k) + Bd*u(k)
% Returns the angles in degrees and the voltages in volts.
    N = length(t);
    x = zeros(4,1);
    Y = zeros(N,2);
    U = zeros(N,2);
    for k = 1:N
        xd = [refP(k); refY(k); 0; 0];
        u  = K*(xd - x);
        u  = min(max(u, -satV), satV);          % +/- satV saturation
        Y(k,:) = x(1:2)' * 180/pi;
        U(k,:) = u';
        x = Ad*x + Bd*u;
    end
end