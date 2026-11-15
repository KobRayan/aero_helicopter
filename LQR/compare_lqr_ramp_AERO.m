% compare_lqr_ramp_AERO.m
% 06/10/2026
% Compares the 4 LQR designs tested on the AERO with RAMP references
% (Rate Limiter 20 deg/s, files aero_lqr_rsp_fixed3_sys1..4) and, if their
% results files exist, with the same 4 designs tested with STEP references
% (aero_lqr_rsp_fixed2_sys1..4).
%
% Uses the results files saved by tune_lqr_AERO.m (results_tune_lqr_<test>.mat).
% Each one already holds the measured signals of its test (copied from the
% 9-row log: time, references, angles, voltages) and the simulation of every
% design with the same references, so the raw logs do not need to be reloaded.
%
% Test sysN was run with design N (row N of the designs list in tune_lqr_AERO.m):
%   sys1 Current, sys2 Softer (R x4), sys3 More damping, sys4 Bryson (60 deg/s)
%
% All metrics are RECOMPUTED here with the current computeMetrics.m and
% modelError.m, so every run (ramp or step, measured or simulated) uses exactly
% the same definitions, whatever version of the code produced the .mat files.
%
% Needs in the same folder: computeMetrics.m, modelError.m and the results files.
% Needs MATLAB R2019b or later (XEndPoints of the bar charts).
%
% Reminders:
%   - with ramps, rise and settling times include the ramp duration: compare
%     overshoot, steady-state error, saturation, voltage activity and coupling
%   - each design ran ONCE: small differences may be run-to-run spread

clear;
close all;
clc

%% 0) Settings
rampBase   = 'aero_lqr_rsp_fixed3_sys';   % ramp tests: <rampBase>1 ... <rampBase>4
stepBase   = 'aero_lqr_rsp_fixed2_sys';   % step tests (optional, skipped if missing)
sysList    = 1:4;                         % test sysN was run with design N
satV       = 20;                          % voltage saturation (V)
epsSettle  = 0.05;                        % settling band: +/- 5 % of the move size
tPitchZoom = 5;                           % start of the pitch move to zoom on (s)
tYawZoom   = 12.5;                        % start of the yaw move to zoom on (s)
zoomWin    = [-1 6];                      % zoom window around the move start (s)
resFile    = 'compare_lqr_ramp_AERO_results.mat';

nT     = numel(sysList);
colors = lines(nT);

%% 1) Load the ramp tests, then the step tests if they exist
% Each run is first stored in a cell, then the cells are joined into one
% struct array (rampRuns(i).name, rampRuns(i).m, ...). Assigning a struct
% directly into an empty array [] is not allowed by MATLAB.
runsC = cell(1, nT);
for i = 1:nT
    N = sysList(i);
    f = ['results_tune_lqr_' rampBase num2str(N) '.mat'];
    runsC{i} = loadRun(f, N, satV, epsSettle);
end
rampRuns = [runsC{:}];                    % 1 x nT struct array
names    = {rampRuns.name};

haveSteps = true;
runsC     = cell(1, nT);
for i = 1:nT
    N = sysList(i);
    f = ['results_tune_lqr_' stepBase num2str(N) '.mat'];
    if ~isfile(f) || ~ismember('results', who('-file', f))
        haveSteps = false;
        fprintf('Step results not found (%s): step vs ramp comparison skipped.\n\n', f);
        break
    end
    runsC{i} = loadRun(f, N, satV, epsSettle);
end
if haveSteps
    stepRuns = [runsC{:}];
else
    stepRuns = [];                        % nothing to compare with
end

%% 2) Check that the tests used the same references
% Metrics can only be compared between runs that use the SAME references.
fprintf('--- Reference check (compared with the first ramp test) ---\n');
for i = 1:nT
    n  = min(numel(rampRuns(1).t), numel(rampRuns(i).t));
    dR = max(abs(rampRuns(i).ref(1:n,:) - rampRuns(1).ref(1:n,:)));
    fprintf('%-18s | %6d samples, %5.1f s | max |ref difference|: pitch %5.2f deg, yaw %5.2f deg\n', ...
            rampRuns(i).name, numel(rampRuns(i).t), rampRuns(i).t(end), dR(1), dR(2));
end
if haveSteps
    % Step and ramp references differ in shape, but must have the same amplitudes
    fprintf('Amplitudes (max |ref|): ramp tests pitch %.1f / yaw %.1f deg, step tests pitch %.1f / yaw %.1f deg\n', ...
            max(abs(rampRuns(1).ref)), max(abs(stepRuns(1).ref)));
end

%% 3) Tables in the Command Window: measured performance (ramp tests)
fprintf('\n--- Ramp tests, MEASURED tracking (median over the moves; RMSE: whole test) ---\n');
fprintf('(rise and settling times are left out: with ramps they mostly measure the ramp)\n');
fprintf('%-18s | %6s %5s %5s | %6s %5s %5s\n', 'Design', ...
        'RMSE_p', 'OS_p', '|e_p|', 'RMSE_y', 'OS_y', '|e_y|');
fprintf('%-18s | %6s %5s %5s | %6s %5s %5s\n', '', ...
        '(deg)', '(%)', '(deg)', '(deg)', '(%)', '(deg)');
for i = 1:nT
    m = rampRuns(i).m;
    fprintf('%-18s | %6.2f %5.1f %5.2f | %6.2f %5.1f %5.2f\n', rampRuns(i).name, ...
            m.rmse(1), m.os(1), m.ess(1), m.rmse(2), m.os(2), m.ess(2));
end

fprintf('\n--- Ramp tests, MEASURED voltages and coupling ---\n');
fprintf('%-18s | %6s %6s | %5s %5s | %6s %6s | %10s %10s\n', 'Design', ...
        'RMS_Vp', 'RMS_Vy', 'sat_p', 'sat_y', 'TV_p', 'TV_y', 'pitch<-yaw', 'yaw<-pitch');
fprintf('%-18s | %6s %6s | %5s %5s | %6s %6s | %10s %10s\n', '', ...
        '(V)', '(V)', '(%)', '(%)', '(V/s)', '(V/s)', '(deg)', '(deg)');
for i = 1:nT
    m = rampRuns(i).m;
    fprintf('%-18s | %6.2f %6.2f | %5.1f %5.1f | %6.1f %6.1f | %10.2f %10.2f\n', rampRuns(i).name, ...
            m.rmsU(1), m.rmsU(2), m.sat(1), m.sat(2), m.tv(1), m.tv(2), m.cpl(1), m.cpl(2));
end

%% 4) Measured vs simulated (how far reality is from the model, per design)
% Ratio = measured / simulated: 1 = the model predicts this metric well.
fprintf('\n--- Ramp tests, MEASURED / SIMULATED ratio, and model fit ---\n');
fprintf('%-18s | %5s %5s | %10s %10s | %6s %6s | %5s %5s | %6s %6s\n', 'Design', ...
        'e_p', 'e_y', 'pitch<-yaw', 'yaw<-pitch', 'RMS_Vp', 'RMS_Vy', 'TV_p', 'TV_y', 'fit_p', 'fit_y');
fprintf('%-18s | %5s %5s | %10s %10s | %6s %6s | %5s %5s | %6s %6s\n', '', ...
        '(x)', '(x)', '(x)', '(x)', '(x)', '(x)', '(x)', '(x)', '(%)', '(%)');
for i = 1:nT
    m  = rampRuns(i).m;
    ms = rampRuns(i).mSim;
    e  = rampRuns(i).err;
    fprintf('%-18s | %5.1f %5.1f | %10.1f %10.1f | %6.1f %6.1f | %5.1f %5.1f | %6.1f %6.1f\n', ...
            rampRuns(i).name, m.ess ./ ms.ess, m.cpl ./ ms.cpl, m.rmsU ./ ms.rmsU, ...
            m.tv ./ ms.tv, e.fit(1), e.fit(2));
end

%% 5) Best and worst measured design for each metric (lower is better)
bestList = {
    'rmse', 1, 'Pitch RMSE (deg)'
    'rmse', 2, 'Yaw RMSE (deg)'
    'ess',  1, 'Pitch steady-state error (deg)'
    'ess',  2, 'Yaw steady-state error (deg)'
    'os',   2, 'Yaw overshoot (%)'
    'cpl',  1, 'Pitch moved by yaw (deg)'
    'cpl',  2, 'Yaw moved by pitch (deg)'
    'tv',   1, 'Pitch voltage activity (V/s)'
    'tv',   2, 'Yaw voltage activity (V/s)'
    };
fprintf('\n--- Ramp tests: best and worst MEASURED design per metric (one run each) ---\n');
for b = 1:size(bestList,1)
    vals = zeros(nT,1);
    for i = 1:nT
        vals(i) = rampRuns(i).m.(bestList{b,1})(bestList{b,2});
    end
    [vBest, iBest]   = min(vals);
    [vWorst, iWorst] = max(vals);
    fprintf('%-31s: best %-18s (%6.2f) | worst %-18s (%6.2f)\n', bestList{b,3}, ...
            names{iBest}, vBest, names{iWorst}, vWorst);
end

%% 6) Step vs ramp, measured, design by design
if haveSteps
    fprintf('\n--- MEASURED, step -> ramp: tracking and coupling ---\n');
    fprintf('%-18s | %14s | %14s | %14s | %14s | %14s\n', 'Design', ...
            'pitch<-yaw', 'yaw<-pitch', 'OS_y (%)', '|e_p| (deg)', '|e_y| (deg)');
    for i = 1:nT
        s = stepRuns(i).m;
        r = rampRuns(i).m;
        fprintf('%-18s | %5.2f -> %5.2f | %5.2f -> %5.2f | %5.1f -> %5.1f | %5.2f -> %5.2f | %5.2f -> %5.2f\n', ...
                rampRuns(i).name, s.cpl(1), r.cpl(1), s.cpl(2), r.cpl(2), s.os(2), r.os(2), ...
                s.ess(1), r.ess(1), s.ess(2), r.ess(2));
    end

    fprintf('\n--- MEASURED, step -> ramp: voltages and model fit ---\n');
    fprintf('%-18s | %14s | %14s | %14s | %14s | %14s | %14s\n', 'Design', ...
            'sat_p (%)', 'sat_y (%)', 'TV_p (V/s)', 'TV_y (V/s)', 'fit_p (%)', 'fit_y (%)');
    for i = 1:nT
        s  = stepRuns(i).m;
        r  = rampRuns(i).m;
        es = stepRuns(i).err;
        er = rampRuns(i).err;
        fprintf('%-18s | %5.1f -> %5.1f | %5.1f -> %5.1f | %5.1f -> %5.1f | %5.1f -> %5.1f | %5.1f -> %5.1f | %5.1f -> %5.1f\n', ...
                rampRuns(i).name, s.sat(1), r.sat(1), s.sat(2), r.sat(2), s.tv(1), r.tv(1), ...
                s.tv(2), r.tv(2), es.fit(1), er.fit(1), es.fit(2), er.fit(2));
    end
end

%% 7) Save the comparison
% To reload later:  C = load('compare_lqr_ramp_AERO_results.mat');  C.rampRuns(1).m
save(resFile, 'rampRuns', 'stepRuns', 'haveSteps', 'sysList', 'satV', 'epsSettle');
fprintf('\nResults saved to %s\n', resFile);

%% 8) Figure 1: measured angles of the 4 designs
figure('Name', 'Ramp tests: angles', 'Color', 'w');
ax1 = subplot(2,1,1);  hold on;  grid on
ax2 = subplot(2,1,2);  hold on;  grid on
plot(ax1, rampRuns(1).t, rampRuns(1).ref(:,1), 'k--', 'LineWidth', 1)
plot(ax2, rampRuns(1).t, rampRuns(1).ref(:,2), 'k--', 'LineWidth', 1)
for i = 1:nT
    plot(ax1, rampRuns(i).t, rampRuns(i).y(:,1), 'Color', colors(i,:), 'LineWidth', 1.2)
    plot(ax2, rampRuns(i).t, rampRuns(i).y(:,2), 'Color', colors(i,:), 'LineWidth', 1.2)
end
title(ax1, 'Pitch, measured (ramp references)')
ylabel(ax1, '\theta (deg)')
title(ax2, 'Yaw, measured (ramp references)')
ylabel(ax2, '\psi (deg)')
xlabel(ax2, 'Time (s)')
legend(ax1, [{'Reference'}, names], 'Location', 'eastoutside')
legend(ax2, [{'Reference'}, names], 'Location', 'eastoutside')
linkaxes([ax1 ax2], 'x')
xlim(ax1, [rampRuns(1).t(1) rampRuns(1).t(end)])

%% 9) Figure 2: measured voltages of the 4 designs
figure('Name', 'Ramp tests: voltages', 'Color', 'w');
bx1 = subplot(2,1,1);  hold on;  grid on
bx2 = subplot(2,1,2);  hold on;  grid on
for i = 1:nT
    plot(bx1, rampRuns(i).t, rampRuns(i).u(:,1), 'Color', colors(i,:))
    plot(bx2, rampRuns(i).t, rampRuns(i).u(:,2), 'Color', colors(i,:))
end
for bx = [bx1 bx2]
    tLim = [rampRuns(1).t(1) rampRuns(1).t(end)];
    plot(bx, tLim, [ satV  satV], 'k--')       % saturation limits
    plot(bx, tLim, [-satV -satV], 'k--')
    ylim(bx, [-satV-2, satV+2])
end
title(bx1, 'Pitch motor voltage, measured (dashed: \pm20 V saturation)')
ylabel(bx1, 'V_p (V)')
title(bx2, 'Yaw motor voltage, measured')
ylabel(bx2, 'V_y (V)')
xlabel(bx2, 'Time (s)')
legend(bx1, names, 'Location', 'eastoutside')
legend(bx2, names, 'Location', 'eastoutside')
linkaxes([bx1 bx2], 'x')
xlim(bx1, [rampRuns(1).t(1) rampRuns(1).t(end)])

%% 10) Figure 3: zoom on one move of each axis, and its effect on the other axis
% Left: a pitch move (pitch angle, then how much the yaw moved).
% Right: a yaw move (yaw angle, then how much the pitch moved).
% The deviation is measured from the value just before the move (mean over 0.2 s).
figure('Name', 'Ramp tests: zoom on one move and coupling', 'Color', 'w');
zx1 = subplot(2,2,1);  hold on;  grid on
zx3 = subplot(2,2,3);  hold on;  grid on
zx2 = subplot(2,2,2);  hold on;  grid on
zx4 = subplot(2,2,4);  hold on;  grid on
for i = 1:nT
    tt = rampRuns(i).t;
    y  = rampRuns(i).y;
    Ts = tt(2) - tt(1);
    nPre = round(0.2/Ts);

    % Pitch move: pitch angle and yaw deviation
    [~, kP] = min(abs(tt - tPitchZoom));
    idx  = (tt >= tPitchZoom + zoomWin(1)) & (tt <= tPitchZoom + zoomWin(2));
    yaw0 = mean(y(kP-nPre:kP-1, 2));
    if i == 1
        plot(zx1, tt(idx) - tPitchZoom, rampRuns(1).ref(idx,1), 'k--', 'LineWidth', 1)
    end
    plot(zx1, tt(idx) - tPitchZoom, y(idx,1),        'Color', colors(i,:), 'LineWidth', 1.2)
    plot(zx3, tt(idx) - tPitchZoom, y(idx,2) - yaw0, 'Color', colors(i,:), 'LineWidth', 1.2)

    % Yaw move: yaw angle and pitch deviation
    [~, kY]  = min(abs(tt - tYawZoom));
    idx      = (tt >= tYawZoom + zoomWin(1)) & (tt <= tYawZoom + zoomWin(2));
    pitch0   = mean(y(kY-nPre:kY-1, 1));
    if i == 1
        plot(zx2, tt(idx) - tYawZoom, rampRuns(1).ref(idx,2), 'k--', 'LineWidth', 1)
    end
    plot(zx2, tt(idx) - tYawZoom, y(idx,2),          'Color', colors(i,:), 'LineWidth', 1.2)
    plot(zx4, tt(idx) - tYawZoom, y(idx,1) - pitch0, 'Color', colors(i,:), 'LineWidth', 1.2)
end
title(zx1, sprintf('Pitch move at t = %.1f s', tPitchZoom))
ylabel(zx1, '\theta (deg)')
title(zx3, 'Yaw moved by this pitch move')
ylabel(zx3, '\Delta\psi (deg)')
xlabel(zx3, 'Time from the start of the move (s)')
title(zx2, sprintf('Yaw move at t = %.1f s', tYawZoom))
ylabel(zx2, '\psi (deg)')
title(zx4, 'Pitch moved by this yaw move')
ylabel(zx4, '\Delta\theta (deg)')
xlabel(zx4, 'Time from the start of the move (s)')
legend(zx2, [{'Reference'}, names], 'Location', 'best')
linkaxes([zx1 zx3], 'x')
linkaxes([zx2 zx4], 'x')
xlim(zx1, zoomWin)
xlim(zx2, zoomWin)

%% 11) Figure 4: metrics of the 4 designs, measured (bars) and simulated (crosses)
% Two bars per design: pitch (left) and yaw (right). Lower is better everywhere.
metricList = {
    'rmse', 'Tracking RMSE (deg)'
    'os',   'Overshoot (%)'
    'ess',  'Steady-state error (deg)'
    'rmsU', 'RMS voltage (V)'
    'sat',  'Time saturated (%)'
    'tv',   'Voltage activity sum|dV|/T (V/s)'
    'cpl',  'Moved by the other axis (deg)'
    'tr',   'Rise time 10-90 % (s), mostly the ramp'
    'ts',   'Settling time (s), includes the ramp'
    };
Mm = [rampRuns.m];
Ms = [rampRuns.mSim];
figure('Name', 'Ramp tests: metrics', 'Color', 'w');
for k = 1:size(metricList,1)
    cx = subplot(3, 3, k);  hold on;  grid on
    valsM = vertcat(Mm.(metricList{k,1}));    % nT x 2: [pitch, yaw], measured
    valsS = vertcat(Ms.(metricList{k,1}));    % nT x 2: [pitch, yaw], simulated
    hb = bar(cx, valsM);
    for a = 1:2
        plot(cx, hb(a).XEndPoints, valsS(:,a), 'kx', 'MarkerSize', 9, 'LineWidth', 1.5)
    end
    set(cx, 'XTick', 1:nT, 'XTickLabel', names)
    xtickangle(cx, 30)
    title(cx, metricList{k,2})
end
legend(cx, {'Pitch (measured)', 'Yaw (measured)', 'Simulated'}, 'Location', 'best')

%% 12) Figure 5: model error per design (simulation of the design vs its test)
E = [rampRuns.err];
figure('Name', 'Ramp tests: model error', 'Color', 'w');
fx1 = subplot(1,3,1);  hold on;  grid on
bar(fx1, vertcat(E.fit))
title(fx1, 'Model fit (%), 100 = perfect')
fx2 = subplot(1,3,2);  hold on;  grid on
bar(fx2, vertcat(E.rmseY))
title(fx2, 'RMS angle error, measured - simulated (deg)')
fx3 = subplot(1,3,3);  hold on;  grid on
bar(fx3, vertcat(E.rmseU))
title(fx3, 'RMS voltage error, measured - simulated (V)')
for fx = [fx1 fx2 fx3]
    set(fx, 'XTick', 1:nT, 'XTickLabel', names)
    xtickangle(fx, 30)
end
legend(fx1, {'Pitch', 'Yaw'}, 'Location', 'southwest')

%% 13) Figure 6: step vs ramp, measured, design by design
if haveSteps
    cmpList = {
        'cpl',  1, 'Pitch moved by yaw (deg)'
        'cpl',  2, 'Yaw moved by pitch (deg)'
        'os',   2, 'Yaw overshoot (%)'
        'ess',  1, 'Pitch steady-state error (deg)'
        'sat',  1, 'Pitch motor saturated (%)'
        'tv',   1, 'Pitch voltage activity (V/s)'
        };
    Mst = [stepRuns.m];
    figure('Name', 'Step vs ramp (measured)', 'Color', 'w');
    for k = 1:size(cmpList,1)
        cx = subplot(2, 3, k);  hold on;  grid on
        a    = cmpList{k,2};
        vSt  = vertcat(Mst.(cmpList{k,1}));    % nT x 2, step tests
        vRa  = vertcat(Mm.(cmpList{k,1}));     % nT x 2, ramp tests
        hb   = bar(cx, [vSt(:,a), vRa(:,a)]);
        hb(1).FaceColor = [0.6 0.6 0.6];       % grey = step
        hb(2).FaceColor = [0.0 0.45 0.74];     % blue = ramp
        set(cx, 'XTick', 1:nT, 'XTickLabel', names)
        xtickangle(cx, 30)
        title(cx, cmpList{k,3})
    end
    legend(cx, {'Step', 'Ramp'}, 'Location', 'best')
end

set(findall(0, 'Type', 'axes'), 'FontSize', 12)

%% ======================= LOCAL FUNCTIONS =======================
% (in a script, the functions must be at the end of the file)

function run = loadRun(fileName, N, satV, epsSettle)
% Loads one results file saved by tune_lqr_AERO.m and returns:
%   name, K        : name and gain matrix of design N (the one used on the AERO)
%   t, ref, y, u   : the measured test (N x 1, then N x 2 [pitch, yaw])
%   ySim, uSim     : the simulation of design N with the same references
%   m, mSim        : metrics of the measured and simulated runs (computeMetrics.m)
%   err            : model error, simulated vs measured (modelError.m)
    S    = load(fileName, 'results');
    res  = S.results;
    meas = res(strcmp({res.source}, 'measured'));   % the measured test
    sim  = res(N);                                  % simulation of design N

    run.name = sim.name;
    run.K    = sim.ctrl.K;
    run.t    = meas.t;
    run.ref  = meas.ref;
    run.y    = meas.y;
    run.u    = meas.u;
    run.ySim = sim.y;
    run.uSim = sim.u;
    run.m    = computeMetrics(meas.t, meas.ref, meas.y, meas.u, satV, epsSettle);
    run.mSim = computeMetrics(sim.t, sim.ref, sim.y, sim.u, satV, epsSettle);
    run.err  = modelError(sim.y, sim.u, meas.y, meas.u);
end