%% tune_lqr_model_refined.m
% =========================================================================
%  LQR: OLD QUANSER MODEL vs NEW IDENTIFIED MODEL, ON THE 4 DESIGNS
% =========================================================================
%
% GOAL
% ----
% We have two versions of the SAME 4-state model (Quanser structure,
% x = [theta, psi, theta_dot, psi_dot], u = [Vp, Vy]):
%   - "Quanser"    : default parameters of quanser_aero_parameters.m
%   - "identified" : parameters identified with CONTSID on our AERO
%                    (quanser_aero_parameters_identified_2.mat, made by
%                     identify_quanser_AERO.m)
% This script answers two questions, for each of the 4 LQR designs that
% were tested on the real AERO (sys1 = Current, sys2 = Softer, sys3 = More
% damping, sys4 = Bryson):
%
%   Q1. WHICH MODEL PREDICTS THE REAL AERO BEST?
%       The gain K that was used on the AERO was computed on the Quanser
%       model. We simulate the closed loop with this SAME K, once with the
%       Quanser model as "plant" and once with the identified model as
%       "plant", and compare both with the measured test.
%
%   Q2. WHAT WOULD THE SAME DESIGN GIVE IF IT WAS COMPUTED ON THE NEW MODEL?
%       We recompute K with the same weights (Q, R) on the identified model
%       ("new K") and simulate it on the identified model. This is a
%       prediction for the next tests on the AERO.
%
% WHY "DESIGN MODEL" AND "PLANT MODEL" ARE SEPARATED
% --------------------------------------------------
% lqr(A, B, Q, R) gives a different K for each model. To compare the models
% with a measured test, the simulation must use the K that was really on
% the AERO (computed on the Quanser model), otherwise we would compare two
% different controllers.
%
% WHAT IS COMPARED (numerical indicators, for every design)
% ---------------------------------------------------------
%   fit pitch / yaw  (%)  : how close a simulation is to the measurement
%                           (100 % = identical, 0 % = as good as a constant)
%   RMSE pitch / yaw (deg): tracking error sqrt(mean((ref - angle)^2))
%   e_ss pitch / yaw (deg): mean absolute error over the last second of each
%                           plateau of the reference (steady-state error)
%   OS pitch / yaw   (%)  : mean overshoot of the moves
%   tr pitch / yaw   (s)  : mean 10-90 % rise time of the moves (with ramp
%                           references it includes the ramp itself: compare
%                           between models, not with step tests)
%   coupling (deg)        : largest motion of one axis while the OTHER axis
%                           moves and its own reference is constant
%                           (pitch moved by yaw moves, yaw moved by pitch moves)
%   saturation (%)        : time with |V| >= 99 % of the +/-20 V limit
%   activity (V/s)        : sum(|dV|)/T, how much the voltage moves
%                           (derivative noise on the AERO)
%   max K_vel (V.s/rad)   : largest velocity gain of K (amplifies the noise)
%   slowest pole (rad/s)  : closed-loop pole closest to the imaginary axis
%
% FIGURES (one window with tabs)
% ------------------------------
%   one tab per design : pitch, yaw, Vp, Vy vs time
%                        black = measured, red = sim Quanser model,
%                        blue = sim identified model (same K as the AERO),
%                        green dotted = identified model with the new K
%   "Comparison" tab   : bar charts of the main indicators, 4 designs
%   "New K" tab        : table of the new gains and their predicted indicators
%
% LIMITS OF THE SIMULATION
% ------------------------
% Both models are linear: no V^2 thrust, no dry friction on the yaw, no
% centrifugal effect, and they use the exact velocities (the real loop
% differentiates the encoders, so the measured voltage activity is much
% larger than in simulation). The pitch delay (~0.05-0.12 s) is not modelled.
%
% FILES NEEDED (current folder)
% -----------------------------
%   quanser_aero_parameters.m                Quanser parameters (and Jp, Jy)
%   quanser_aero_parameters_identified_2.mat identified parameters
%   aero_lqr_rsp_<series>_sys1.mat ... sys4  measured LQR tests, 9 rows:
%     1 time | 2 Vp | 3 pitch ref (deg) | 4 pitch (deg) | 5 Vy | 6 yaw ref (deg)
%     7 yaw (deg) | 8 pitch propeller speed | 9 yaw propeller speed

clear; close all; clc;

%% ===================== 0. SETTINGS =====================
series    = 'fixed3';   % 'fixed2' = step references (pitch +/-20, yaw +/-40 deg)
                        % 'fixed3' = same with ramp references (Rate Limiter)
identFile = 'quanser_aero_parameters_identified_2.mat';
satV      = 20;         % voltage saturation (V), as before HIL Write
d2r       = pi/180;

% The 4 designs, in the same order as the files sys1 ... sys4: {name, Q, R}
% State order: x = [theta, psi, theta_dot, psi_dot] (rad, rad/s)
% Bryson's rule: Q(i,i) = 1/(largest acceptable x_i)^2, R(j,j) = 1/(largest acceptable u_j)^2
designs = {
    'Current',      diag([200 75 0 0]),   0.005*eye(2)
    'Softer (Rx4)', diag([200 75 0 0]),   0.02*eye(2)
    'More damping', diag([200 75 20 10]), 0.005*eye(2)
    'Bryson',       diag([1/(5*d2r)^2, 1/(10*d2r)^2, 1/(60*d2r)^2, 1/(60*d2r)^2]), eye(2)/15^2
    };
% In the fixed3 series, Bryson (sys4) was run with R = 0.005 instead of
% 1/15^2 = 0.0044 (forgotten change): we use the R that was really on the AERO.
if strcmp(series, 'fixed3')
    designs{4,3} = 0.005*eye(2);
end
nD = size(designs, 1);

% Names of the 4 simulation "sources" compared with the measurement
srcNames = {'measured', 'sim Quanser', 'sim identified', 'identified + new K'};

%% ===================== 1. THE TWO MODELS =====================
% aeroModel (local function at the end) builds A and B from the parameters
[Aq, Bq, Pq] = aeroModel('quanser',    identFile);   % old Quanser model
[Ai, Bi, Pi] = aeroModel('identified', identFile);   % new identified model

fprintf('===== Model parameters =====\n');
fprintf('          Quanser    identified   ratio\n');
pn = {'Ksp', 'Dp', 'Kpp', 'Kpy', 'Dy', 'Kyy', 'Kyp'};
for k = 1:numel(pn)
    fprintf('  %-4s  %10.5f  %10.5f   %5.2f\n', pn{k}, Pq.(pn{k}), Pi.(pn{k}), Pi.(pn{k})/Pq.(pn{k}));
end
fprintf('  Open-loop poles Quanser   : %s\n', mat2str(eig(Aq).', 3));
fprintf('  Open-loop poles identified: %s\n\n', mat2str(eig(Ai).', 3));

%% ===================== 2. SIMULATION OF THE 4 DESIGNS =====================
% res(d) stores, for design d: the measured signals, the 3 simulations,
% the gains and the indicators (one struct per source).
res = struct([]);

for d = 1:nD
    fileName = sprintf('aero_lqr_rsp_%s_sys%d', series, d);
    if ~isfile([fileName '.mat'])
        fprintf('File not found, design %d skipped: %s.mat\n', d, fileName);
        continue;
    end

    % --- Measured test (references, angles, voltages)
    [t, Vp, rP, yP, Vy, rY, yY] = loadLQRTest(fileName);
    Ts = t(2) - t(1);

    % --- Gains
    Q = designs{d,2};  R = designs{d,3};
    Kused = lqr(Aq, Bq, Q, R);   % K really used on the AERO (computed on Quanser)
    Knew  = lqr(Ai, Bi, Q, R);   % same weights, computed on the identified model

    % --- Discrete models (zero-order hold at Ts = 2 ms, like the digital loop)
    sysQd = c2d(ss(Aq, Bq, eye(4), zeros(4,2)), Ts);
    sysId = c2d(ss(Ai, Bi, eye(4), zeros(4,2)), Ts);

    % --- Closed-loop simulations with the measured references (rad)
    %     Y = [pitch yaw] (deg), U = [Vp Vy] (V)
    [YQ, UQ] = simulateLQR(sysQd.A, sysQd.B, Kused, t, rP*d2r, rY*d2r, satV);  % Quanser plant
    [YI, UI] = simulateLQR(sysId.A, sysId.B, Kused, t, rP*d2r, rY*d2r, satV);  % identified plant
    [YN, UN] = simulateLQR(sysId.A, sysId.B, Knew,  t, rP*d2r, rY*d2r, satV);  % identified + new K

    % --- Indicators for the 4 sources (same function for all, so they are comparable)
    M(1) = computeIndicators(t, rP, rY, yP,      yY,      Vp,      Vy,      satV);
    M(2) = computeIndicators(t, rP, rY, YQ(:,1), YQ(:,2), UQ(:,1), UQ(:,2), satV);
    M(3) = computeIndicators(t, rP, rY, YI(:,1), YI(:,2), UI(:,1), UI(:,2), satV);
    M(4) = computeIndicators(t, rP, rY, YN(:,1), YN(:,2), UN(:,1), UN(:,2), satV);

    % --- Fit of each model vs the measurement (only for the K used on the AERO)
    fitQ = [fitPercent(yP, YQ(:,1)), fitPercent(yY, YQ(:,2))];
    fitI = [fitPercent(yP, YI(:,1)), fitPercent(yY, YI(:,2))];

    % --- Closed-loop poles and velocity gains
    pQ = eig(Aq - Bq*Kused);   % what we designed (Quanser model)
    pI = eig(Ai - Bi*Kused);   % what the AERO really has (if the identified model is right)
    pN = eig(Ai - Bi*Knew);    % with the new K

    % --- Store
    res(d).name   = designs{d,1};
    res(d).file   = fileName;
    res(d).t = t;  res(d).rP = rP;  res(d).rY = rY;
    res(d).meas = [yP yY];  res(d).Umeas = [Vp Vy];
    res(d).YQ = YQ;  res(d).UQ = UQ;
    res(d).YI = YI;  res(d).UI = UI;
    res(d).YN = YN;  res(d).UN = UN;
    res(d).Kused = Kused;  res(d).Knew = Knew;
    res(d).M = M;  res(d).fitQ = fitQ;  res(d).fitI = fitI;
    res(d).slowQ = slowestPole(pQ);  res(d).slowI = slowestPole(pI);  res(d).slowN = slowestPole(pN);
    res(d).kvelUsed = max(max(abs(Kused(:,3:4))));
    res(d).kvelNew  = max(max(abs(Knew(:,3:4))));

    % --- Table in the Command Window
    printDesignTable(d, res(d), srcNames);
end

valid = find(~arrayfun(@(r) isempty(r.name), res));   % designs with a measured file
if isempty(valid)
    error('No measured LQR file found for series %s.', series);
end

%% ===================== 3. SUMMARY: WHICH MODEL PREDICTS THE AERO BEST? =====================
fprintf('\n===== Summary (%s): fit of the simulation vs the measurement =====\n', series);
fprintf('%-14s | %-21s | %-21s\n', '', 'pitch fit (%)', 'yaw fit (%)');
fprintf('%-14s | %9s %11s | %9s %11s\n', 'design', 'Quanser', 'identified', 'Quanser', 'identified');
for d = valid
    fprintf('%-14s | %9.1f %11.1f | %9.1f %11.1f\n', res(d).name, ...
            res(d).fitQ(1), res(d).fitI(1), res(d).fitQ(2), res(d).fitI(2));
end
fprintf('\nPitch steady-state error (deg): measured / sim Quanser / sim identified\n');
for d = valid
    fprintf('  %-14s %6.2f / %6.2f / %6.2f\n', res(d).name, ...
            res(d).M(1).essP, res(d).M(2).essP, res(d).M(3).essP);
end

%% ===================== 4. FIGURES (one window, one tab per design) =====================
fig = figure('Name', ['LQR: Quanser vs identified model - ' series], ...
             'NumberTitle', 'off', 'Position', [60 40 1150 850], 'Color', 'w');
tg = uitabgroup(fig);
cMeas = [0 0 0];  cQ = [0.85 0.2 0.1];  cI = [0.1 0.35 0.85];  cN = [0.1 0.6 0.2];

for d = valid
    r   = res(d);
    tab = uitab(tg, 'Title', sprintf('%d. %s', d, r.name));
    tl  = tiledlayout(tab, 4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    title(tl, sprintf(['%s (%s) - fit pitch: Quanser %.0f %%, identified %.0f %%  |  ' ...
                       'fit yaw: Quanser %.0f %%, identified %.0f %%'], ...
                       r.name, r.file, r.fitQ(1), r.fitI(1), r.fitQ(2), r.fitI(2)), ...
          'Interpreter', 'none', 'FontSize', 10);

    % Pitch
    ax(1) = nexttile(tl); hold(ax(1), 'on'); grid(ax(1), 'on');
    plot(ax(1), r.t, r.rP, 'k--');
    plot(ax(1), r.t, r.meas(:,1), 'Color', cMeas, 'LineWidth', 1.5);
    plot(ax(1), r.t, r.YQ(:,1), 'Color', cQ);
    plot(ax(1), r.t, r.YI(:,1), 'Color', cI);
    plot(ax(1), r.t, r.YN(:,1), ':', 'Color', cN, 'LineWidth', 1.2);
    ylabel(ax(1), '\theta (deg)');
    legend(ax(1), 'reference', 'measured', 'sim Quanser', 'sim identified', ...
           'identified + new K', 'Location', 'eastoutside');

    % Yaw
    ax(2) = nexttile(tl); hold(ax(2), 'on'); grid(ax(2), 'on');
    plot(ax(2), r.t, r.rY, 'k--');
    plot(ax(2), r.t, r.meas(:,2), 'Color', cMeas, 'LineWidth', 1.5);
    plot(ax(2), r.t, r.YQ(:,2), 'Color', cQ);
    plot(ax(2), r.t, r.YI(:,2), 'Color', cI);
    plot(ax(2), r.t, r.YN(:,2), ':', 'Color', cN, 'LineWidth', 1.2);
    ylabel(ax(2), '\psi (deg)');
    legend(ax(2), 'reference', 'measured', 'sim Quanser', 'sim identified', ...
           'identified + new K', 'Location', 'eastoutside');

    % Pitch voltage
    ax(3) = nexttile(tl); hold(ax(3), 'on'); grid(ax(3), 'on');
    plot(ax(3), r.t, r.Umeas(:,1), 'Color', [0.6 0.6 0.6]);
    plot(ax(3), r.t, r.UQ(:,1), 'Color', cQ);
    plot(ax(3), r.t, r.UI(:,1), 'Color', cI);
    plot(ax(3), r.t, r.UN(:,1), ':', 'Color', cN, 'LineWidth', 1.2);
    ylabel(ax(3), 'V_p (V)'); ylim(ax(3), [-satV-2 satV+2]);
    legend(ax(3), 'measured', 'sim Quanser', 'sim identified', 'identified + new K', ...
           'Location', 'eastoutside');

    % Yaw voltage
    ax(4) = nexttile(tl); hold(ax(4), 'on'); grid(ax(4), 'on');
    plot(ax(4), r.t, r.Umeas(:,2), 'Color', [0.6 0.6 0.6]);
    plot(ax(4), r.t, r.UQ(:,2), 'Color', cQ);
    plot(ax(4), r.t, r.UI(:,2), 'Color', cI);
    plot(ax(4), r.t, r.UN(:,2), ':', 'Color', cN, 'LineWidth', 1.2);
    ylabel(ax(4), 'V_y (V)'); ylim(ax(4), [-satV-2 satV+2]); xlabel(ax(4), 'Time (s)');
    legend(ax(4), 'measured', 'sim Quanser', 'sim identified', 'identified + new K', ...
           'Location', 'eastoutside');

    linkaxes(ax, 'x');
    xlim(ax(1), [r.t(1) r.t(end)]);
end

%% ===================== 5. COMPARISON TAB (bar charts) =====================
% Each bar chart: one group of bars per design.
names = {res(valid).name};
tabC  = uitab(tg, 'Title', 'Comparison');
tlc   = tiledlayout(tabC, 2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tlc, sprintf('Quanser model vs identified model (%s)', series), 'FontSize', 11);

% (a) fit of the pitch: which model is closer to the measurement
bx = nexttile(tlc);
bar(bx, [arrayfun(@(r) r.fitQ(1), res(valid)); arrayfun(@(r) r.fitI(1), res(valid))]');
title(bx, 'Pitch fit vs measured (%)'); legend(bx, 'Quanser', 'identified', 'Location', 'best');
setNames(bx, names);

% (b) fit of the yaw
bx = nexttile(tlc);
bar(bx, [arrayfun(@(r) r.fitQ(2), res(valid)); arrayfun(@(r) r.fitI(2), res(valid))]');
title(bx, 'Yaw fit vs measured (%)'); legend(bx, 'Quanser', 'identified', 'Location', 'best');
setNames(bx, names);

% (c) pitch steady-state error: measured vs both simulations
bx = nexttile(tlc);
bar(bx, indicatorMatrix(res(valid), 'essP', 1:3));
title(bx, 'Pitch steady-state error (deg)');
legend(bx, srcNames(1:3), 'Location', 'best'); setNames(bx, names);

% (d) yaw overshoot
bx = nexttile(tlc);
bar(bx, indicatorMatrix(res(valid), 'osY', 1:3));
title(bx, 'Yaw overshoot (%)');
legend(bx, srcNames(1:3), 'Location', 'best'); setNames(bx, names);

% (e) coupling: pitch moved by the yaw moves
bx = nexttile(tlc);
bar(bx, indicatorMatrix(res(valid), 'coupPbyY', 1:3));
title(bx, 'Pitch moved by yaw moves (deg)');
legend(bx, srcNames(1:3), 'Location', 'best'); setNames(bx, names);

% (f) coupling: yaw moved by the pitch moves
bx = nexttile(tlc);
bar(bx, indicatorMatrix(res(valid), 'coupYbyP', 1:3));
title(bx, 'Yaw moved by pitch moves (deg)');
legend(bx, srcNames(1:3), 'Location', 'best'); setNames(bx, names);

%% ===================== 6. "NEW K" TAB (prediction for the next AERO tests) =====================
% Predicted indicators of each design when K is recomputed on the
% identified model, next to the indicators measured with the old K.
tabN = uitab(tg, 'Title', 'New K');
rows = cell(numel(valid), 11);
for i = 1:numel(valid)
    r = res(valid(i));
    rows(i,:) = {r.name, ...
                 sprintf('%.2f -> %.2f', r.M(1).essP, r.M(4).essP), ...
                 sprintf('%.1f -> %.1f', r.M(1).osY,  r.M(4).osY), ...
                 sprintf('%.2f -> %.2f', r.M(1).trP,  r.M(4).trP), ...
                 sprintf('%.2f -> %.2f', r.M(1).trY,  r.M(4).trY), ...
                 sprintf('%.1f -> %.1f', r.M(1).sat,  r.M(4).sat), ...
                 sprintf('%.1f -> %.1f', r.kvelUsed,  r.kvelNew), ...
                 sprintf('%.2f -> %.2f', r.slowI,     r.slowN), ...
                 mat2str(round(r.Knew(1,:), 1)), mat2str(round(r.Knew(2,:), 1)), ...
                 sprintf('%.1f -> %.1f', r.M(1).coupPbyY, r.M(4).coupPbyY)};
end
uitable(tabN, 'Data', rows, 'Units', 'normalized', 'Position', [0.01 0.30 0.98 0.68], ...
        'ColumnName', {'Design', 'e_ss pitch (deg)', 'OS yaw (%)', 'tr pitch (s)', ...
                       'tr yaw (s)', 'sat (%)', 'max K_vel', 'slowest pole', ...
                       'new K row 1 (Vp)', 'new K row 2 (Vy)', 'pitch by yaw (deg)'}, ...
        'ColumnWidth', {100, 105, 85, 85, 85, 75, 80, 90, 210, 210, 110});
uicontrol(tabN, 'Style', 'text', 'Units', 'normalized', 'Position', [0.01 0.02 0.98 0.25], ...
          'HorizontalAlignment', 'left', 'FontSize', 10, 'String', { ...
          'How to read this table: "A -> B" = A measured on the AERO with the old K (computed on the Quanser model),', ...
          'B = prediction of the identified model with the new K (same Q and R, computed on the identified model).', ...
          'Slowest pole: real part of the slowest closed-loop pole (rad/s), old K on the identified model -> new K.', ...
          'max K_vel (V.s/rad): largest velocity gain of K; a larger value amplifies the encoder derivative noise.', ...
          'Use the new K rows in the state-feedback gain block of the Simulink model, with the ramp references.'});

%% ===================== 7. SAVE =====================
save(['results_model_refined_' series '.mat'], 'res', 'designs', 'series', 'Aq', 'Bq', 'Ai', 'Bi', 'Pq', 'Pi');
fprintf('\nSaved: results_model_refined_%s.mat\n', series);

%% ======================= LOCAL FUNCTIONS =======================

function [A, B, P] = aeroModel(source, identFile)
% aeroModel  Builds the 4-state Quanser model.
%   source = 'quanser'    : parameters of quanser_aero_parameters.m
%   source = 'identified' : same Jp, Jy, other parameters from identFile
% Returns A, B and a structure P with the 7 parameters (for the printout).
quanser_aero_parameters;                % Jp, Jy (geometry) + default values
if strcmp(source, 'identified')
    S = load(identFile);                % overwrite with the identified values
    Ksp = S.Ksp; Dp = S.Dp; Kpp = S.Kpp; Kpy = S.Kpy;
    Dy  = S.Dy;  Kyy = S.Kyy; Kyp = S.Kyp;
end
A = [0 0 1 0;
     0 0 0 1;
    -Ksp/Jp 0 -Dp/Jp 0;
     0 0 0 -Dy/Jy];
B = [0 0;
     0 0;
     Kpp/Jp Kpy/Jp;
     Kyp/Jy Kyy/Jy];
P = struct('Ksp', Ksp, 'Dp', Dp, 'Kpp', Kpp, 'Kpy', Kpy, 'Dy', Dy, 'Kyy', Kyy, 'Kyp', Kyp);
end

function [t, Vp, rP, yP, Vy, rY, yY] = loadLQRTest(fileName)
% loadLQRTest  Loads a measured LQR test (9 rows, whatever the variable name).
S    = load([fileName '.mat']);
fn   = fieldnames(S);
data = S.(fn{1});
if size(data,1) > size(data,2), data = data'; end   % rows = signals
t  = data(1,:)';     % time (s)
Vp = data(2,:)';     % pitch voltage (V)
rP = data(3,:)';     % pitch reference (deg)
yP = data(4,:)';     % pitch angle (deg)
Vy = data(5,:)';     % yaw voltage (V)
rY = data(6,:)';     % yaw reference (deg)
yY = data(7,:)';     % yaw angle (deg)
end

function [Y, U] = simulateLQR(Ad, Bd, K, t, refP, refY, satV)
% simulateLQR  Discrete simulation of the closed loop (same law as on the AERO):
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
    u  = min(max(u, -satV), satV);       % +/- satV saturation
    Y(k,:) = x(1:2)' * 180/pi;
    U(k,:) = u';
    x = Ad*x + Bd*u;
end
end

function M = computeIndicators(t, rP, rY, yP, yY, Vp, Vy, satV)
% computeIndicators  All the numerical indicators of one closed-loop run.
% Same function for the measurement and the simulations, so they compare.
T = t(end) - t(1);
M.rmseP = sqrt(mean((rP - yP).^2));                 % tracking RMSE (deg)
M.rmseY = sqrt(mean((rY - yY).^2));
mvP = findMoves(t, rP);                             % moves of the pitch reference
mvY = findMoves(t, rY);                             % moves of the yaw reference
[M.trP, M.osP, M.essP] = moveSummary(t, yP, mvP);
[M.trY, M.osY, M.essY] = moveSummary(t, yY, mvY);
M.coupPbyY = coupling(yP, rP, mvY);                 % pitch moved by yaw moves
M.coupYbyP = coupling(yY, rY, mvP);                 % yaw moved by pitch moves
M.sat  = 100*mean(abs([Vp; Vy]) >= 0.99*satV);      % % of samples saturated
M.actP = sum(abs(diff(Vp))) / T;                    % voltage activity (V/s)
M.actY = sum(abs(diff(Vy))) / T;
end

function mv = findMoves(t, ref)
% findMoves  Finds the moves of a reference (steps or ramps).
% A plateau = the reference stays constant for at least 1 s. A move goes
% from the end of one plateau to the end of the next one (so a whole ramp
% counts as ONE move, its target being the next plateau).
% Each row of mv = [first sample, last sample, start value, target value].
Ts       = t(2) - t(1);
minPlat  = 1.0;                                   % (s) minimum plateau length
minStep  = 1.0;                                   % (deg) ignore tiny changes
newVal   = [true; abs(diff(ref)) > 1e-9];         % a new value starts here
runStart = find(newVal);
runEnd   = [runStart(2:end) - 1; numel(ref)];
keep     = (runEnd - runStart + 1)*Ts >= minPlat;
pS = runStart(keep);  pE = runEnd(keep);
mv = zeros(0,4);
for j = 2:numel(pS)
    r0 = ref(pE(j-1));  r1 = ref(pS(j));
    if abs(r1 - r0) >= minStep
        mv(end+1,:) = [pE(j-1)+1, pE(j), r0, r1]; %#ok<AGROW>
    end
end
end

function [tr, OS, ess] = moveSummary(t, y, mv)
% moveSummary  Mean rise time 10-90 % (s), mean overshoot (%) and mean
% absolute steady-state error (deg, last second of the plateau) of the moves.
Ts = t(2) - t(1);
n  = size(mv,1);
trs = nan(n,1);  oss = nan(n,1);  es = nan(n,1);
for m = 1:n
    k0 = mv(m,1);  k1 = mv(m,2);  r1 = mv(m,4);
    pre = max(1, k0 - round(0.2/Ts)):max(1, k0 - 1);
    y0  = mean(y(pre));                          % angle just before the move
    idx = (k0:k1)';
    z   = (y(idx) - y0) / (r1 - y0);             % normalised response (0 -> 1)
    tt  = t(idx) - t(k0);
    i10 = find(z >= 0.1, 1);  i90 = find(z >= 0.9, 1);
    if ~isempty(i10) && ~isempty(i90), trs(m) = tt(i90) - tt(i10); end
    oss(m) = max(0, 100*(max(z) - 1));
    es(m)  = abs(r1 - mean(y(idx(tt >= tt(end) - 1))));
end
tr  = mean(trs, 'omitnan');
OS  = mean(oss, 'omitnan');
ess = mean(es,  'omitnan');
end

function c = coupling(yOther, rOther, mv)
% coupling  Largest motion (deg) of one axis during the moves of the OTHER
% axis, only for the moves where its own reference stays constant.
c = 0;
for m = 1:size(mv,1)
    idx = mv(m,1):mv(m,2);
    if all(rOther(idx) == rOther(idx(1)))
        c = max(c, max(abs(yOther(idx) - yOther(idx(1)))));
    end
end
end

function f = fitPercent(y, ys)
% fitPercent  100*(1 - ||y - ys|| / ||y - mean(y)||): 100 % = identical.
f = 100 * (1 - norm(y - ys) / norm(y - mean(y)));
end

function s = slowestPole(p)
% slowestPole  Real part of the closed-loop pole closest to the imaginary axis.
s = max(real(p));
end

function X = indicatorMatrix(r, field, src)
% indicatorMatrix  Matrix (designs x sources) of one indicator, for bar().
X = zeros(numel(r), numel(src));
for i = 1:numel(r)
    for j = 1:numel(src)
        X(i,j) = r(i).M(src(j)).(field);
    end
end
end

function setNames(ax, names)
% setNames  Design names under the groups of bars.
set(ax, 'XTick', 1:numel(names), 'XTickLabel', names);
grid(ax, 'on');
end

function printDesignTable(d, r, srcNames)
% printDesignTable  Table of all the indicators of one design.
fprintf('\n===== Design %d: %s  (%s) =====\n', d, r.name, r.file);
fprintf('  fit vs measured (%%): pitch Quanser %.1f | identified %.1f    yaw Quanser %.1f | identified %.1f\n', ...
        r.fitQ(1), r.fitI(1), r.fitQ(2), r.fitI(2));
fprintf('  slowest closed-loop pole (rad/s): designed (Quanser) %.2f | AERO (identified) %.2f | new K %.2f\n', ...
        r.slowQ, r.slowI, r.slowN);
fprintf('  max velocity gain (V.s/rad): K used %.1f | new K %.1f\n', r.kvelUsed, r.kvelNew);
fprintf('  %-22s %10s %12s %15s %19s\n', 'indicator', srcNames{:});
rowsDef = {'RMSE pitch (deg)', 'rmseP'; 'RMSE yaw (deg)', 'rmseY'; ...
           'e_ss pitch (deg)', 'essP';  'e_ss yaw (deg)', 'essY'; ...
           'OS pitch (%)', 'osP';       'OS yaw (%)', 'osY'; ...
           'tr pitch (s)', 'trP';       'tr yaw (s)', 'trY'; ...
           'pitch by yaw (deg)', 'coupPbyY'; 'yaw by pitch (deg)', 'coupYbyP'; ...
           'saturation (%)', 'sat';     'activity Vp (V/s)', 'actP'; ...
           'activity Vy (V/s)', 'actY'};
for k = 1:size(rowsDef,1)
    f = rowsDef{k,2};
    fprintf('  %-22s %10.2f %12.2f %15.2f %19.2f\n', rowsDef{k,1}, ...
            r.M(1).(f), r.M(2).(f), r.M(3).(f), r.M(4).(f));
end
fprintf('  K used (Quanser model)   = [%s ; %s]\n', num2str(r.Kused(1,:), '%8.2f'), num2str(r.Kused(2,:), '%8.2f'));
fprintf('  new K (identified model) = [%s ; %s]\n', num2str(r.Knew(1,:),  '%8.2f'), num2str(r.Knew(2,:),  '%8.2f'));
end
