%% plot_yaw_linearity_AERO.m
% =========================================================================
%  YAW LINEARITY TEST: SERIES OF PULSES 0 -> V -> 0 ON THE YAW MOTOR
% =========================================================================
%
% TEST
% ----
% Yaw free, Vp = 0. Repeating Sequence Stair on Vy:
%   0 -> 1 V -> 0 -> 2 V -> 0 ... (5 s per level, 5 s at 0 V between levels)
% Works with both recordings:
%   custom_yaw_0to20     : 1, 2, ... 20 V (the AERO disconnected during 17 V)
%   custom_yaw_0to12-12  : +1, -1, +2, -2, ... +12, -12 V
%
% WHAT A LINEAR SYSTEM WOULD DO (teacher's test = homogeneity principle)
% ------------------------------------------------------------------------
% For a linear system, multiplying the input by V multiplies the output by
% V and changes NOTHING else. So for every pulse:
%   - the gain per volt  K = (yaw rate reached) / V   must be the SAME,
%   - the time constant  tau (time to 63 % of that rate) must be the SAME,
%   - the responses divided by V must all fall on ONE curve,
%   - the yaw rate reached must be a straight line through 0 vs V.
% Any difference is a nonlinearity. The linear CONTSID model (G22, first
% order + integrator) is drawn as the reference: K22 = Kyy/Dy, T22 = Jy/Dy.
%
% FIGURES (for the report)
% ------------------------
%   1. Overview: Vy, yaw rate, yaw angle, pitch angle vs time
%   2. Linearity test: responses divided by V, K(V) and tau(V)
%   3. Static characteristic (rate reached vs V, dead zone, V^2 fit) and
%      stopping phase (deceleration at 0 V: constant = Coulomb friction)
%
% Data: 12-row data0 of AERO_basics.slx (4 pitch deg, 5 yaw deg, 12 Vy V).

clear; close all; clc;

%% ===================== 0. SETTINGS =====================
fileName  = 'custom_yaw_0to12-12';   % or 'custom_yaw_0to20'
addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'reponse_BO_experimentation'))   % identified parameters (.mat)
identFile = 'quanser_aero_parameters_identified.mat';   % linear CONTSID model (optional)
fricFile  = 'aero_friction_parameters.mat';             % friction model (optional)
nSmooth   = 25;      % samples of moving average before differentiating the yaw
rMove     = 3;       % (deg/s) a pulse "moves" the yaw if the rate reached exceeds this
rStop     = 1;       % (deg/s) the yaw is considered stopped below this rate

% Colours (same meaning in all figures)
cPos   = [0.16 0.47 0.84];   % positive voltages
cNeg   = [0.92 0.41 0.20];   % negative voltages
cModel = [0.15 0.15 0.15];   % linear CONTSID model
cFric  = [0.10 0.60 0.30];   % friction model

%% ===================== 1. LOAD AND CLEAN THE DATA =====================
S = load([fileName '.mat']);
data0 = S.data0;
t            = data0(1,:)';     % time (s)
PitchEncoder = data0(4,:)';     % pitch (deg)
YawEncoder   = data0(5,:)';     % yaw (deg)
YawVoltage   = data0(12,:)';    % Vy (V)
Ts = t(2) - t(1);

% If the AERO disconnected, the encoders jump back to 0: cut the data at the
% first jump larger than 10 deg in one sample (impossible physically)
jump = find(abs(diff(YawEncoder)) > 10, 1);
if ~isempty(jump)
    fprintf('Encoder jump at t = %.1f s (disconnection): data cut there.\n', t(jump));
    keep = 1:jump;
    t = t(keep); PitchEncoder = PitchEncoder(keep); YawEncoder = YawEncoder(keep);
    YawVoltage = YawVoltage(keep);
end

% Yaw rate (deg/s): moving average then derivative
% (one encoder count per sample = 0.0879 deg / 2 ms = 44 deg/s without smoothing)
YawRate = gradient(movmean(YawEncoder, nSmooth), Ts);

%% ===================== 2. ONE LINE OF RESULTS PER PULSE =====================
% A pulse = the samples where Vy keeps the same non-zero value.
starts = find(abs(diff([0; YawVoltage])) > 1e-6 & abs(YawVoltage) > 1e-6);
nP = numel(starts);
P = struct('V', [], 'k0', [], 'k1', [], 'rEnd', [], 'K', [], 'tau', [], ...
           'moved', [], 'tStop', [], 'decel', []);
for i = 1:nP
    k0 = starts(i);
    k1 = k0;
    while k1 < numel(t) && YawVoltage(k1+1) == YawVoltage(k0), k1 = k1 + 1; end
    P(i).V  = YawVoltage(k0);
    P(i).k0 = k0;  P(i).k1 = k1;
end
% Keep only complete pulses (the last one may be cut by the disconnection)
dur = arrayfun(@(p) (p.k1 - p.k0 + 1)*Ts, P);
P   = P(dur >= 0.9*median(dur));
nP  = numel(P);
Tp  = median(dur);                       % pulse duration (5 s)

for i = 1:nP
    k0 = P(i).k0;  k1 = P(i).k1;
    % rate reached = mean over the last 0.5 s of the pulse
    P(i).rEnd  = mean(YawRate(k1 - round(0.5/Ts) + 1 : k1));
    P(i).K     = P(i).rEnd / P(i).V;               % gain per volt (deg/s per V)
    P(i).moved = YawEncoder(k1) - YawEncoder(k0);  % yaw displacement (deg)
    P(i).tau   = NaN;  P(i).tStop = NaN;  P(i).decel = NaN;
    if abs(P(i).rEnd) > rMove
        % time constant: first time the rate reaches 63 % of rEnd
        k63 = find(abs(YawRate(k0:k1)) >= 0.632*abs(P(i).rEnd), 1);
        P(i).tau = (k63 - 1)*Ts;
        % stopping time after Vy returns to 0 (only if the yaw really stops
        % before the next pulse / the end of the file)
        kEnd  = min(numel(t), k1 + round(Tp/Ts));
        kStop = find(abs(YawRate(k1+1:kEnd)) < rStop, 1);
        if ~isempty(kStop) && kStop > 1 && kEnd > k1 + kStop
            P(i).tStop = kStop*Ts;
            P(i).decel = abs(P(i).rEnd) / P(i).tStop;   % mean deceleration (deg/s^2)
        end
    end
end

V    = [P.V]';   rEnd = [P.rEnd]';   K = [P.K]';   tau = [P.tau]';
pos  = V > 0;    neg  = V < 0;
movesP = pos & abs(rEnd) > rMove;    movesN = neg & abs(rEnd) > rMove;

% Breakaway voltage: smallest |V| at which the yaw really moves
vBreakPos = min(V(movesP));  vBreakNeg = max(V(movesN));

% Table in the Command Window
fprintf('\n%s: %d complete pulses of %.1f s\n', fileName, nP, Tp);
fprintf('   V (V)  rate reached (deg/s)  K (deg/s/V)  tau63 (s)  moved (deg)  stop (s)  decel (deg/s^2)\n');
for i = 1:nP
    fprintf('  %6.1f  %18.1f  %11.2f  %9.2f  %11.1f  %8.2f  %10.1f\n', P(i).V, P(i).rEnd, ...
            P(i).K, P(i).tau, P(i).moved, P(i).tStop, P(i).decel);
end
if ~isempty(vBreakPos), fprintf('Breakaway voltage (V > 0): %.0f V\n', vBreakPos); end
if ~isempty(vBreakNeg), fprintf('Breakaway voltage (V < 0): %.0f V\n', vBreakNeg); end

%% ===================== 3. REFERENCE MODELS =====================
Jy = 0.02199;                         % yaw inertia (kg.m^2), Quanser value kept
haveLin = isfile(identFile);
if haveLin
    L   = load(identFile);
    K22 = L.Kyy / L.Dy * 180/pi;      % final yaw rate per volt (deg/s per V)
    T22 = Jy / L.Dy;                  % time constant (s)
    fprintf('Linear CONTSID model: K22 = %.2f deg/s/V, T22 = %.2f s\n', K22, T22);
end
haveFric = isfile(fricFile);
if haveFric
    F = load(fricFile);
    tauF = Jy / F.fricYaw.Dy;          % friction model time constant (s)
end

% V^2 fit of the rate reached, beyond the dead zone, for each sign:
%   |rate| = a * (V^2 - V0^2)   (thrust ~ V^2, minus a constant friction)
% -> straight line in V^2: polyfit gives a and V0
fitPos = []; fitNeg = [];
if nnz(movesP) >= 3
    c = polyfit(V(movesP).^2, abs(rEnd(movesP)), 1);  fitPos = [c(1), sqrt(max(-c(2)/c(1), 0))];
    fprintf('V^2 fit, V > 0: a = %.3f deg/s/V^2, V0 = %.2f V\n', fitPos);
end
if nnz(movesN) >= 3
    c = polyfit(V(movesN).^2, abs(rEnd(movesN)), 1);  fitNeg = [c(1), sqrt(max(-c(2)/c(1), 0))];
    fprintf('V^2 fit, V < 0: a = %.3f deg/s/V^2, V0 = %.2f V\n', fitNeg);
end
decelMean = mean([P.decel], 'omitnan');
fprintf('Mean deceleration when Vy = 0: %.1f deg/s^2 -> Fc = Jy*decel = %.4f N.m\n', ...
        decelMean, Jy*decelMean*pi/180);

%% ===================== FIGURE 1: OVERVIEW =====================
figure('Name', 'Yaw pulses - overview', 'Color', 'w', 'Position', [60 60 1000 800]);
ax(1) = subplot(4,1,1);
plot(t, YawVoltage, 'k', 'LineWidth', 1.2); grid on;
ylabel('V_y (V)'); title(['Yaw pulse test - ' fileName], 'Interpreter', 'none');
ax(2) = subplot(4,1,2);
plot(t, YawRate, 'Color', cPos, 'LineWidth', 1); grid on;
ylabel('d\psi/dt (deg/s)');
ax(3) = subplot(4,1,3);
plot(t, YawEncoder, 'Color', cPos, 'LineWidth', 1.2); grid on;
ylabel('\psi (deg)');
ax(4) = subplot(4,1,4);
plot(t, PitchEncoder, 'Color', cNeg, 'LineWidth', 1.2); grid on;
ylabel('\theta (deg)'); xlabel('Time (s)');
title('Pitch moved by the yaw motor (G12 coupling)');
linkaxes(ax, 'x'); xlim(ax(1), [t(1) t(end)]);

%% ===================== FIGURE 2: LINEARITY (HOMOGENEITY) TEST =====================
figure('Name', 'Yaw pulses - linearity test', 'Color', 'w', 'Position', [80 60 1100 750]);

% (a) Responses divided by V: a linear system gives ONE curve
subplot(2,2,[1 2]); hold on; grid on;
cmap = turbo(256);
vMax = max(abs(V));
for i = 1:nP
    k0 = P(i).k0;
    kE = min(numel(t), k0 + round(2*Tp/Ts) - 1);    % pulse + following 0 V phase
    tt = t(k0:kE) - t(k0);
    col = cmap(max(1, round(abs(P(i).V)/vMax*256)), :);
    ls = '-';  if P(i).V < 0, ls = '--'; end
    plot(tt, YawRate(k0:kE)/P(i).V, ls, 'Color', col, 'LineWidth', 1.1, ...
         'HandleVisibility', 'off');
end
if haveLin
    tt = (0:Ts:2*Tp)';
    yLin = (1 - exp(-min(tt, Tp)/T22)) .* exp(-max(tt - Tp, 0)/T22);   % linear step response / V
    plot(tt, K22*yLin, 'k:', 'LineWidth', 2.5, 'DisplayName', 'linear model (CONTSID)');
    legend('Location', 'northwest');
end
xline(Tp, 'k-', 'V_y back to 0', 'HandleVisibility', 'off');
colormap(gca, cmap); clim([0 vMax]);
cb = colorbar; cb.Label.String = '|V_y| (V)';
xlabel('Time since the start of the pulse (s)');
ylabel('(d\psi/dt) / V_y   (deg/s per V)');
title('Yaw rate divided by the voltage: a linear system would give ONE curve (dashed = V < 0)');

% (b) Gain per volt K(V): constant for a linear system
subplot(2,2,3); hold on; grid on;
plot(abs(V(pos)), K(pos), 'o-', 'Color', cPos, 'MarkerFaceColor', cPos, 'LineWidth', 1.2, ...
     'DisplayName', 'V > 0');
if any(neg)
    plot(abs(V(neg)), K(neg), 's-', 'Color', cNeg, 'MarkerFaceColor', cNeg, 'LineWidth', 1.2, ...
         'DisplayName', 'V < 0');
end
if haveLin
    yline(K22*(1 - exp(-Tp/T22)), 'k--', 'LineWidth', 1.5, ...
          'DisplayName', sprintf('linear model (%.1f deg/s/V at %.0f s)', K22*(1-exp(-Tp/T22)), Tp));
end
xlabel('|V_y| (V)'); ylabel(sprintf('K = rate after %.0f s / V   (deg/s per V)', Tp));
title('Gain per volt'); legend('Location', 'northwest');

% (c) Time constant tau(V): constant for a linear system
subplot(2,2,4); hold on; grid on;
plot(abs(V(pos)), tau(pos), 'o-', 'Color', cPos, 'MarkerFaceColor', cPos, 'LineWidth', 1.2, ...
     'DisplayName', 'V > 0');
if any(neg)
    plot(abs(V(neg)), tau(neg), 's-', 'Color', cNeg, 'MarkerFaceColor', cNeg, 'LineWidth', 1.2, ...
         'DisplayName', 'V < 0');
end
if haveLin
    yline(T22, 'k--', 'LineWidth', 1.5, 'DisplayName', sprintf('linear model (%.1f s)', T22));
end
xlabel('|V_y| (V)'); ylabel('\tau_{63%} (s)');
title('Time constant (no value: the yaw did not move)'); legend('Location', 'northeast');
xlim([0 vMax + 1]); ylim([0 max([tau; 3]) + 0.5]);

%% ===================== FIGURE 3: STATIC CHARACTERISTIC AND STOPPING =====================
figure('Name', 'Yaw pulses - static characteristic', 'Color', 'w', 'Position', [100 60 1100 480]);

% (a) Rate reached vs V: a straight line through 0 for a linear system
subplot(1,2,1); hold on; grid on;
vv = linspace(-vMax, vMax, 400)';
% dead zone (no motion)
dzP = vBreakPos; dzN = vBreakNeg;
if isempty(dzP), dzP = vMax; end
if isempty(dzN), dzN = -dzP; end
fill([dzN dzP dzP dzN], [-1 -1 1 1]*1e3, [0.92 0.92 0.92], 'EdgeColor', 'none', ...
     'DisplayName', 'no motion (dead zone)');
if haveLin
    plot(vv, K22*(1 - exp(-Tp/T22))*vv, 'k--', 'LineWidth', 1.5, 'DisplayName', 'linear model');
end
if haveFric
    % friction model, rate after Tp: (Kyy|V| - Fc)/Dy * (1 - exp(-Tp/tau)) if Kyy|V| > Fc
    rF = sign(vv) .* max(F.fricYaw.Kyy*abs(vv) - F.fricYaw.Fc, 0) / F.fricYaw.Dy ...
         .* (1 - exp(-Tp/tauF)) * 180/pi;
    plot(vv, rF, '-', 'Color', cFric, 'LineWidth', 1.5, 'DisplayName', 'friction model');
end
if ~isempty(fitPos)
    vf = linspace(fitPos(2), max(V), 100);
    plot(vf, fitPos(1)*(vf.^2 - fitPos(2)^2), ':', 'Color', cPos, 'LineWidth', 2, ...
         'DisplayName', sprintf('fit a(V^2 - V_0^2), V_0 = %.1f V', fitPos(2)));
end
if ~isempty(fitNeg)
    vf = linspace(fitNeg(2), max(abs(V(neg))), 100);
    plot(-vf, -fitNeg(1)*(vf.^2 - fitNeg(2)^2), ':', 'Color', cNeg, 'LineWidth', 2, ...
         'DisplayName', sprintf('fit V < 0, V_0 = %.1f V', fitNeg(2)));
end
plot(V(pos), rEnd(pos), 'o', 'Color', cPos, 'MarkerFaceColor', cPos, 'MarkerSize', 7, ...
     'DisplayName', 'measured, V > 0');
if any(neg)
    plot(V(neg), rEnd(neg), 's', 'Color', cNeg, 'MarkerFaceColor', cNeg, 'MarkerSize', 7, ...
         'DisplayName', 'measured, V < 0');
end
ylim([min(rEnd) max(rEnd)]*1.15 + [-5 5]);  xlim([-vMax vMax]*1.05 + [-0.5 0.5]*any(neg));
if ~any(neg), xlim([0 vMax + 1]); end
xlabel('V_y (V)'); ylabel(sprintf('yaw rate after %.0f s (deg/s)', Tp));
title('Static characteristic: dead zone + growth faster than linear');
legend('Location', 'northwest');

% (b) Stopping phase: Coulomb friction -> constant deceleration -> stop time
%     proportional to the speed (straight line through 0)
subplot(1,2,2); hold on; grid on;
ok = ~isnan([P.tStop]');
tStop = [P.tStop]';
plot(abs(rEnd(ok & pos)), tStop(ok & pos), 'o', 'Color', cPos, 'MarkerFaceColor', cPos, ...
     'MarkerSize', 7, 'DisplayName', 'V > 0');
if any(ok & neg)
    plot(abs(rEnd(ok & neg)), tStop(ok & neg), 's', 'Color', cNeg, 'MarkerFaceColor', cNeg, ...
         'MarkerSize', 7, 'DisplayName', 'V < 0');
end
rr = linspace(0, max(abs(rEnd(ok))), 50);
plot(rr, rr/decelMean, 'k--', 'LineWidth', 1.5, ...
     'DisplayName', sprintf('constant deceleration %.0f deg/s^2', decelMean));
xlabel('yaw rate when V_y returns to 0 (deg/s)'); ylabel('time to stop (s)');
title('Stopping: constant deceleration = Coulomb friction'); legend('Location', 'northwest');

set(findall(0, 'Type', 'axes'), 'FontSize', 11);
