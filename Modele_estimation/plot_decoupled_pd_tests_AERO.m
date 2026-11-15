% plot_decoupled_pd_tests_AERO.m
% 14/10/2026
% Plots and analyses the MEASURED decoupled PD tests (9-row logs, same
% layout as the LQR logs):
%   1 time | 2 Vp (V) | 3 pitch ref (deg) | 4 pitch (deg) | 5 Vy (V)
%   6 yaw ref (deg) | 7 yaw (deg) | 8 pitch propeller speed | 9 yaw propeller speed
%
% Figures:
%   1) tracking: reference and measured angle, one column per test
%   2) COUPLING: motion of one axis while the OTHER moves, aligned on the
%      start of each move (this is where the decoupling is judged)
%   3) motor voltages
%   4) real pitch torque / commanded pitch torque on each pitch plateau,
%      against the holding voltage (shows the V^2 thrust)
%   5) the tests overlaid move by move (to see the effect of a change,
%      e.g. cKyp = 1 in test2 and 0.75 in test3)
% Printed: unit check, indicators (same definitions as decoupled_pd_AERO.m),
% signed coupling of each move (to tune cKpy / cKyp, see the report), and
% the decoupler that was REALLY in the loop (cKpy, cKyp recovered from the
% logged voltages: useful when you are not sure what was set).
%
% To compare a test with the simulation, use decoupled_pd_AERO.m with
% measFile = '<test file>' and iMeas = 2 (Decoupled PD).
%
% Files needed: the test .mat files and quanser_aero_parameters_identified.mat.

clear;
close all;
clc

%% 0) Settings
testFiles  = {'aero_dpd_rsp_test1', 'aero_dpd_rsp_test2', 'aero_dpd_rsp_test3'};
testLabels = {'test1 (0.5, cKyp 1)', 'test2 (1, cKyp 1)', 'test3 (1, cKyp 0.75)'};
PO = 7.5;  tp = 1.75;    % PD specs used in the tests (setup_decoupled_pd_simulink.m)
Wn_vel = 50;             % cut-off of the PIV velocity filter (rad/s)
zetaVel = 0.8;           % damping of the 2nd-order velocity filter
identFile  = 'quanser_aero_parameters_identified.mat';
satV       = 20;
couplingWindow = 7;      % (s) longest window after the start of a move
colors = lines(3);       % one color per move

P  = load(identFile);
Bt = [P.Kpp P.Kpy;
      P.Kyp P.Kyy];      % the decoupler's torque gains (cKpy = cKyp = 1)
nT = numel(testFiles);

%% 1) Load the tests
T = struct([]);
for i = 1:nT
    S    = load(testFiles{i});
    fn   = fieldnames(S);
    data = S.(fn{1});
    if size(data,1) > size(data,2)
        data = data';
    end
    T(i).t     = data(1,:)';
    T(i).Vp    = data(2,:)';
    T(i).refP  = data(3,:)';
    T(i).pitch = data(4,:)';
    T(i).Vy    = data(5,:)';
    T(i).refY  = data(6,:)';
    T(i).yaw   = data(7,:)';
    T(i).wP    = data(8,:)';
    T(i).wY    = data(9,:)';

    % Unit check: the smallest non-zero change of an angle must be one encoder count
    dP = abs(diff(T(i).pitch));  dP = min(dP(dP > 1e-9));
    dY = abs(diff(T(i).yaw));    dY = min(dY(dY > 1e-9));
    fprintf('%s: smallest step pitch %.4f deg (0.1758 expected), yaw %.4f deg (0.0879 expected)\n', ...
            testLabels{i}, dP, dY);
end
fprintf('\n');

%% 2) Indicators (same definitions as decoupled_pd_AERO.m)
fprintf('%-22s | %5s %5s | %5s %5s | %5s %5s | %6s %6s | %5s | %6s\n', 'Test', ...
        'e_p', 'e_y', 'OS_p', 'OS_y', 'RMS_p', 'RMS_y', 'cpl_p', 'cpl_y', 'sat', 'activ');
fprintf('%-22s | %5s %5s | %5s %5s | %5s %5s | %6s %6s | %5s | %6s\n', '', ...
        '(deg)', '(deg)', '(%)', '(%)', '(deg)', '(deg)', '(deg)', '(deg)', '(%)', '(V/s)');
for i = 1:nT
    m = loopMetrics(T(i).t, T(i).refP, T(i).refY, [T(i).pitch T(i).yaw], [T(i).Vp T(i).Vy], satV);
    fprintf('%-22s | %5.2f %5.2f | %5.1f %5.1f | %5.2f %5.2f | %6.2f %6.2f | %5.1f | %6.1f\n', testLabels{i}, ...
            m.essP, m.essY, m.osP, m.osY, m.rmseP, m.rmseY, m.coupP, m.coupY, m.sat, m.act);
end
fprintf('\n');

%% 3) Signed coupling of each move (to read the sign rules of the report)
%   pitch goes UP at the start of a POSITIVE yaw move   -> Kpy too small in the decoupler (increase cKpy)
%   yaw turns POSITIVE when the pitch goes UP / stays up -> Kyp too negative in the decoupler (decrease cKyp)
for i = 1:nT
    fprintf('--- %s ---\n', testLabels{i});
    t = T(i).t;  Ts = t(2) - t(1);
    [kS, kE] = findMoves(T(i).refY);
    for k = 1:numel(kS)
        idx = couplingIndices(kS, kE, k, findMoves(T(i).refP), length(t), Ts, couplingWindow);
        d = T(i).pitch(idx) - T(i).pitch(kS(k));
        [dMax, iMax] = max(d);  [dMin, iMin] = min(d);
        fprintf('  yaw move %+5.0f deg at %4.1f s: pitch max %+5.2f deg (+%.2f s), min %+5.2f deg (+%.2f s)\n', ...
                T(i).refY(kE(k)) - T(i).refY(kS(k)-1), t(kS(k)), dMax, t(idx(iMax)) - t(kS(k)), dMin, t(idx(iMin)) - t(kS(k)));
    end
    [kS, kE] = findMoves(T(i).refP);
    for k = 1:numel(kS)
        idx = couplingIndices(kS, kE, k, findMoves(T(i).refY), length(t), Ts, couplingWindow);
        d = T(i).yaw(idx) - T(i).yaw(kS(k));
        fprintf('  pitch move %+5.0f deg at %4.1f s: yaw at the end of the window %+5.2f deg (max |dev| %.2f deg)\n', ...
                T(i).refP(kE(k)) - T(i).refP(kS(k)-1), t(kS(k)), d(end), max(abs(d)));
    end
end
fprintf('\n');

%% 4) Which decoupler was in the loop? (cKpy, cKyp recovered from the log)
% The PD torques are rebuilt from the logged references and angles (same
% gains, same 2nd-order velocity filter as the PIV block). The decoupler
% satisfies  tau = Bt_dec * V,  so each row of Bt_dec is found by least
% squares between the rebuilt torques and the logged voltages.
% Row 1 = [Kpp, cKpy*Kpy], row 2 = [cKyp*Kyp, Kyy].
zeta = -log(PO/100) / sqrt(log(PO/100)^2 + pi^2);
wn   = pi / (tp*sqrt(1 - zeta^2));
kp_p = P.Jp*wn^2 - P.Ksp;   kd_p = 2*zeta*wn*P.Jp - P.Dp;
kp_y = P.Jy*wn^2;           kd_y = 2*zeta*wn*P.Jy - P.Dy;
fprintf('Decoupler recovered from the logs (velocity filter %g rad/s):\n', Wn_vel);
for i = 1:nT
    t   = T(i).t;
    thd = filteredDerivative(T(i).pitch*pi/180, t, Wn_vel, zetaVel);
    psd = filteredDerivative(T(i).yaw*pi/180,   t, Wn_vel, zetaVel);
    tauP = kp_p*(T(i).refP - T(i).pitch)*pi/180 - kd_p*thd + P.Ksp*T(i).refP*pi/180;
    tauY = kp_y*(T(i).refY - T(i).yaw)*pi/180   - kd_y*psd;
    V    = [T(i).Vp T(i).Vy];
    ok   = all(abs(V) < 0.99*satV, 2);        % saturated samples do not follow V = W*tau
    rowP = V(ok,:) \ tauP(ok);                % [Kpp, cKpy*Kpy]
    rowY = V(ok,:) \ tauY(ok);                % [cKyp*Kyp, Kyy]
    fitP = 100*(1 - norm(V(ok,:)*rowP - tauP(ok)) / norm(tauP(ok) - mean(tauP(ok))));
    fitY = 100*(1 - norm(V(ok,:)*rowY - tauY(ok)) / norm(tauY(ok) - mean(tauY(ok))));
    fprintf('  %-22s cKpy = %.2f, cKyp = %.2f  (Kpp x%.2f, Kyy x%.2f; fit %.0f / %.0f %%)\n', testLabels{i}, ...
            rowP(2)/P.Kpy, rowY(1)/P.Kyp, rowP(1)/P.Kpp, rowY(2)/P.Kyy, fitP, fitY);
end
fprintf('  (fit > 95 %% and Kpp, Kyy close to x1.00: the rebuilt controller matches the one that ran)\n\n');

%% 4b) Pitch torque efficiency on each plateau
% At rest, the spring torque Ksp*theta is what the motors really give.
% The controller commands tau_p = Kpp*Vp + Kpy*Vy (decoupler model).
% Ratio < 1: the motors give less torque than the model says (V^2 thrust).
% Dry friction makes the ratio uncertain by +/- Fc_p/tau_p (Fc_p = 0.00057 N.m).
FcP = 0.00057;
effV = [];  effR = [];  effE = [];  effTest = [];
fprintf('Pitch plateaus: |Vp|, real/commanded pitch torque\n');
for i = 1:nT
    t  = T(i).t;  Ts = t(2) - t(1);
    [kS, ~] = findMoves(T(i).refP);
    for k = 1:numel(kS)
        if k < numel(kS), k1 = kS(k+1) - 1; else, k1 = length(t); end
        last = (k1 - round(1/Ts) + 1):k1;                 % last second of the plateau
        if abs(T(i).refP(k1)) < 1, continue; end          % skip the 0 deg plateaus (no holding torque)
        tauCmd  = P.Kpp*mean(T(i).Vp(last)) + P.Kpy*mean(T(i).Vy(last));
        tauReal = P.Ksp*mean(T(i).pitch(last))*pi/180;
        effV(end+1) = abs(mean(T(i).Vp(last)));          %#ok<SAGROW>
        effR(end+1) = tauReal/tauCmd;                     %#ok<SAGROW>
        effE(end+1) = FcP/abs(tauCmd);                    %#ok<SAGROW>
        effTest(end+1) = i;                               %#ok<SAGROW>
        fprintf('  %s, t = %4.1f s: |Vp| = %5.2f V, ratio %.2f (+/- %.2f)\n', testLabels{i}, t(k1), effV(end), effR(end), effE(end));
    end
end
fprintf('\n');

%% 5) Figure 1: tracking
figure('Name', 'Decoupled PD tests: tracking', 'Color', 'w');
ax = gobjects(2, nT);
for i = 1:nT
    ax(1,i) = subplot(2, nT, i);       hold on;  grid on
    plot(T(i).t, T(i).refP,  'k--', 'LineWidth', 1)
    plot(T(i).t, T(i).pitch, 'Color', colors(1,:), 'LineWidth', 1.5)
    title(testLabels{i})
    ylabel('\theta (deg)')
    legend('Reference', 'Measured', 'Location', 'southwest')
    ax(2,i) = subplot(2, nT, nT + i);  hold on;  grid on
    plot(T(i).t, T(i).refY, 'k--', 'LineWidth', 1)
    plot(T(i).t, T(i).yaw,  'Color', colors(1,:), 'LineWidth', 1.5)
    ylabel('\psi (deg)')
    xlabel('Time (s)')
end
linkaxes(ax(:), 'x')
xlim(ax(1,1), [T(1).t(1) T(1).t(end)])

%% 6) Figure 2: coupling, aligned on the start of each move
figure('Name', 'Decoupled PD tests: coupling', 'Color', 'w');
bx = gobjects(2, nT);
for i = 1:nT
    t = T(i).t;  Ts = t(2) - t(1);
    % pitch motion during the yaw moves
    bx(1,i) = subplot(2, nT, i);  hold on;  grid on
    [kS, kE] = findMoves(T(i).refY);
    names = {};
    for k = 1:numel(kS)
        idx = couplingIndices(kS, kE, k, findMoves(T(i).refP), length(t), Ts, couplingWindow);
        plot(t(idx) - t(kS(k)), T(i).pitch(idx) - T(i).pitch(kS(k)), 'Color', colors(k,:), 'LineWidth', 1.5)
        names{end+1} = sprintf('yaw %+.0f deg at %.0f s (\\theta_{ref} %+.0f deg)', ...
                               T(i).refY(kE(k)) - T(i).refY(kS(k)-1), t(kS(k)), T(i).refP(kS(k))); %#ok<SAGROW>
    end
    yline(0, 'k:');
    title([testLabels{i} ': pitch moved by a YAW move'])
    ylabel('\Delta\theta (deg)')
    legend(names, 'Location', 'southwest')
    % yaw motion during the pitch moves
    bx(2,i) = subplot(2, nT, nT + i);  hold on;  grid on
    [kS, kE] = findMoves(T(i).refP);
    names = {};
    for k = 1:numel(kS)
        idx = couplingIndices(kS, kE, k, findMoves(T(i).refY), length(t), Ts, couplingWindow);
        plot(t(idx) - t(kS(k)), T(i).yaw(idx) - T(i).yaw(kS(k)), 'Color', colors(k,:), 'LineWidth', 1.5)
        names{end+1} = sprintf('pitch %+.0f deg at %.0f s', T(i).refP(kE(k)) - T(i).refP(kS(k)-1), t(kS(k))); %#ok<SAGROW>
    end
    yline(0, 'k:');
    title([testLabels{i} ': yaw moved by a PITCH move'])
    ylabel('\Delta\psi (deg)')
    xlabel('Time since the start of the move (s)')
    legend(names, 'Location', 'southwest')
end
linkaxes(bx(1,:), 'x')
linkaxes(bx(2,:), 'x')

%% 7) Figure 3: voltages
figure('Name', 'Decoupled PD tests: voltages', 'Color', 'w');
cx = gobjects(2, nT);
for i = 1:nT
    cx(1,i) = subplot(2, nT, i);       hold on;  grid on
    plot(T(i).t, T(i).Vp, 'Color', colors(1,:))
    title(testLabels{i})
    ylabel('V_p (V)')
    ylim([-satV-2, satV+2])
    cx(2,i) = subplot(2, nT, nT + i);  hold on;  grid on
    plot(T(i).t, T(i).Vy, 'Color', colors(2,:))
    ylabel('V_y (V)')
    xlabel('Time (s)')
    ylim([-satV-2, satV+2])
end
linkaxes(cx(:), 'x')
xlim(cx(1,1), [T(1).t(1) T(1).t(end)])

%% 8) Figure 4: pitch torque efficiency against the holding voltage
figure('Name', 'Decoupled PD tests: pitch torque efficiency', 'Color', 'w');
hold on;  grid on
for i = 1:nT
    sel = effTest == i;
    errorbar(effV(sel), effR(sel), effE(sel), 'o', 'Color', colors(i,:), 'LineWidth', 1.2, 'MarkerFaceColor', colors(i,:))
end
V = linspace(3, 11, 50);
plot(V, V/8, 'k--')                  % thrust ~ V|V|/8: torque per volt grows like |V|/8
yline(1, 'k:');
xlabel('|V_p| on the plateau (V)')
ylabel('real / commanded pitch torque')
title('Pitch torque efficiency on the plateaus')
legend([testLabels, {'V|V|/8 thrust'}], 'Location', 'northwest')

%% 9) Figure 5: the tests overlaid, move by move
% Top row: yaw motion during each pitch move. Bottom row: pitch motion
% during each yaw move. One line per test (same references in every test).
figure('Name', 'Decoupled PD tests: coupling, tests overlaid', 'Color', 'w');
testColors = lines(nT);
[kSP, ~] = findMoves(T(nT).refP);
[kSY, ~] = findMoves(T(nT).refY);
nMoves = max(numel(kSP), numel(kSY));
dx = gobjects(2, nMoves);
for k = 1:nMoves
    dx(1,k) = subplot(2, nMoves, k);           hold on;  grid on
    dx(2,k) = subplot(2, nMoves, nMoves + k);  hold on;  grid on
    for i = 1:nT
        t = T(i).t;  Ts = t(2) - t(1);
        [kS, kE] = findMoves(T(i).refP);
        if k <= numel(kS)
            idx = couplingIndices(kS, kE, k, findMoves(T(i).refY), length(t), Ts, couplingWindow);
            plot(dx(1,k), t(idx) - t(kS(k)), T(i).yaw(idx) - T(i).yaw(kS(k)), 'Color', testColors(i,:), 'LineWidth', 1.5)
            title(dx(1,k), sprintf('Yaw during pitch move %d (t = %.0f s)', k, t(kS(k))))
        end
        [kS, kE] = findMoves(T(i).refY);
        if k <= numel(kS)
            idx = couplingIndices(kS, kE, k, findMoves(T(i).refP), length(t), Ts, couplingWindow);
            plot(dx(2,k), t(idx) - t(kS(k)), T(i).pitch(idx) - T(i).pitch(kS(k)), 'Color', testColors(i,:), 'LineWidth', 1.5)
            title(dx(2,k), sprintf('Pitch during yaw move %d (t = %.0f s)', k, t(kS(k))))
        end
    end
    yline(dx(1,k), 0, 'k:');
    yline(dx(2,k), 0, 'k:');
    xlabel(dx(2,k), 'Time since the start of the move (s)')
end
ylabel(dx(1,1), '\Delta\psi (deg)')
ylabel(dx(2,1), '\Delta\theta (deg)')
legend(dx(1,1), testLabels, 'Location', 'best')

set(findall(0, 'Type', 'axes'), 'FontSize', 12)

%% ======================= LOCAL FUNCTIONS =======================

function v = filteredDerivative(y, t, wf, zf)
% Filtered derivative of y, same as the 2nd-order filter of the PIV block:
%   V(s)/Y(s) = wf^2 s / (s^2 + 2 zf wf s + wf^2)   (zero-order hold, step Ts)
    Ts   = t(2) - t(1);
    filt = c2d(ss([0 1; -wf^2 -2*zf*wf], [0; wf^2], [0 1], 0), Ts);
    v    = lsim(filt, y, t);
end

function [kStart, kEnd] = findMoves(ref)
% Start and end index of each move (run of samples where ref changes).
    moving = [false; abs(diff(ref)) > 1e-9];
    kStart = find(diff([false; moving]) == 1);
    kEnd   = find(diff([moving; false]) == -1);
end

function idx = couplingIndices(kS, kE, k, kOther, N, Ts, Tmax)
% Samples from the start of move k until the next move of the same axis,
% the next move of the other axis, or Tmax seconds, whichever comes first.
    k0 = kS(k);
    k1 = min(k0 + round(Tmax/Ts), N);
    if k < numel(kS), k1 = min(k1, kS(k+1) - 1); end
    nextO = kOther(kOther > k0);
    if ~isempty(nextO), k1 = min(k1, nextO(1) - 1); end
    idx = (k0:k1)';
end

function m = loopMetrics(t, refP, refY, Y, U, satV)
% Same indicators as decoupled_pd_AERO.m (angles in deg).
    Ts = t(2) - t(1);
    [m.essP, m.osP]  = stepIndicators(t, refP, Y(:,1));
    [m.essY, m.osY]  = stepIndicators(t, refY, Y(:,2));
    m.rmseP = sqrt(mean((refP - Y(:,1)).^2));
    m.rmseY = sqrt(mean((refY - Y(:,2)).^2));
    m.coupP = coupling(refY, refP, Y(:,1), Ts);
    m.coupY = coupling(refP, refY, Y(:,2), Ts);
    m.sat   = 100*mean(abs(U(:)) >= 0.99*satV);
    m.act   = sum(sum(abs(diff(U)))) / (t(end) - t(1));
end

function [ess, os] = stepIndicators(t, ref, y)
    Ts = t(2) - t(1);
    [kS, ~] = findMoves(ref);
    n = numel(kS);
    es = nan(n,1);  oss = nan(n,1);
    for i = 1:n
        k0 = kS(i);
        if i < n, k1 = kS(i+1) - 1; else, k1 = length(t); end
        r0  = ref(k0 - 1);
        r1  = ref(k1);
        z   = (y(k0:k1) - r0) / (r1 - r0);
        oss(i) = max(0, 100*(max(z) - 1));
        last  = max(k0, k1 - round(1/Ts) + 1):k1;
        es(i) = abs(r1 - mean(y(last)));
    end
    ess = mean(es);
    os  = mean(oss);
end

function c = coupling(refMove, refOther, yOther, Ts)
% Largest motion of the OTHER axis during each move of refMove (ramp + 3 s),
% counted only when refOther stays constant.
    [kS, kE] = findMoves(refMove);
    [oS, ~]  = findMoves(refOther);
    otherMoving = false(size(refOther));
    otherMoving([false; abs(diff(refOther)) > 1e-9]) = true;
    c = 0;
    for i = 1:numel(kS)
        k0 = kS(i);
        k1 = min(kE(i) + round(3/Ts), length(yOther));
        if i < numel(kS), k1 = min(k1, kS(i+1) - 1); end
        nextO = oS(oS > k0);
        if ~isempty(nextO), k1 = min(k1, nextO(1) - 1); end
        if any(otherMoving(k0:k1)), continue; end
        c = max(c, max(abs(yOther(k0:k1) - yOther(k0))));
    end
end