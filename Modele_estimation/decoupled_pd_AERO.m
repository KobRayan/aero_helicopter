% decoupled_pd_AERO.m
% 14/10/2026
% DECOUPLED PD CONTROLLER for the Quanser AERO (2-DOF helicopter).
%
% WHY A DECOUPLER
% ---------------
% Model (Quanser structure, identified with CONTSID):
%   Jp*theta'' + Dp*theta' + Ksp*theta = tau_p
%   Jy*psi''   + Dy*psi'               = tau_y
%   [tau_p; tau_y] = Bt*[Vp; Vy],   Bt = [Kpp Kpy; Kyp Kyy]
% The two "bodies" (pitch, yaw) are NOT coupled; ONLY the torque gains Bt
% couple the axes (each motor creates a torque on both axes). So
%   G(s) = diag(Pp(s), Py(s)) * Bt,  Pp = 1/(Jp s^2+Dp s+Ksp),  Py = 1/(s(Jy s+Dy))
% and the cross-couplings G12 = Kpy*Pp and G21 = Kyp*Py have the SAME
% dynamics as the direct paths: a STATIC decoupler is enough.
% Choose  V = W*tau_c  with  W = inv(Bt):
%   G(s)*W = diag(Pp, Py)  -> G12 and G21 disappear (in the model).
% Each axis is then a simple 1-input/1-output body driven by "its" torque
% tau_c, and we put one PD on each, designed with the Quanser formulas
% (quanser_aero_pd_design_Ksp.m), but in TORQUE units:
%   pitch: tau_p = kp_p*(r_p - theta) - kd_p*theta_dot + Ksp*r_p
%          kp_p = Jp*wn^2 - Ksp,  kd_p = 2*zeta*wn*Jp - Dp
%          (+ Ksp*r_p = holding torque of the spring: no steady-state error)
%   yaw  : tau_y = kp_y*(r_y - psi) - kd_y*psi_dot
%          kp_y = Jy*wn^2,        kd_y = 2*zeta*wn*Jy - Dy
% Closed loop of each axis: s^2 + 2*zeta*wn*s + wn^2 (exactly the spec).
% The derivative acts on the measurement (PV form): no kick at a step.
%
% SAME DIAGRAM AS THE LQR
% -----------------------
% The whole controller is a state feedback + a reference gain:
%   V = Keq*(xd - x) + Nff*r,   xd = [r_p; r_y; 0; 0],  r = [r_p; r_y]
%   Keq = W*[kp_p 0 kd_p 0; 0 kp_y 0 kd_y],   Nff = W*[Ksp 0; 0 0]
% -> in the LQR Simulink model, replace K by Keq and add the gain Nff on
%    the reference (the "pre-control" Nu). Printed in section 2.
%
% SATURATION THAT KEEPS THE DECOUPLING ("pitch priority")
% -------------------------------------------------------
% If one voltage hits +/-20 V, the torques are no longer the commanded
% ones and the decoupling is lost (the yaw then drags the pitch). Option
% pitchPrio: the pitch torque is kept exact and only the yaw torque is
% reduced until both voltages fit in +/-satV (function allocateVoltages,
% to copy into a MATLAB Function block in Simulink).
%
% WHAT THE SCRIPT DOES
% --------------------
%  1) loads the identified model, prints Bt, W = inv(Bt), RGA, cond(Bt)
%  2) computes the gains of each design, Keq, Nff, closed-loop poles
%  3) builds the references (staggered ramps: only one axis moves at a
%     time, so the coupling can be measured), or reads them from a test
%  4) simulates each design on the chosen plant (linear, or "realistic":
%     dry friction + motor delay + encoder quantization), Ts = 2 ms,
%     +/-20 V, velocities from the QUARC 2nd-order filter (as in State X)
%  5) prints the indicators (steady-state error, overshoot, RMSE,
%     coupling, saturation, voltage activity)
%  6) robustness: residual coupling when the plant cross gains Kpy, Kyp
%     differ from the ones used in the decoupler
%  7) figures, and saves the gains in results_decoupled_pd.mat
%
% Files needed in the current folder:
%   quanser_aero_parameters_identified.mat  (identify_quanser_AERO.m)
%   aero_friction_parameters.mat            (fit_friction_AERO.m, realistic plant only)
% Control System Toolbox (c2d, ss).

clear;
close all;
clc

%% 0) Settings
addpath('../LQR')
addpath('../CONTSID7_6')
addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'reponse_BO_experimentation'))   % identified parameters (.mat)
identFile = 'quanser_aero_parameters_identified.mat';  % linear (CONTSID) model: used for the DESIGN
fricFile  = 'aero_friction_parameters.mat';            % friction model: used for the "realistic" plant
measFile  = '';          % optional measured PD test (9-row log): '' = use the references built below
iMeas     = 3;           % design that was run on the AERO (compared with measFile)

plantMode = 'realistic'; % 'linear'    : identified linear model (design model)
                         % 'realistic' : friction model + delays + encoder quantization
useV2     = false;       % realistic plant only: thrust ~ V|V|/8 instead of V (V^2 thrust)

satV  = 20;              % voltage saturation (V)
Ts    = 0.002;           % sample time (s)
d2r   = pi/180;

% References built by the script (if measFile is empty), in deg.
% One axis moves at a time, so the coupling can be measured.
rateLim  = 20;           % Rate Limiter (deg/s), Inf = steps
Tend     = 38;           % duration (s)
pitchMoves = [ 1  20;    % [time (s), new pitch target (deg)]
              13 -20;
              26   0];
yawMoves   = [ 6  40;    % [time (s), new yaw target (deg)]
              18 -40;
              31   0];

% Velocity estimation used in the AERO model (State X block):
% QUARC "Second-Order Low-Pass Filter", wn = 50 rad/s, zeta = 0.8 (its 2nd output = filtered derivative)
wf = 50;
zf = 0.8;
% Realistic plant: pure delays of the motors + propellers (s), from fit_friction_AERO.m
TdP = 0.05;
TdY = 0.10;

% PD designs to compare, one per row:
%  name | decouple (W = inv(Bt)) | PO_p (%) | tp_p (s) | PO_y (%) | tp_y (s) | spring feedforward | pitch priority
% PO = 7.5 %, tp = 1.75 s are the Quanser specifications (quanser_aero_pd_design.m)
designs = {
    'Independent PD (Quanser)', false, 7.5, 1.75, 7.5, 1.75, false, false
    'Decoupled PD',             true,  7.5, 1.75, 7.5, 1.75, true,  false
    'Decoupled PD + priority',  true,  7.5, 1.75, 7.5, 1.75, true,  true
    'Decoupled PD faster',      true,  7.5, 1.50, 7.5, 1.50, true,  true
    };
% Reference controller: the LQR gain used on the AERO ("Current", designed on the Quanser model)
compareLQR = true;
K_LQR = [ 98.21 -103.06 32.26 -29.08 ;
         156.35   66.16 45.51  17.11 ];

doRobustness = true;     % section 6 (a few more simulations)

% Tuning knobs of the decoupler ON THE AERO (1 = identified values).
% The decoupler uses cKpy*Kpy and cKyp*Kyp. See the report for how to read
% the sign of the residual coupling and correct these two factors.
cKpy = 1;
cKyp = 1;

%% 1) Identified model and coupling analysis
P = load(identFile);     % Jp Jy Ksp Dp Dy Kpp Kpy Kyp Kyy (+ A B C D)
Bt = [P.Kpp P.Kpy;
      P.Kyp P.Kyy];      % torques = Bt * voltages  (N.m/V)
Wdec = [P.Kpp cKpy*P.Kpy;
        cKyp*P.Kyp P.Kyy];
W  = inv(Wdec);          % decoupler: voltages = W * torques (V/(N.m))
RGA = Bt .* inv(Bt)';    % relative gain array (same at ALL frequencies here)

A = [0 0 1 0;
     0 0 0 1;
    -P.Ksp/P.Jp 0 -P.Dp/P.Jp 0;
     0 0 0 -P.Dy/P.Jy];
B = [0 0;
     0 0;
     P.Kpp/P.Jp P.Kpy/P.Jp;
     P.Kyp/P.Jy P.Kyy/P.Jy];

fprintf('===== 1) COUPLING OF THE IDENTIFIED MODEL =====\n');
fprintf('Bt = [Kpp Kpy; Kyp Kyy] (N.m/V) =\n');  disp(Bt)
fprintf('W = inv(Bt) (V per N.m) =\n');           disp(W)
fprintf('  pure pitch torque: Vp = %+.0f, Vy = %+.0f V per N.m\n', W(1,1), W(2,1));
fprintf('  pure yaw torque  : Vp = %+.0f, Vy = %+.0f V per N.m\n', W(1,2), W(2,2));
fprintf('cond(Bt) = %.2f (close to 1: inverting Bt is safe)\n', cond(Bt));
fprintf('RGA = [%.2f %.2f; %.2f %.2f]  (lambda11 < 0.5: strong interaction,\n', RGA');
fprintf('      an independent PD per axis sees its gain change by 1/lambda11 = %.1f when the other loop is closed)\n\n', 1/RGA(1,1));

%% 2) Gains of each design
nD = size(designs,1);
D  = struct([]);                % one entry per controller (PD designs + LQR)
fprintf('===== 2) GAINS =====\n');
for d = 1:nD
    [zp, wp] = specToZetaWn(designs{d,3}, designs{d,4});
    [zy, wy] = specToZetaWn(designs{d,5}, designs{d,6});
    kp_p = P.Jp*wp^2 - P.Ksp;   kd_p = 2*zp*wp*P.Jp - P.Dp;   % torque gains (N.m/rad, N.m.s/rad)
    kp_y = P.Jy*wy^2;           kd_y = 2*zy*wy*P.Jy - P.Dy;
    Kt = [kp_p 0 kd_p 0;
          0 kp_y 0 kd_y];       % torque = Kt*(xd - x)
    if designs{d,2}
        Wd = W;                               % decoupler
    else
        Wd = diag([1/P.Kpp, 1/P.Kyy]);        % independent PD: each motor for "its" axis only
    end
    Ft = [P.Ksp*designs{d,7} 0; 0 0];         % spring feedforward (torque per rad of reference)

    D(d).name = designs{d,1};
    D(d).Kt   = Kt;
    D(d).W    = Wd;
    D(d).Ft   = Ft;
    D(d).prio = designs{d,8};
    D(d).Keq  = Wd*Kt;                        % V = Keq*(xd - x) + Nff*r
    D(d).Nff  = Wd*Ft;
    p = eig(A - B*D(d).Keq);                  % closed-loop poles on the identified model

    fprintf('--- %s ---\n', D(d).name);
    fprintf('  spec pitch: zeta %.3f, wn %.2f rad/s | yaw: zeta %.3f, wn %.2f rad/s\n', zp, wp, zy, wy);
    fprintf('  torque gains: kp_p %.4f N.m/rad, kd_p %.4f N.m.s/rad, kp_y %.4f, kd_y %.4f\n', kp_p, kd_p, kp_y, kd_y);
    fprintf('  Keq (V/rad, V.s/rad), to put in place of K in the LQR model =\n');  disp(D(d).Keq)
    fprintf('  Nff (V/rad, on [r_p; r_y]) =\n');  disp(D(d).Nff)
    fprintf('  closed-loop poles: %s\n', mat2str(round(p.', 3)));
    fprintf('  max velocity gain %.1f V.s/rad (derivative noise grows with it)\n\n', max(max(abs(D(d).Keq(:,3:4)))));
end
if compareLQR
    d = nD + 1;
    D(d).name = 'LQR Current';
    D(d).Kt   = K_LQR;          % already in volts
    D(d).W    = eye(2);
    D(d).Ft   = zeros(2);
    D(d).prio = false;
    D(d).Keq  = K_LQR;
    D(d).Nff  = zeros(2);
    fprintf('--- LQR Current (reference) --- closed-loop poles on the identified model: %s\n\n', ...
            mat2str(round(eig(A - B*K_LQR).', 3)));
end
nC = numel(D);
colors = lines(nC);

%% 3) References
if isempty(measFile)
    t = (0:Ts:Tend)';
    refP = makeReference(t, pitchMoves, rateLim);     % deg
    refY = makeReference(t, yawMoves,   rateLim);
    measured = false;
else
    S    = load(measFile);
    fn   = fieldnames(S);
    data = S.(fn{1});
    if size(data,1) > size(data,2)
        data = data';
    end
    t          = data(1,:)';
    PitchVolt  = data(2,:)';
    refP       = data(3,:)';
    PitchAngle = data(4,:)';
    YawVolt    = data(5,:)';
    refY       = data(6,:)';
    YawAngle   = data(7,:)';
    Ts = t(2) - t(1);
    measured = true;
end

%% 4) Plant and simulation
plant = struct('Jp', P.Jp, 'Jy', P.Jy, 'Ksp', P.Ksp, 'Dp', P.Dp, 'Dy', P.Dy, 'Bt', Bt, ...
               'FcP', 0, 'FcY', 0, 'TdP', 0, 'TdY', 0, 'quant', false, 'V2', false);
if strcmp(plantMode, 'realistic')
    Fr = load(fricFile);
    plant.Ksp = Fr.fricPitch.Ksp;   plant.Dp = Fr.fricPitch.Dp;   plant.FcP = Fr.fricPitch.Fc;
    plant.Dy  = Fr.fricYaw.Dy;      plant.FcY = Fr.fricYaw.Fc;
    plant.Bt  = [Fr.fricPitch.Kpp P.Kpy;      % direct gains from the friction fit,
                 P.Kyp Fr.fricYaw.Kyy];       % cross gains from CONTSID
    plant.TdP = TdP;   plant.TdY = TdY;
    plant.quant = true;
    plant.V2 = useV2;
end
fprintf('===== 4) SIMULATION on the %s plant%s =====\n', plantMode, repmat(' (V^2 thrust)', 1, double(plant.V2)));
fprintf('plant Bt * W (identity = perfect decoupling) =\n');  disp(plant.Bt*W)

% Velocity filter of State X, discretised (zero-order hold)
filt = c2d(ss([0 1; -wf^2 -2*zf*wf], [0; wf^2], eye(2), zeros(2,1)), Ts);

Ysim = cell(nC,1);   Usim = cell(nC,1);   M = cell(nC,1);
for d = 1:nC
    [Ysim{d}, Usim{d}] = simulateLoop(D(d), plant, t, refP*d2r, refY*d2r, filt.A, filt.B, satV);
    M{d} = loopMetrics(t, refP, refY, Ysim{d}, Usim{d}, satV);
end

%% 5) Indicators
fprintf('%-26s | %5s %5s | %5s %5s | %5s %5s | %6s %6s | %5s | %6s\n', 'Controller', ...
        'e_p', 'e_y', 'OS_p', 'OS_y', 'RMS_p', 'RMS_y', 'cpl_p', 'cpl_y', 'sat', 'activ');
fprintf('%-26s | %5s %5s | %5s %5s | %5s %5s | %6s %6s | %5s | %6s\n', '', ...
        '(deg)', '(deg)', '(%)', '(%)', '(deg)', '(deg)', '(deg)', '(deg)', '(%)', '(V/s)');
for d = 1:nC
    m = M{d};
    fprintf('%-26s | %5.2f %5.2f | %5.1f %5.1f | %5.2f %5.2f | %6.2f %6.2f | %5.1f | %6.1f\n', D(d).name, ...
            m.essP, m.essY, m.osP, m.osY, m.rmseP, m.rmseY, m.coupP, m.coupY, m.sat, m.act);
end
fprintf(['e = steady-state error (last second before the next move), OS = overshoot,\n' ...
         'cpl_p = largest pitch motion during a yaw move, cpl_y = largest yaw motion during a pitch move,\n' ...
         'activ = sum|dV|/T of both motors (encoder quantization -> derivative noise).\n\n']);
if measured
    fitP = 100*(1 - norm(PitchAngle - Ysim{iMeas}(:,1)) / norm(PitchAngle - mean(PitchAngle)));
    fitY = 100*(1 - norm(YawAngle   - Ysim{iMeas}(:,2)) / norm(YawAngle   - mean(YawAngle)));
    Mm = loopMetrics(t, refP, refY, [PitchAngle YawAngle], [PitchVolt YawVolt], satV);
    fprintf('%-26s | %5.2f %5.2f | %5.1f %5.1f | %5.2f %5.2f | %6.2f %6.2f | %5.1f | %6.1f\n', 'MEASURED', ...
            Mm.essP, Mm.essY, Mm.osP, Mm.osY, Mm.rmseP, Mm.rmseY, Mm.coupP, Mm.coupY, Mm.sat, Mm.act);
    fprintf('Fit of "%s" simulation vs measurement: pitch %.1f %%, yaw %.1f %%\n\n', D(iMeas).name, fitP, fitY);
end

%% 6) Robustness: cross gains of the plant different from the decoupler
% The decoupler uses Kpy and Kyp, the least reliable parameters (G12 fit ~60 %).
% We multiply the PLANT cross gains and look at the coupling left.
if doRobustness
    scales = [1 1; 0.5 1; 1.5 1; 1 0.5; 1 1.5; 0.5 0.5; 1.5 1.5];   % [x Kpy, x Kyp]
    iRob = [3, nC];      % row 3 of designs (decoupled PD + priority) and the last controller (LQR if compareLQR)
    fprintf('===== 6) ROBUSTNESS: plant cross gains scaled, coupling cpl_p / cpl_y (deg) =====\n');
    fprintf('%-12s', 'Kpy  Kyp');
    for d = iRob, fprintf(' | %-24s', D(d).name); end
    fprintf('\n');
    for s = 1:size(scales,1)
        pl = plant;
        pl.Bt(1,2) = plant.Bt(1,2)*scales(s,1);
        pl.Bt(2,1) = plant.Bt(2,1)*scales(s,2);
        fprintf('x%-4.1f x%-4.1f', scales(s,1), scales(s,2));
        for d = iRob
            [Yr, Ur] = simulateLoop(D(d), pl, t, refP*d2r, refY*d2r, filt.A, filt.B, satV);
            mr = loopMetrics(t, refP, refY, Yr, Ur, satV);
            fprintf(' | %6.2f / %6.2f          ', mr.coupP, mr.coupY);
        end
        fprintf('\n');
    end
    fprintf('\n');
end

%% 7) Figures
% Figure 1: angles
figure('Name', 'Decoupled PD: angles', 'Color', 'w');
ax1 = subplot(2,1,1);  hold on;  grid on
ax2 = subplot(2,1,2);  hold on;  grid on
plot(ax1, t, refP, 'k--', 'LineWidth', 1)
plot(ax2, t, refY, 'k--', 'LineWidth', 1)
names = {'Reference'};
if measured
    plot(ax1, t, PitchAngle, 'k', 'LineWidth', 2)
    plot(ax2, t, YawAngle,   'k', 'LineWidth', 2)
    names{end+1} = 'Measured';
end
for d = 1:nC
    plot(ax1, t, Ysim{d}(:,1), 'Color', colors(d,:), 'LineWidth', 1.2)
    plot(ax2, t, Ysim{d}(:,2), 'Color', colors(d,:), 'LineWidth', 1.2)
end
names = [names, {D.name}];
title(ax1, ['Pitch (simulation, ' plantMode ' plant)'])
ylabel(ax1, '\theta (deg)')
title(ax2, 'Yaw')
ylabel(ax2, '\psi (deg)')
xlabel(ax2, 'Time (s)')
legend(ax1, names, 'Location', 'eastoutside')
legend(ax2, names, 'Location', 'eastoutside')
linkaxes([ax1 ax2], 'x')
xlim(ax1, [t(1) t(end)])

% Figure 2: tracking errors (the coupling shows as bumps when the OTHER axis moves)
figure('Name', 'Decoupled PD: tracking errors', 'Color', 'w');
cx1 = subplot(2,1,1);  hold on;  grid on
cx2 = subplot(2,1,2);  hold on;  grid on
for d = 1:nC
    plot(cx1, t, refP - Ysim{d}(:,1), 'Color', colors(d,:), 'LineWidth', 1.2)
    plot(cx2, t, refY - Ysim{d}(:,2), 'Color', colors(d,:), 'LineWidth', 1.2)
end
for k = find(diff([0; abs(diff(refY)) > 1e-9]) == 1)'     % yaw moves -> dotted lines on the pitch plot
    xline(cx1, t(k), 'k:');
end
for k = find(diff([0; abs(diff(refP)) > 1e-9]) == 1)'     % pitch moves -> dotted lines on the yaw plot
    xline(cx2, t(k), 'k:');
end
title(cx1, 'Pitch error r_p - \theta  (dotted: start of a YAW move)')
ylabel(cx1, 'e_\theta (deg)')
title(cx2, 'Yaw error r_y - \psi  (dotted: start of a PITCH move)')
ylabel(cx2, 'e_\psi (deg)')
xlabel(cx2, 'Time (s)')
legend(cx1, {D.name}, 'Location', 'eastoutside')
legend(cx2, {D.name}, 'Location', 'eastoutside')
linkaxes([cx1 cx2], 'x')
xlim(cx1, [t(1) t(end)])

% Figure 3: voltages
figure('Name', 'Decoupled PD: voltages', 'Color', 'w');
bx1 = subplot(2,1,1);  hold on;  grid on
bx2 = subplot(2,1,2);  hold on;  grid on
if measured
    plot(bx1, t, PitchVolt, 'Color', [0.6 0.6 0.6])
    plot(bx2, t, YawVolt,   'Color', [0.6 0.6 0.6])
end
for d = 1:nC
    plot(bx1, t, Usim{d}(:,1), 'Color', colors(d,:), 'LineWidth', 1.2)
    plot(bx2, t, Usim{d}(:,2), 'Color', colors(d,:), 'LineWidth', 1.2)
end
title(bx1, 'Pitch motor voltage')
ylabel(bx1, 'V_p (V)')
title(bx2, 'Yaw motor voltage')
ylabel(bx2, 'V_y (V)')
xlabel(bx2, 'Time (s)')
legend(bx1, names(2:end), 'Location', 'eastoutside')
legend(bx2, names(2:end), 'Location', 'eastoutside')
ylim(bx1, [-satV-2, satV+2])
ylim(bx2, [-satV-2, satV+2])
linkaxes([bx1 bx2], 'x')
xlim(bx1, [t(1) t(end)])

% Figure 4: indicators side by side
Mall = [M{:}];
figure('Name', 'Decoupled PD: indicators', 'Color', 'w');
dx1 = subplot(2,2,1);  bar(categorical({D.name}, {D.name}), [[Mall.coupP]' [Mall.coupY]']);  grid on
title(dx1, 'Coupling (deg)');  legend(dx1, 'pitch moved by yaw', 'yaw moved by pitch')
dx2 = subplot(2,2,2);  bar(categorical({D.name}, {D.name}), [[Mall.rmseP]' [Mall.rmseY]']);  grid on
title(dx2, 'Tracking RMSE (deg)');  legend(dx2, 'pitch', 'yaw')
dx3 = subplot(2,2,3);  bar(categorical({D.name}, {D.name}), [[Mall.essP]' [Mall.essY]']);  grid on
title(dx3, 'Steady-state error (deg)');  legend(dx3, 'pitch', 'yaw')
dx4 = subplot(2,2,4);  bar(categorical({D.name}, {D.name}), [Mall.act]');  grid on
title(dx4, 'Voltage activity sum|dV|/T (V/s)')

set(findall(0, 'Type', 'axes'), 'FontSize', 12)

%% 8) Save
designNames = {D.name};
KeqAll = {D.Keq};          % V = Keq*(xd - x) + Nff*r            (plain saturation)
NffAll = {D.Nff};
KtAll  = {D.Kt};           % tau = Kt*(xd - x) + Ft*r, V = allocateVoltages(tau, W, satV, prio)
FtAll  = {D.Ft};
save('results_decoupled_pd.mat', 'designNames', 'KeqAll', 'NffAll', 'KtAll', 'FtAll', ...
     'W', 'Bt', 'RGA', 'Mall', 'plantMode');
fprintf('Saved: results_decoupled_pd.mat\n');

%% ======================= LOCAL FUNCTIONS =======================

function [zeta, wn] = specToZetaWn(PO, tp)
% Damping ratio and natural frequency from the overshoot PO (%) and the
% peak time tp (s) of a 2nd-order system (same formulas as Quanser).
    zeta = -log(PO/100) / sqrt(log(PO/100)^2 + pi^2);
    wn   = pi / (tp*sqrt(1 - zeta^2));
end

function V = allocateVoltages(tau, W, satV, prio)
% Torques -> voltages, with the +/-satV saturation.
%   V = W*tau. If prio is true and a voltage would saturate, the PITCH
%   torque tau(1) is kept and the yaw torque tau(2) is reduced until both
%   voltages fit in +/-satV, so the decoupling is kept (W stays exact).
% Can be copied as it is into a MATLAB Function block in Simulink.
    if prio
        lo = -inf;  hi = inf;               % allowed interval for tau(2)
        for i = 1:2                         % |W(i,1)*tau1 + W(i,2)*tau2| <= satV
            if W(i,2) == 0, continue; end   % this motor does not depend on tau(2)
            a  = W(i,1)*tau(1);
            l  = (-satV - a) / W(i,2);
            h  = ( satV - a) / W(i,2);
            if W(i,2) < 0                   % dividing by a negative number swaps the bounds
                tmp = l;  l = h;  h = tmp;
            end
            lo = max(lo, l);
            hi = min(hi, h);
        end
        if lo <= hi                         % pitch torque alone is feasible
            tau(2) = min(max(tau(2), lo), hi);
        end
    end
    V = W*tau;
    V = min(max(V, -satV), satV);           % final safety saturation
end

function [Y, U] = simulateLoop(C, plant, t, refP, refY, Af, Bf, satV)
% Closed-loop simulation, controller at Ts = t(2)-t(1), plant integrated
% with 4 sub-steps per sample (semi-implicit Euler, Karnopp dry friction).
%   controller: xh  = [measured angles; filtered derivatives]
%               tau = C.Kt*(xd - xh) + C.Ft*r     (torques; volts for the LQR)
%               V   = allocateVoltages(tau, C.W, satV, C.prio)
%   plant     : torque = Bt * delayed V   (or Bt * V|V|/8 if plant.V2)
% Returns the angles (deg) and the commanded voltages (V).
    Ts   = t(2) - t(1);
    nSub = 4;
    h    = Ts/nSub;
    N    = length(t);
    nP   = round(plant.TdP/Ts);
    nY   = round(plant.TdY/Ts);
    qP   = 2*pi/2048;                       % one encoder count (rad)
    qY   = 2*pi/4096;
    th = 0; ps = 0; thd = 0; psd = 0;       % plant state
    zf = zeros(2,2);                        % filter states [pitch yaw] x [angle; derivative]
    Y  = zeros(N,2);
    U  = zeros(N,2);
    for k = 1:N
        ym = [th; ps];
        if plant.quant
            ym = [round(th/qP)*qP; round(ps/qY)*qY];
        end
        xh  = [ym; zf(2,1); zf(2,2)];       % velocities from the State X filter
        zf  = Af*zf + Bf*ym';               % filter update (one column per axis)
        r   = [refP(k); refY(k)];
        tau = C.Kt*([r; 0; 0] - xh) + C.Ft*r;
        V   = allocateVoltages(tau, C.W, satV, C.prio);
        U(k,:) = V';
        Y(k,:) = [th ps]*180/pi;
        % voltages reaching the motors (pure delay)
        Vd = [0; 0];
        if k - nP >= 1, Vd(1) = U(k-nP,1); end
        if k - nY >= 1, Vd(2) = U(k-nY,2); end
        if plant.V2
            Vd = Vd.*abs(Vd)/8;             % thrust ~ V^2, equal to the linear model at 8 V
        end
        T = plant.Bt*Vd;                    % torques (N.m)
        for s = 1:nSub
            [th, thd] = axisStep(th, thd, T(1), plant.Jp, plant.Ksp, plant.Dp, plant.FcP, h);
            [ps, psd] = axisStep(ps, psd, T(2), plant.Jy, 0,         plant.Dy, plant.FcY, h);
        end
    end
end

function [x, v] = axisStep(x, v, T, J, Ks, Dv, Fc, h)
% One integration step of  J*x'' = T - Ks*x - Dv*x' - Fc*sign(x')
% with static friction (same rules as simFriction in fit_friction_AERO.m).
    Td = T - Ks*x;
    if v == 0
        if abs(Td) <= Fc
            a = 0;                          % stuck
        else
            a = (Td - Fc*sign(Td)) / J;     % breaks free
        end
    else
        a = (Td - Dv*v - Fc*sign(v)) / J;
    end
    vNew = v + a*h;
    if v ~= 0 && sign(vNew) ~= sign(v)
        vNew = 0;                           % speed crosses zero -> stop
    end
    v = vNew;
    x = x + v*h;
end

function ref = makeReference(t, moves, rate)
% Piecewise-constant targets [time, value] followed by a rate limiter (deg/s).
    Ts  = t(2) - t(1);
    tgt = zeros(size(t));
    for i = 1:size(moves,1)
        tgt(t >= moves(i,1) - 1e-9) = moves(i,2);
    end
    ref = zeros(size(t));
    for k = 2:length(t)
        ref(k) = ref(k-1) + min(max(tgt(k) - ref(k-1), -rate*Ts), rate*Ts);
    end
end

function m = loopMetrics(t, refP, refY, Y, U, satV)
% Indicators of one run (angles in deg). A "move" = a change of a reference
% (a whole ramp counts as one move).
%   essP/essY  : mean |error| over the last second before the next move
%   osP/osY    : mean overshoot (%) of the moves
%   rmseP/rmseY: tracking RMSE over the whole run
%   coupP      : largest pitch motion during a yaw move (pitch reference
%                constant), window = yaw ramp + 3 s; coupY the reverse
%   sat        : % of samples with a saturated voltage
%   act        : sum|dV|/T of both motors (V/s)
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

function [kStart, kEnd] = findMoves(ref)
% Start and end index of each move (run of samples where ref changes).
    moving = [false; abs(diff(ref)) > 1e-9];
    kStart = find(diff([false; moving]) == 1);
    kEnd   = find(diff([moving; false]) == -1);
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
% Largest motion of the OTHER axis during each move of refMove, counted
% only when refOther stays constant (window = ramp + 3 s).
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
