%% fit_friction_AERO.m
% =========================================================================
%  DRY (COULOMB) FRICTION ADDED TO THE QUANSER MODEL OF THE AERO
% =========================================================================
%
% WHY THIS SCRIPT
% ---------------
% The Quanser model (Lab Guide eq. 2.1-2.2) only has VISCOUS friction, a
% torque proportional to the speed (Dp*theta_dot, Dy*psi_dot). With viscous
% friction only, an oscillation decays EXPONENTIALLY (each swing is a fixed
% fraction of the previous one) and never really stops.
% On the real AERO we see:
%   - pitch: one or two big swings, then the motion stops quickly, and the
%     pitch can stay "stuck" at about +/-1.5 deg from where it should be
%     (staircase test: +1.4 / -1.8 deg at 0 V);
%   - yaw  : no motion below ~6-7 V (dead zone), and when the voltage goes
%     back to 0 the yaw stops in ~2 s at a constant deceleration.
% This is DRY (Coulomb) friction: a torque of CONSTANT size that always
% opposes the motion, and that can hold the axis still (static friction /
% stiction) as long as the other torques are smaller than it.
% A linear model can only imitate it with a large viscous damping, which is
% then right for one amplitude only (describing-function result: the
% equivalent viscous damping of Coulomb friction is 4*Fc/(pi*w*A), larger
% when the amplitude A is smaller). That is why the identified damping
% depended on the test (zeta 0.10 at 10 V, 0.16-0.21 at 8 V; yaw time
% constant 2 s, 4.9 s or 14 s depending on the test).
%
% WHAT THIS SCRIPT DOES
% ---------------------
% It keeps the Quanser equations and adds ONE term per axis:
%
%   Jp*theta'' + Dp*theta' + Fc_p*sign(theta') + Ksp*theta = Kpp*Vp(t-Td_p)
%   Jy*psi''   + Dy*psi'   + Fc_y*sign(psi')               = Kyy*Vy(t-Td_y)
%
% plus static friction: an axis at rest stays at rest as long as the other
% torques are smaller than Fc (Karnopp friction model).
% The parameters are estimated on the WHOLE test files (no cutting), by
% simulating the model and minimising the error with the measurement
% (nonlinear least squares, lsqnonlin). CONTSID cannot be used for this
% step: it only identifies LINEAR models, and sign(speed) is not linear.
% The viscous damping (Dp, Dy) is re-estimated TOGETHER with Fc: the
% CONTSID value of Dp/Dy already contains part of the friction effect,
% keeping it would count the damping twice.
% Jp and Jy are kept (geometry, quanser_aero_parameters.m).
% The starting point is the linear model identified with CONTSID
% (identify_quanser_AERO.m).
%
% ESTIMATION AND VALIDATION DATA
% ------------------------------
%   pitch: estimation on custom_pitch_free_8V (Vp 0/-8/0/+8 V, yaw free),
%          validation on the staircase custom_pitchsteps_8V_locked and on
%          the 1 V ramp custom_freeramp_pitch (pitch output)
%   yaw  : estimation on custom_yaw_free_8V (Vy 0/-8/0/+8 V),
%          validation on the old 10 V step step_yaw_10V (if the file is there)
%          -> only the FIRST tValYaw seconds of the validation are shown and
%             used for the fit (see section 0). After ~15 s the yaw spins
%             faster and faster (V^2 thrust, > 100 deg/s), far outside the
%             range of the controllers, where the model is not meant to work.
% In every test only ONE motor is used, so only the direct terms (Kpp, Kyy)
% matter; the cross terms Kpy, Kyp are kept from CONTSID.
% The yaw is compared on its angle AND on its rate (the rate shows the
% stops and the dead zone much better than the angle).
%
% WHAT TO LOOK AT
% ---------------
%   - fit (%) linear model vs model with friction, on estimation and
%     validation data (validation = data not used for the estimation)
%   - the new viscous damping: much smaller than the CONTSID value
%     (the "high damping" of the linear model was mostly friction)
%   - Fc and what it means physically: stuck band of the pitch (Fc/Ksp)
%     and breakaway voltage of the yaw (Fc_y/Kyy)
%   - yaw: one set of parameters for both the 8 V square and the 10 V step
%     (the linear model needed a different time constant for each)
%
% REQUIREMENTS
% ------------
% Optimization Toolbox (lsqnonlin). Without it, the script uses fminsearch
% (slower, same result in most cases). Files in the current folder:
% quanser_aero_parameters.m, the identified parameters .mat and the tests.

clear; close all; clc;

%% ===================== 0. SETTINGS =====================
identFile = 'quanser_aero_parameters_identified.mat';   % from identify_quanser_AERO.m

% Pure delay of each motor + propeller (s). The pitch value is the one found
% by tfsrivc; for the yaw, ~0.1 s (propeller spin-up). Not estimated here.
TdPitch = 0.05;
TdYaw   = 0.10;

% Estimation files {file, input row, output row}
estPitch = {'custom_pitch_free_8V', 11, 4};
estYaw   = {'custom_yaw_free_8V',   12, 5};
% Validation files (skipped if not in the folder)
valPitch = {'custom_pitchsteps_8V_locked', 11, 4;
            'custom_freeramp_pitch',              11, 4};
valYaw   = {'step_yaw_10V',                12, 5};
tValYaw  = 15;  % (s) only the first tValYaw seconds of the yaw validation are
                %     shown and used for the fit (instead of the whole 40 s)

dec = 2;        % keep 1 sample out of 2 (4 ms): 2x faster, same result
nSmooth = 25;   % moving average (samples) before differentiating the yaw angle
wAngle  = 0.3;  % weight of the yaw ANGLE error compared with the yaw RATE error

%% ===================== 1. PARAMETERS OF THE LINEAR (CONTSID) MODEL =====================
addpath('../LQR')
quanser_aero_parameters;            % Jp, Jy (geometry, kept)
L = load(identFile);                % linear parameters identified with CONTSID
fprintf('Linear model (CONTSID, %s):\n', identFile);
fprintf('  Ksp = %.5f  Dp = %.5f  Kpp = %.6f  |  Dy = %.5f  Kyy = %.6f\n\n', ...
        L.Ksp, L.Dp, L.Kpp, L.Dy, L.Kyy);

useLsq = license('test', 'Optimization_Toolbox') && exist('lsqnonlin', 'file') == 2;
if ~useLsq
    fprintf('Optimization Toolbox not found: fminsearch is used instead of lsqnonlin.\n\n');
end

%% ===================== 2. PITCH: Ksp, Dp, Kpp, Fc_p =====================
[t, u, y] = loadTest(estPitch{:}, dec);
dt = t(2) - t(1);
uD = delayInput(u, TdPitch, dt);     % Vp delayed by TdPitch

% Parameter vector p = [Ksp Dp Kpp Fc_p]
% Simulated pitch: simFriction(input torque, J, spring, viscous, Coulomb)
pitchModel = @(p, uu) simFriction(p(3)*uu, Jp, p(1), p(2), p(4), dt);
resPitch   = @(p) pitchModel(p, uD) - y;            % error to minimise (rad)

p0 = [L.Ksp, L.Dp/2, L.Kpp, 3e-4];    % start: CONTSID values, half the damping
pP = runFit(resPitch, p0, useLsq);    % nonlinear least squares

yLin = pitchModel([L.Ksp L.Dp L.Kpp 0], uD);   % linear model (Fc = 0, CONTSID values)
yFr  = pitchModel(pP, uD);                      % model with friction
fprintf('===== PITCH (estimation on %s) =====\n', estPitch{1});
fprintf('                 linear (CONTSID)   with friction\n');
fprintf('  Ksp (N.m/rad)     %9.5f        %9.5f\n', L.Ksp, pP(1));
fprintf('  Dp  (N.m.s/rad)   %9.5f        %9.5f\n', L.Dp,  pP(2));
fprintf('  zeta              %9.3f        %9.3f\n', L.Dp/(2*sqrt(L.Ksp*Jp)), pP(2)/(2*sqrt(pP(1)*Jp)));
fprintf('  Kpp (N.m/V)       %9.6f        %9.6f\n', L.Kpp, pP(3));
fprintf('  Fc_p (N.m)        %9s        %9.6f\n', '-', pP(4));
fprintf('  -> stuck band of the pitch: +/- %.2f deg (Fc_p/Ksp)\n', pP(4)/pP(1)*180/pi);
fprintf('  fit on estimation data: linear %.1f %%, with friction %.1f %%\n', ...
        fitPercent(y, yLin), fitPercent(y, yFr));

figure('Name', 'Pitch: linear vs friction model');
nV = size(valPitch, 1);
ax = subplot(nV+1, 1, 1);
plotThree(ax, t, y, yLin, yFr, ['Estimation - ' estPitch{1}], '\theta (deg)');
for v = 1:nV
    if ~isfile([valPitch{v,1} '.mat']), continue; end
    [tv, uv, yv] = loadTest(valPitch{v,:}, dec);
    uvD = delayInput(uv, TdPitch, dt);
    yvL = pitchModel([L.Ksp L.Dp L.Kpp 0], uvD);
    yvF = pitchModel(pP, uvD);
    fprintf('  validation on %-28s: linear %6.1f %%, with friction %6.1f %%\n', ...
            valPitch{v,1}, fitPercent(yv, yvL), fitPercent(yv, yvF));
    ax = subplot(nV+1, 1, v+1);
    plotThree(ax, tv, yv, yvL, yvF, ['Validation - ' valPitch{v,1}], '\theta (deg)');
end
xlabel('Time (s)');

%% ===================== 3. YAW: Dy, Kyy, Fc_y =====================
[t, u, y] = loadTest(estYaw{:}, dec);
dt = t(2) - t(1);
uD = delayInput(u, TdYaw, dt);
r  = yawRate(y, dt, nSmooth);          % measured yaw rate (rad/s)

% Parameter vector q = [Dy Kyy Fc_y]   (no spring on the yaw: Ks = 0)
yawModel = @(q, uu) simFriction(q(2)*uu, Jy, 0, q(1), q(3), dt);
% Error on the rate (shows the stops) and on the angle (shows the drift)
resYaw = @(q) [gradient(yawModel(q, uD), dt) - r; wAngle*(yawModel(q, uD) - y)];

% Start: the friction must be overcome, so the motor torque gain is larger
% than in the linear model (x2), Fc at half the 8 V torque, half the damping
q0 = [L.Dy/2, 2*L.Kyy, 0.5*(2*L.Kyy)*8];
qY = runFit(resYaw, q0, useLsq);

yLin = yawModel([L.Dy L.Kyy 0], uD);
yFr  = yawModel(qY, uD);
fprintf('\n===== YAW (estimation on %s) =====\n', estYaw{1});
fprintf('                 linear (CONTSID)   with friction\n');
fprintf('  Dy  (N.m.s/rad)   %9.5f        %9.5f\n', L.Dy, qY(1));
fprintf('  time constant Jy/Dy (s) %5.2f        %9.2f\n', Jy/L.Dy, Jy/qY(1));
fprintf('  Kyy (N.m/V)       %9.6f        %9.6f\n', L.Kyy, qY(2));
fprintf('  Fc_y (N.m)        %9s        %9.5f\n', '-', qY(3));
fprintf('  -> breakaway voltage of the yaw: %.2f V (Fc_y/Kyy; measured dead zone ~6-7 V)\n', qY(3)/qY(2));
fprintf('  fit on estimation data: angle linear %.1f %% / friction %.1f %%   rate linear %.1f %% / friction %.1f %%\n', ...
        fitPercent(y, yLin), fitPercent(y, yFr), ...
        fitPercent(r, gradient(yLin, dt)), fitPercent(r, gradient(yFr, dt)));

figure('Name', 'Yaw: linear vs friction model');
nV = size(valYaw, 1);
ax = subplot(2*(nV+1), 1, 1);
plotThree(ax, t, y, yLin, yFr, ['Estimation - ' estYaw{1}], '\psi (deg)');
ax = subplot(2*(nV+1), 1, 2);
plotThree(ax, t, r, gradient(yLin, dt), gradient(yFr, dt), 'yaw rate', 'd\psi/dt (deg/s)');
for v = 1:nV
    if ~isfile([valYaw{v,1} '.mat'])
        fprintf('  validation file not found, skipped: %s.mat\n', valYaw{v,1});
        continue;
    end
    [tv, uv, yv] = loadTest(valYaw{v,:}, dec);
    uvD = delayInput(uv, TdYaw, dt);
    rv  = yawRate(yv, dt, nSmooth);
    yvL = yawModel([L.Dy L.Kyy 0], uvD);      % simulated on the whole file (starts at rest)
    yvF = yawModel(qY, uvD);
    rvL = gradient(yvL, dt);                  % model yaw rates
    rvF = gradient(yvF, dt);

    % Keep only the first tValYaw seconds: plot AND fit on this window.
    % (The models start from rest, so cutting after the simulation is the
    %  same as simulating only the first tValYaw seconds.)
    k = (tv - tv(1)) <= tValYaw;
    fprintf('  validation on %-20s (first %.0f s): angle linear %6.1f %% / friction %6.1f %%   rate linear %6.1f %% / friction %6.1f %%\n', ...
            valYaw{v,1}, tValYaw, fitPercent(yv(k), yvL(k)), fitPercent(yv(k), yvF(k)), ...
            fitPercent(rv(k), rvL(k)), fitPercent(rv(k), rvF(k)));
    ttl = sprintf('Validation - %s (first %.0f s)', valYaw{v,1}, tValYaw);
    ax = subplot(2*(nV+1), 1, 2*v+1);
    plotThree(ax, tv(k), yv(k), yvL(k), yvF(k), ttl, '\psi (deg)');
    ax = subplot(2*(nV+1), 1, 2*v+2);
    plotThree(ax, tv(k), rv(k), rvL(k), rvF(k), ...
              sprintf('yaw rate (first %.0f s)', tValYaw), 'd\psi/dt (deg/s)');
end
xlabel('Time (s)');

%% ===================== 4. SAVE =====================
% Linear part (for the LQR) stays in the CONTSID file. This file only adds
% the friction model, for simulation and for the report.
fricPitch = struct('Ksp', pP(1), 'Dp', pP(2), 'Kpp', pP(3), 'Fc', pP(4), 'Td', TdPitch);
fricYaw   = struct('Dy', qY(1), 'Kyy', qY(2), 'Fc', qY(3), 'Td', TdYaw);
save('aero_friction_parameters.mat', 'fricPitch', 'fricYaw', 'Jp', 'Jy', 'identFile');
fprintf('\nSaved: aero_friction_parameters.mat\n');

%% ======================= LOCAL FUNCTIONS =======================

function x = simFriction(Tin, J, Ks, Dv, Fc, dt)
% simFriction  Simulates  J*x'' = Tin - Ks*x - Dv*x' - Fc*sign(x')
% with static friction (Karnopp model), explicit Euler, step dt.
%   Tin : motor torque, one value per sample (N.m)
%   Ks  : spring stiffness (0 for the yaw)
%   Dv  : viscous damping;  Fc : Coulomb friction torque (>= 0)
% Rules:
%   - at rest (v = 0): if the other torques are smaller than Fc, the axis
%     stays at rest (static friction holds it); otherwise it starts moving
%     and friction opposes the motion with a torque Fc;
%   - moving: friction = Fc*sign(v), opposite to the motion;
%   - if the speed changes sign during one step, the axis stops (v = 0),
%     and the rule "at rest" is checked again at the next step.
N = numel(Tin);
x = zeros(N,1);
pos = 0;  v = 0;
for k = 1:N
    Tdrive = Tin(k) - Ks*pos;              % all torques except damping and friction
    if v == 0
        if abs(Tdrive) <= Fc
            a = 0;                         % stuck
        else
            a = (Tdrive - Fc*sign(Tdrive)) / J;   % breaks free
        end
    else
        a = (Tdrive - Dv*v - Fc*sign(v)) / J;
    end
    vNew = v + a*dt;
    if v ~= 0 && sign(vNew) ~= sign(v)
        vNew = 0;                          % speed crosses zero -> stop
    end
    v   = vNew;
    pos = pos + v*dt;
    x(k) = pos;
end
end

function p = runFit(resFun, p0, useLsq)
% runFit  Nonlinear least squares on the error vector resFun(p).
% All parameters are physical and positive (lower bound 0).
if useLsq
    opts = optimoptions('lsqnonlin', 'Display', 'off', ...
                        'FiniteDifferenceStepSize', 1e-3, 'TypicalX', p0);
    p = lsqnonlin(resFun, p0, zeros(size(p0)), [], opts);
else
    % fminsearch has no bounds: abs() keeps the parameters positive
    cost = @(p) sum(resFun(abs(p)).^2);
    p = abs(fminsearch(cost, p0, optimset('Display', 'off', 'MaxFunEvals', 4000, 'MaxIter', 4000)));
end
end

function [t, u, y] = loadTest(fileName, inRow, outRow, dec)
% loadTest  Loads one open-loop test (12-row data0 of AERO_basics.slx),
% keeps one sample out of dec, output converted from deg to rad.
S  = load([fileName '.mat']);
d0 = S.data0(:, 1:dec:end);
t  = d0(1,:)';
u  = d0(inRow,:)';
y  = d0(outRow,:)' * pi/180;
end

function uD = delayInput(u, Td, dt)
% delayInput  Pure delay of Td seconds (motor + propeller spin-up).
n  = round(Td/dt);
uD = [zeros(n,1); u(1:end-n)];
end

function r = yawRate(psi, dt, nSmooth)
% yawRate  Yaw rate from the encoder: moving average, then derivative.
% (One encoder count per sample = 0.0879 deg / 4 ms = 22 deg/s: smoothing needed.)
r = gradient(movmean(psi, nSmooth), dt);
end

function plotThree(ax, t, y, yLin, yFr, ttl, yl)
% plotThree  Measured (black), linear model (red dashed), friction model (blue).
% The fits in the legend are computed on the data given (so on the
% shortened window when the caller cuts the data).
plot(ax, t, y*180/pi, 'k', t, yLin*180/pi, 'r--', t, yFr*180/pi, 'b'); grid(ax, 'on');
ylabel(ax, yl); title(ax, ttl, 'Interpreter', 'none');
legend(ax, sprintf('measured'), sprintf('linear (%.0f %%)', fitPercent(y, yLin)), ...
       sprintf('with friction (%.0f %%)', fitPercent(y, yFr)), 'Location', 'best');
end

function f = fitPercent(y, ys)
% fitPercent  100*(1 - ||y - ys|| / ||y - mean(y)||): 100 % = identical.
f = 100 * (1 - norm(y - ys) / norm(y - mean(y)));
end