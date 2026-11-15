%% identify_steps_AERO.m
% Identification of the AERO transfer functions from ONE open-loop step test
% with the CONTSID toolbox (tfsrivc = SRIVC method, continuous-time models).
%
% One step test excites ONE input, so it gives TWO transfer functions:
%   pitch step test (input Vp) ->  G11 = theta/Vp   and   G21 = psi/Vp
%   yaw   step test (input Vy) ->  G12 = theta/Vy   and   G22 = psi/Vy
% Run the script once per test file (change fileName below).
%
% Model structures (Quanser equations of motion, Lab Guide eq. 2.1-2.4):
%   Jp*theta'' + Dp*theta' + Ksp*theta = Kpp*Vp + Kpy*Vy
%   Jy*psi''   + Dy*psi'               = Kyp*Vp + Kyy*Vy
%   -> pitch output : 2 poles, no zero, no integrator (the spring Ksp)
%   -> yaw output   : 1 integrator (no spring on the yaw axis) x 1 pole
% The yaw angle keeps growing in a step test (it is not stable), which is
% hard for an output-error method. So we identify the yaw RATE psi_dot
% (stable, it settles) and add the integrator 1/s afterwards:
%   G_yaw(s) = (1/s) * G_rate(s)
%
% The orders are NOT imposed: section 5 tries several (np, nz) and prints
% the fit, so you choose the smallest model that fits well (section 0).
%
% Requirements: System Identification Toolbox + CONTSID on the MATLAB path.
% Units: angles in rad, voltages in V  ->  torque gains come out in N.m/V,
% same units as quanser_aero_parameters.m.

clear; close all; clc;

%% ===================== 0. SETTINGS =====================
fileName  = 'step_yaw_10V';  % step_pitch_5V/10V/15V or step_yaw_05V/10V/15V
tPad      = 0.5;   % (s) zeros added BEFORE the step: the AERO was at rest
                   %     with Vp = Vy = 0 before the test started
TfitPitch = 10;    % (s) fit window for the pitch. After ~10 s the yaw
                   %     spins fast and pulls the pitch down (centrifugal
                   %     effect), which is not in the linear model
TfitYaw   = 40;    % (s) fit window for the yaw rate (it has a slow part,
                   %     time constant ~15 s, so we keep all the data)
TdMax     = 0.2;   % (s) largest pure delay searched by tfsrivc
nSmooth   = 25;    % samples of moving average before differentiating the
                   % yaw (25 x 2 ms = 50 ms). One yaw encoder count in one
                   % sample is 0.0879 deg / 2 ms = 44 deg/s: raw diff is useless

% Candidate orders [np nz]  (np = number of poles, nz = number of zeros)
ordersPitch   = [2 0; 2 1; 3 0; 3 1];   % 2 = spring-mass-damper, 3 = + motor lag
ordersYawRate = [1 0; 2 0; 2 1; 3 1];   % 1 = Quanser structure, 2-1 = fast + slow part

% Orders kept for the final models.
% CHOOSE THEM AFTER READING THE ORDER-SCAN TABLES (section 5): take the
% smallest model after which the fit stops improving (less than ~1-2 %).
orderPitch   = [2 0];
orderYawRate = [2 1];

%% ===================== 1. LOAD THE DATA =====================
addpath(genpath(fullfile('..', 'CONTSID7_6')))
addpath(fullfile('..', 'LQR')); quanser_aero_parameters

S = load([fileName '.mat']);
data0 = S.data0;

t            = data0(1,:)';    % time (s)
PitchEncoder = data0(4,:)';    % pitch angle (deg)
YawEncoder   = data0(5,:)';    % yaw angle (deg)
PitchVoltage = data0(11,:)';   % pitch motor voltage (V)
YawVoltage   = data0(12,:)';   % yaw motor voltage (V)
Ts = t(2) - t(1);              % sampling period (s)

% Which input was stepped?
if contains(fileName, 'pitch')
    u = PitchVoltage;   uOther = YawVoltage;
    inName = 'Vp';
    namePitch = 'G11 = theta/Vp';   nameYaw = 'G21 = psi/Vp';
    gainPitchName = 'Kpp';          gainYawName = 'Kyp';
elseif contains(fileName, 'yaw')
    u = YawVoltage;     uOther = PitchVoltage;
    inName = 'Vy';
    namePitch = 'G12 = theta/Vy';   nameYaw = 'G22 = psi/Vy';
    gainPitchName = 'Kpy';          gainYawName = 'Kyy';
else
    error(['Use a single-input test (pitch OR yaw). The 10V+10V test has ' ...
           'two inputs at once: it needs a 2-input (MISO) model.']);
end
if max(abs(uOther)) > 0.1
    warning('The other motor voltage is not zero in this test.');
end

%% ===================== 2. UNIT CHECK (encoder resolution) =====================
% Smallest non-zero change of an angle = one encoder count
dP = abs(diff(PitchEncoder));   dP = dP(dP > 1e-9);
dY = abs(diff(YawEncoder));     dY = dY(dY > 1e-9);
fprintf('Unit check (%s):\n', fileName);
if ~isempty(dP)
    fprintf('  pitch: smallest change %.4f deg (expected 360/2048 = 0.1758)\n', min(dP));
end
if ~isempty(dY)
    fprintf('  yaw  : smallest change %.4f deg (expected 360/4096 = 0.0879)\n', min(dY));
end

% Does the yaw move at all? (yaw dead zone: no motion below ~6-7 V)
yawMoves = max(abs(YawEncoder)) > 2;    % more than 2 deg of yaw motion
if ~yawMoves
    fprintf('  The yaw does not move in this test (dry friction): no yaw model.\n');
end

%% ===================== 3. PRE-PROCESSING =====================
% No detrend here: the test starts from rest at 0 deg / 0 V, so the data
% are already deviations from the operating point. Removing the mean
% would be wrong for a step from rest.
nPad   = round(tPad/Ts);
u      = [zeros(nPad,1); u];
theta  = [zeros(nPad,1); PitchEncoder] * pi/180;   % pitch (rad)
psi    = [zeros(nPad,1); YawEncoder]   * pi/180;   % yaw (rad)
N      = length(u);
t      = (0:N-1)' * Ts;                            % new time, step at t = tPad

% Yaw rate from the encoder: smooth, then differentiate (rad/s)
psiDot = gradient(movmean(psi, nSmooth), Ts);

% Fit windows (indices)
kP = t <= tPad + TfitPitch;
kY = t <= tPad + TfitYaw;

figure('Name', ['Data ' fileName]);
ax(1) = subplot(4,1,1); plot(t, u); grid on;
ylabel([inName ' (V)']); title(['Prepared data - ' fileName], 'Interpreter', 'none');
ax(2) = subplot(4,1,2); plot(t, theta*180/pi); grid on; ylabel('\theta (deg)');
xline(tPad + TfitPitch, 'r--', 'end of pitch fit');
ax(3) = subplot(4,1,3); plot(t, psi*180/pi); grid on; ylabel('\psi (deg)');
ax(4) = subplot(4,1,4); plot(t, psiDot*180/pi); grid on;
ylabel('d\psi/dt (deg/s)'); xlabel('Time (s)');
linkaxes(ax, 'x');

%% ===================== 4. IDDATA OBJECTS =====================
% 'zoh': the voltage is held constant between samples (HIL Write)
dataP = iddata(theta(kP),  u(kP), Ts, 'InterSample', 'zoh');
dataR = iddata(psiDot(kY), u(kY), Ts, 'InterSample', 'zoh');

%% ===================== 5. ORDER SCAN =====================
fitsP = scanOrders(dataP, ordersPitch, TdMax, ...
                   sprintf('%s (pitch, 0-%g s)', namePitch, TfitPitch));
if yawMoves
    fitsR = scanOrders(dataR, ordersYawRate, TdMax, ...
                       sprintf('%s x s (yaw rate, 0-%g s)', nameYaw, TfitYaw));
else
    fitsR = [];
end

%% ===================== 6. FINAL MODELS =====================
Mp = tfsrivc(dataP, orderPitch(1), orderPitch(2), 'TdMax', TdMax);
Gp = tf(Mp.Numerator, Mp.Denominator, 'IODelay', Mp.IODelay);   % theta/u (rad/V)
fprintf('\n===== %s, order [np nz] = [%d %d] =====\n', namePitch, orderPitch);
Gp
fprintf('  Delay: %.3f s,  static gain: %.3f deg/V\n', Mp.IODelay, dcgain(Gp)*180/pi);


if yawMoves
    Mr    = tfsrivc(dataR, orderYawRate(1), orderYawRate(2), 'TdMax', TdMax);
    Grate = tf(Mr.Numerator, Mr.Denominator, 'IODelay', Mr.IODelay);  % psi_dot/u
    Gyaw  = Grate * tf(1, [1 0]);                                       % psi/u (rad/V)
    fprintf('\n===== %s x s (yaw rate), order [np nz] = [%d %d] =====\n', ...
            nameYaw, orderYawRate);
    Grate
    fprintf('  Delay: %.3f s,  static rate gain: %.3f (deg/s)/V\n', ...
            Mr.IODelay, dcgain(Grate)*180/pi);
    pr = pole(Grate);  zr = zero(Grate);
    fprintf('  Time constants of the poles: %s s\n', mat2str(sort(-1./real(pr)).', 3));
    if ~isempty(zr)
        fprintf('  Time constant of the zero  : %s s\n', mat2str(-1./real(zr).', 3));
    end
else
    Mr = []; Grate = []; Gyaw = [];
end

%% ===================== 7. PHYSICAL PARAMETERS =====================
% Jp and Jy come from the geometry (Quanser), we keep them.
quanser_aero_parameters;     % Jp, Jy, Kpp, Kpy, Kyp, Kyy, Ksp, Dp, Dy

% Quanser grey-box transfer functions for the same input
if strcmp(inName, 'Vp')
    GpQ = tf(Kpp, [Jp Dp Ksp]);   GrQ = tf(Kyp, [Jy Dy]);
    KpQ = Kpp;                    KyQ = Kyp;
else
    GpQ = tf(Kpy, [Jp Dp Ksp]);   GrQ = tf(Kyy, [Jy Dy]);
    KpQ = Kpy;                    KyQ = Kyy;
end

% Pitch: dominant complex poles -> wn, zeta -> Ksp, Dp, torque gain
pp = pole(Gp);
pc = pp(abs(imag(pp)) > 1e-6);
if ~isempty(pc)
    wn   = abs(pc(1));
    zeta = -real(pc(1)) / wn;
    Ksp_id = Jp * wn^2;               % stiffness (N.m/rad)
    Dp_id  = 2 * zeta * wn * Jp;      % viscous damping (N.m.s/rad)
    Kp_id  = dcgain(Gp) * Ksp_id;     % torque gain (N.m/V)
else
    warning('No complex poles in the pitch model: wn and zeta not defined.');
    wn = NaN; zeta = NaN; Ksp_id = NaN; Dp_id = NaN; Kp_id = NaN;
end

% Yaw: slowest pole -> Dy = Jy/tau_slow ; static rate gain -> torque gain
% (if the model has a zero, the 4-state Quanser structure cannot hold the
%  fast part: these two numbers only match the steady state)
if yawMoves
    tauSlow = max(-1./real(pole(Grate)));
    Dy_id   = Jy / tauSlow;
    Ky_id   = dcgain(Grate) * Dy_id;
else
    tauSlow = NaN; Dy_id = NaN; Ky_id = NaN;
end

fprintf('\n===== Physical parameters (Jp = %.4f, Jy = %.4f kg.m^2 kept) =====\n', Jp, Jy);
fprintf('                       Quanser     identified\n');
fprintf('  wn pitch (rad/s)     %8.4f    %8.4f\n', sqrt(Ksp/Jp), wn);
fprintf('  zeta pitch           %8.4f    %8.4f\n', Dp/(2*sqrt(Ksp*Jp)), zeta);
fprintf('  Ksp  (N.m/rad)       %8.4f    %8.4f\n', Ksp, Ksp_id);
fprintf('  Dp   (N.m.s/rad)     %8.4f    %8.4f\n', Dp, Dp_id);
fprintf('  %s  (N.m/V)         %8.5f    %8.5f\n', gainPitchName, KpQ, Kp_id);
fprintf('  tau yaw (s)          %8.3f    %8.3f\n', Jy/Dy, tauSlow);
fprintf('  Dy   (N.m.s/rad)     %8.4f    %8.4f\n', Dy, Dy_id);
fprintf('  %s  (N.m/V)         %8.5f    %8.5f\n', gainYawName, KyQ, Ky_id);

%% ===================== 8. COMPARISON: MEASURED / IDENTIFIED / QUANSER =====================
thetaId = lsim(Gp,  u, t, 'zoh');
thetaQ  = lsim(GpQ, u, t, 'zoh');
fprintf('\n===== Fit (%%) on the fit windows =====\n');
fprintf('  pitch    : identified %6.1f   Quanser %6.1f\n', ...
        fitPercent(theta(kP), thetaId(kP)), fitPercent(theta(kP), thetaQ(kP)));

if yawMoves
    psiDotId = lsim(Grate, u, t, 'zoh');
    psiDotQ  = lsim(GrQ,   u, t, 'zoh');
    psiId    = lsim(Gyaw,  u, t, 'zoh');
    psiQ     = lsim(GrQ * tf(1,[1 0]), u, t, 'zoh');
    fprintf('  yaw rate : identified %6.1f   Quanser %6.1f\n', ...
            fitPercent(psiDot(kY), psiDotId(kY)), fitPercent(psiDot(kY), psiDotQ(kY)));
    fprintf('  yaw angle: identified %6.1f   Quanser %6.1f\n', ...
            fitPercent(psi(kY), psiId(kY)), fitPercent(psi(kY), psiQ(kY)));
end

figure('Name', ['Identification ' fileName]);
bx(1) = subplot(3,1,1);
plot(t, theta*180/pi, 'k', t, thetaId*180/pi, 'b', t, thetaQ*180/pi, 'r--'); grid on;
xline(tPad + TfitPitch, 'k:');
ylabel('\theta (deg)'); legend('measured', 'identified', 'Quanser', 'Location', 'best');
title([namePitch ' and ' nameYaw ' - ' fileName], 'Interpreter', 'none');
if yawMoves
    bx(2) = subplot(3,1,2);
    plot(t, psiDot*180/pi, 'k', t, psiDotId*180/pi, 'b', t, psiDotQ*180/pi, 'r--'); grid on;
    ylabel('d\psi/dt (deg/s)');
    bx(3) = subplot(3,1,3);
    plot(t, psi*180/pi, 'k', t, psiId*180/pi, 'b', t, psiQ*180/pi, 'r--'); grid on;
    ylabel('\psi (deg)'); xlabel('Time (s)');
    linkaxes(bx, 'x');
end

%% ===================== 9. SAVE =====================
params = struct('wn', wn, 'zeta', zeta, 'Ksp', Ksp_id, 'Dp', Dp_id, ...
                'Kpitch', Kp_id, 'tauYaw', tauSlow, 'Dy', Dy_id, 'Kyaw', Ky_id, ...
                'gainPitchName', gainPitchName, 'gainYawName', gainYawName);
save(['results_ident_' fileName '.mat'], 'Gp', 'Grate', 'Gyaw', 'Mp', 'Mr', ...
     'fitsP', 'fitsR', 'ordersPitch', 'ordersYawRate', 'orderPitch', ...
     'orderYawRate', 'params', 'inName', 'fileName');
fprintf('\nSaved: results_ident_%s.mat\n', fileName);

%% ===================== LOCAL FUNCTIONS =====================
function fits = scanOrders(data, orders, TdMax, label)
% Estimates one model per row of orders = [np nz] with tfsrivc and prints
% the fit of its simulated output (zero initial conditions).
y  = data.OutputData;
u  = data.InputData;
t  = (0:length(y)-1)' * data.Ts;
fits = nan(size(orders,1), 1);
fprintf('\nOrder scan: %s\n', label);
fprintf('  np  nz  n_par  Td (s)   fit (%%)   poles\n');
for i = 1:size(orders,1)
    np = orders(i,1);  nz = orders(i,2);
    try
        M  = tfsrivc(data, np, nz, 'TdMax', TdMax);
        G  = tf(M.Numerator, M.Denominator, 'IODelay', M.IODelay);
        ys = lsim(G, u, t, 'zoh');
        fits(i) = fitPercent(y, ys);
        fprintf('  %d   %d   %d      %6.3f   %6.2f    %s\n', np, nz, np+nz+1, ...
                M.IODelay, fits(i), mat2str(pole(G).', 3));
    catch err
        fprintf('  %d   %d   failed: %s\n', np, nz, err.message);
    end
end
end

function f = fitPercent(y, ys)
% Same definition as compare / comparec: 100 % = perfect simulation
f = 100 * (1 - norm(y - ys) / norm(y - mean(y)));
end
