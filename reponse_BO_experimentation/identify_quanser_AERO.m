%% identify_quanser_AERO.m
% =========================================================================
%  IDENTIFICATION OF THE QUANSER AERO GREY-BOX MODEL PARAMETERS (CONTSID)
% =========================================================================
%
% GOAL
% ----
% Keep the physical model given by Quanser (2-DOF Lab Guide, section 2.1.2,
% equations 2.3 and 2.4) and only re-estimate its PARAMETERS from our own
% measurements with the CONTSID toolbox. The structure of the model is NOT
% changed (teacher's instruction).
%
% THE QUANSER MODEL (equations of motion, Lab Guide eq. 2.1 - 2.2)
% -----------------------------------------------------------------
%   Jp*theta'' + Dp*theta' + Ksp*theta = Kpp*Vp + Kpy*Vy     (pitch)
%   Jy*psi''   + Dy*psi'               = Kyp*Vp + Kyy*Vy     (yaw)
%
%   theta, psi : pitch and yaw angles (rad)
%   Vp, Vy     : pitch and yaw motor voltages (V)
%   Jp, Jy     : moments of inertia (kg.m^2)    -> known from the geometry
%   Ksp        : pitch "spring" stiffness (N.m/rad)
%   Dp, Dy     : viscous damping of each axis (N.m.s/rad)
%   Kpp, Kyy   : direct torque gains (N.m/V)     (motor -> its own axis)
%   Kpy, Kyp   : cross torque gains (N.m/V)      (motor -> the other axis)
%
% With zero initial conditions, the 4 transfer functions are (eq. 2.3, 2.4):
%
%   G11 = Theta/Vp = Kpp / (Jp s^2 + Dp s + Ksp)   -> 2nd order, no zero
%   G12 = Theta/Vy = Kpy / (Jp s^2 + Dp s + Ksp)   -> 2nd order, no zero
%   G21 = Psi/Vp   = Kyp / (s (Jy s + Dy))         -> 1st order + integrator
%   G22 = Psi/Vy   = Kyy / (s (Jy s + Dy))         -> 1st order + integrator
%
% Physical meaning:
%   - The pitch has a spring (Ksp): after a step it oscillates and settles
%     at a constant angle -> 2nd order, no integrator.
%   - The yaw has no spring: a constant torque makes it turn forever, its
%     angle keeps growing -> integrator.
%
% WHICH CONTSID ROUTINE FOR WHICH TRANSFER FUNCTION
% -------------------------------------------------
% procsrivc only handles 1st-order process models (teacher's remark):
%   'P1D' : G(s) = K * exp(-Td*s) / (1 + T*s)       (1st order + delay)
%   'P1I' : G(s) = K / ( s * (1 + T*s) )            (1st order + integrator)
% so:
%   - YAW (G21, G22)  : procsrivc(data, 'P1I')  -> exactly the Quanser form,
%                        estimated directly on the yaw angle.
%   - PITCH (G11, G12): procsrivc cannot do a 2nd order. We use the other
%                        CONTSID routine, tfsrivc, with 2 poles and 0 zero
%                        (np = 2, nz = 0), which is exactly the Quanser form
%                        K / (s^2 + a1 s + a0), plus a small pure delay.
%
% LINK BETWEEN THE CONTSID PARAMETERS AND THE QUANSER PARAMETERS
% --------------------------------------------------------------
% Pitch: dividing G11 by Jp gives
%   G11 = (Kpp/Jp) / (s^2 + (Dp/Jp) s + Ksp/Jp) = b0 / (s^2 + a1 s + a0)
%   so, with Jp known:  Ksp = Jp*a0 ,  Dp = Jp*a1 ,  Kpp = Jp*b0
%   Natural frequency wn = sqrt(a0), damping ratio zeta = a1/(2*wn),
%   static gain = b0/a0 (rad/V): steady pitch angle per volt.
%
% Yaw: dividing G22 by Dy gives
%   G22 = (Kyy/Dy) / ( s (1 + (Jy/Dy) s) ) = K / ( s (1 + T s) )
%   so, with Jy known:  Dy = Jy/T ,  Kyy = K*Dy
%   K = final yaw RATE per volt ((rad/s)/V), T = time to reach 63 % of it.
%
% SHARED DENOMINATORS
% -------------------
% In the Quanser model G11 and G12 have the SAME denominator (same pitch
% body, whatever motor pushes it); G21 and G22 too (same yaw body). So:
%   - Ksp and Dp come from G11 (main path); G12 only gives its static gain:
%       Kpy = (static gain of G12) * Ksp
%   - Dy comes from G22 (main path); G21 only gives its rate gain:
%       Kyp = K21 * Dy
% The script prints the poles found in G12 and G21 as a check: if they are
% far from those of G11 and G22, the Quanser structure misses something.
%
% WHICH TEST FOR WHICH TRANSFER FUNCTION
% --------------------------------------
%   G11 : square wave on Vp (custom_pitch_free_8V) -> pitch output
%   G12 : square wave on Vy (custom_yaw_free_8V)   -> pitch output
%   G21 : 10 V step on Vp   (step_pitch_10V)       -> yaw output
%   G22 : square wave on Vy (custom_yaw_free_8V)   -> yaw output
% All with the yaw FREE (as the real system is used).
%
% COMPARISON WITH THE QUANSER SCRIPTS (find_*.m)
% ----------------------------------------------
% Quanser found its parameters by reading 2 points by hand on a plot
% (ginput): log decrement for Ksp/Dp, 63 % time for Dy, slope of the rate
% for Kpp, Kyy, Kpy, Kyp. CONTSID does the same job but uses ALL the
% samples of the test with an optimal method (SRIVC), so the result does
% not depend on which 2 points are picked.
%
% UNITS
% -----
% Angles in rad and voltages in V, so the gains come out in N.m/V, the same
% units as quanser_aero_parameters.m.
%
% REQUIREMENTS
% ------------
% System Identification + Control System Toolboxes + CONTSID 7.6. The .mat
% files must be in the current folder, each containing the 12-row matrix
% data0 of AERO_basics.slx:
%   1 time | 2-3 motor currents | 4 pitch (deg) | 5 yaw (deg)
%   6-7 propeller speeds | 8-10 gyro | 11 Vp (V) | 12 Vy (V)

clear; close all; clc;

%% ===================== 0. SETTINGS =====================
% Folder where CONTSID was unzipped (the folder that contains procsrivc.p)
contsidPath = 'C../CONTSID7_6';   % <-- CHANGE to your CONTSID folder
addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'CONTSID7_6'))   % CONTSID 7.6 toolbox

% One line per transfer function:
%   name | file (without .mat) | input row | output row | method
%   input row  : 11 = Vp (pitch motor), 12 = Vy (yaw motor)
%   output row :  4 = pitch angle,       5 = yaw angle
%   method     : 'TF2' = tfsrivc, 2 poles, 0 zero (pitch)
%                'P1I' = procsrivc, 1st order + integrator (yaw)
tests = {
  'G11', 'custom_pitch_free_8V',    11, 4, 'TF2';   % yaw free, Vp 0/-8/0/+8 V
  'G12', 'custom_yaw_free_8V',      12, 4, 'TF2';   % yaw free, Vy 0/-8/0/+8 V
  'G21', 'step_pitch_10V',          11, 5, 'P1I';   % yaw free, Vp step 0 -> 10 V
  'G22', 'custom_yaw_free_8V',      12, 5, 'P1I';   % yaw free, Vy 0/-8/0/+8 V
};
% All the tests are now done with the yaw FREE (teacher's remark: locking
% the yaw does not reflect the real system). At 8 V the yaw turns slowly
% (max ~22 deg/s), so the centrifugal effect on the pitch is negligible
% (psi_dot^2 ~ 0.15 vs wn^2 ~ 1.4): the locked tests gave the same G11/G12.
% The locked files (custom_pitch_8V_blocked, custom_yaw_blocked_8V) can
% still be put here to compare.
% Why the old 10 V step for G21 and not custom_pitch_free_8V?
% At 8 V the pitch motor only "kicks" the yaw while its propeller speeds up
% or slows down (reaction torque), then dry friction stops the yaw. A
% "1st order + integrator" cannot describe a yaw that stops by itself
% (fit ~10 %). At 10 V the propeller drag torque is larger than the yaw
% friction and the yaw keeps turning, as the model assumes.
% Put 'custom_pitch_free_8V' on the G21 line to see the difference.

TdMax = 0.1;   % (s) largest pure delay searched by tfsrivc for the pitch
               % (motor + propeller spin-up, ~0.05 s on the AERO)

% The AERO was at rest (0 V, angles at 0) before each test. We add tPad
% seconds of zeros at the start so that the first change of the input is
% inside the data (needed for the 10 V step, which starts at t = 0).
tPad = 0.5;    % (s)

% Validation data: free 2-DOF tests (Vp +/-6 V, period 20 s and Vy +/-9 V,
% period 26 s, at the same time). These files are NOT used for the
% estimation: they test the model on new data (cross-validation). Only the
% first 88 s are used: at ~91 s both inputs switch to negative within 1 s,
% the pitch hits the base and QUARC stops.
valFiles = {'custom_free_2dof1', 'custom_free_2dof2'};
tValEnd  = 88;   % (s)


%% ===================== CHECK THAT CONTSID IS AVAILABLE =====================
% exist(...) returns 2 for a .m file and 6 for a .p file (CONTSID is p-coded)
if ~any(exist('procsrivc', 'file') == [2 6])
    if isfolder(contsidPath), addpath(contsidPath); end
end
if ~any(exist('procsrivc', 'file') == [2 6]) || ~any(exist('tfsrivc', 'file') == [2 6])
    error('CONTSID not found: set contsidPath to the folder that contains procsrivc.p');
end

%% ===================== 1. IDENTIFICATION OF THE 4 TRANSFER FUNCTIONS =====================
% For each line of "tests": load the data, build an iddata object, estimate
% the model, simulate it with the same input and compare with the measured
% output. The results are stored in the structure array R.
nT = size(tests, 1);
R  = struct('name', {}, 'G', {}, 'fit', {}, 'par', {});

figure('Name', 'Identification of the 4 transfer functions');
for i = 1:nT
    % --- Load one test: time, input (V) and output (rad), with zero padding
    [t, u, y] = loadTest(tests{i,2}, tests{i,3}, tests{i,4}, tPad);
    Ts = t(2) - t(1);     % sampling period (2 ms)

    % --- iddata: output first, then input. 'zoh' (zero-order hold): the
    %     voltage is constant between two samples, as with HIL Write.
    data = iddata(y, u, Ts, 'InterSample', 'zoh');

    switch tests{i,5}
        case 'TF2'
            % ---- PITCH: tfsrivc with 2 poles, 0 zero (+ delay search)
            M  = tfsrivc(data, 2, 0, 'TdMax', TdMax);
            num = M.Numerator;  den = M.Denominator;
            % Normalise so that den = [1 a1 a0] and keep the last numerator
            % coefficient b0 (the numerator is a constant: no zero)
            b0 = num(end) / den(1);
            a1 = den(2)   / den(1);
            a0 = den(3)   / den(1);
            par.b0 = b0;  par.a1 = a1;  par.a0 = a0;
            par.wn   = sqrt(a0);              % natural frequency (rad/s)
            par.zeta = a1 / (2*sqrt(a0));     % damping ratio
            par.Kdc  = b0 / a0;               % static gain (rad/V)
            par.Td   = M.IODelay;             % pure delay (s)
            G = tf(b0, [1 a1 a0], 'IODelay', M.IODelay);

        case 'P1I'
            % ---- YAW: procsrivc, 1st order + integrator, on the yaw angle
            M  = procsrivc(data, 'P1I');
            par.K = M.Kp;                     % final yaw rate per volt ((rad/s)/V)
            par.T = M.Tp1;                    % time constant (s)
            G = tf(M.Kp, [M.Tp1 1 0]);        % K / (s (1 + T s))
    end

    % --- Simulation of the model with the measured input (zero initial
    %     conditions) and fit in %
    ys = lsim(G, u, t, 'zoh');
    R(i).name = tests{i,1};
    R(i).G    = G;
    R(i).fit  = fitPercent(y, ys);
    R(i).par  = par;
    clear par

    % --- Plot measured vs model (one subplot per transfer function)
    subplot(nT, 1, i);
    plot(t, y*180/pi, 'k', t, ys*180/pi, 'b'); grid on;
    ylabel([tests{i,1} ' (deg)']);
    title(sprintf('%s from %s (%s): fit %.1f %%', tests{i,1}, tests{i,2}, ...
          tests{i,5}, R(i).fit), 'Interpreter', 'none');
    legend('measured', 'model', 'Location', 'best');
end
xlabel('Time (s)');

% Short names for the rest of the script
P11 = R(1).par;  P12 = R(2).par;  P21 = R(3).par;  P22 = R(4).par;

% Print the identified models in physical terms
fprintf('===== Identified models =====\n');
fprintf('  G11 (pitch <- Vp): wn = %.3f rad/s, zeta = %.3f, static gain = %.3f deg/V, Td = %.3f s, fit %.1f %%\n', ...
        P11.wn, P11.zeta, P11.Kdc*180/pi, P11.Td, R(1).fit);
fprintf('  G12 (pitch <- Vy): wn = %.3f rad/s, zeta = %.3f, static gain = %.3f deg/V, Td = %.3f s, fit %.1f %%\n', ...
        P12.wn, P12.zeta, P12.Kdc*180/pi, P12.Td, R(2).fit);
fprintf('  G21 (yaw <- Vp)  : rate gain K = %.3f (deg/s)/V, T = %.2f s, fit %.1f %%\n', ...
        P21.K*180/pi, P21.T, R(3).fit);
fprintf('  G22 (yaw <- Vy)  : rate gain K = %.3f (deg/s)/V, T = %.2f s, fit %.1f %%\n', ...
        P22.K*180/pi, P22.T, R(4).fit);

%% ===================== 2. PHYSICAL PARAMETERS (Quanser structure) =====================
% quanser_aero_parameters.m gives Jp and Jy (from the masses and dimensions,
% we keep them) and the default Quanser values, stored in Q to compare.
quanser_aero_parameters;
Q = struct('Ksp', Ksp, 'Dp', Dp, 'Kpp', Kpp, 'Kpy', Kpy, 'Dy', Dy, 'Kyy', Kyy, 'Kyp', Kyp);

% Identified parameters (formulas explained in the header)
P.Ksp = Jp * P11.a0;          % stiffness       (N.m/rad)    from G11
P.Dp  = Jp * P11.a1;          % pitch damping   (N.m.s/rad)  from G11
P.Kpp = Jp * P11.b0;          % pitch motor -> pitch (N.m/V) from G11
P.Kpy = P12.Kdc * P.Ksp;      % yaw motor   -> pitch (N.m/V) from G12 static gain
P.Dy  = Jy / P22.T;           % yaw damping     (N.m.s/rad)  from G22
P.Kyy = P22.K * P.Dy;         % yaw motor   -> yaw   (N.m/V) from G22
P.Kyp = P21.K * P.Dy;         % pitch motor -> yaw   (N.m/V) from G21 rate gain

% Comparison table: ratio = identified / Quanser
% (ratio < 1: the real AERO is weaker than the Quanser model on that term)
fprintf('\n===== Parameters (Jp = %.4f, Jy = %.4f kg.m^2 kept) =====\n', Jp, Jy);
fprintf('            Quanser     identified   ratio\n');
names = fieldnames(P);
for k = 1:numel(names)
    fprintf('  %-4s   %10.5f   %10.5f   %6.2f\n', names{k}, Q.(names{k}), P.(names{k}), ...
            P.(names{k}) / Q.(names{k}));
end

% Consistency check of the Quanser structure: G12 should have the same
% poles as G11 (same pitch body), and G21 the same time constant as G22
% (same yaw body). Large differences show what the 4-state model misses.
fprintf('\n  Check pitch body: G12 wn = %.3f, zeta = %.3f  vs  G11 wn = %.3f, zeta = %.3f\n', ...
        P12.wn, P12.zeta, P11.wn, P11.zeta);
fprintf('  Check yaw body  : G21 T = %.2f s  vs  G22 T = %.2f s\n', P21.T, P22.T);
% The 4-state model has no delay: the pitch delay found by tfsrivc is only reported
fprintf('  Pitch delay found (not in the 4-state model): Td = %.3f s\n', P11.Td);

%% ===================== 3. STATE-SPACE MODEL (same form as quanser_aero_state_space.m) =====================
% State x = [theta; psi; theta_dot; psi_dot], input u = [Vp; Vy],
% output y = [theta; psi]. Rows 3 and 4 = the equations of motion / J.
A = [0 0 1 0;
     0 0 0 1;
     -P.Ksp/Jp 0 -P.Dp/Jp 0;
     0 0 0 -P.Dy/Jy];
B = [0 0;
     0 0;
     P.Kpp/Jp P.Kpy/Jp;
     P.Kyp/Jy P.Kyy/Jy];
C = eye(2,4);        % we measure the two angles
D = zeros(2,2);      % no direct feedthrough
sysId = ss(A, B, C, D);          % identified model

% Same structure with the default Quanser parameters, for comparison
AQ = [0 0 1 0; 0 0 0 1; -Q.Ksp/Jp 0 -Q.Dp/Jp 0; 0 0 0 -Q.Dy/Jy];
BQ = [0 0; 0 0; Q.Kpp/Jp Q.Kpy/Jp; Q.Kyp/Jy Q.Kyy/Jy];
sysQ = ss(AQ, BQ, C, D);         % Quanser model

%% ===================== 4. VALIDATION ON THE FREE 2-DOF TESTS =====================
% Both models are simulated with the two measured voltages at the same time.
% Fit: 100 % = perfect, 0 % = as good as a constant, negative = worse.
fprintf('\n===== Validation on the free 2-DOF tests (0-%g s) =====\n', tValEnd);
for v = 1:numel(valFiles)
    if ~isfile([valFiles{v} '.mat'])
        fprintf('  File not found, skipped: %s.mat\n', valFiles{v});
        continue;
    end
    S  = load([valFiles{v} '.mat']);
    d0 = S.data0;
    k  = d0(1,:)' <= tValEnd;                   % valid part only
    t  = d0(1,k)';                              % time (s)
    u  = [d0(11,k)' d0(12,k)'];                 % inputs [Vp Vy] (V)
    y  = [d0(4,k)' d0(5,k)'] * pi/180;          % outputs [pitch yaw] (rad)

    yId = lsim(sysId, u, t, 'zoh');             % identified model
    yQ  = lsim(sysQ,  u, t, 'zoh');             % Quanser model

    fprintf('  %s: pitch fit identified %6.1f %% (Quanser %6.1f %%) | yaw fit identified %6.1f %% (Quanser %6.1f %%)\n', ...
            valFiles{v}, fitPercent(y(:,1), yId(:,1)), fitPercent(y(:,1), yQ(:,1)), ...
            fitPercent(y(:,2), yId(:,2)), fitPercent(y(:,2), yQ(:,2)));

    figure('Name', ['Validation ' valFiles{v}]);
    ax(1) = subplot(3,1,1);
    plot(t, y(:,1)*180/pi, 'k', t, yId(:,1)*180/pi, 'b', t, yQ(:,1)*180/pi, 'r--'); grid on;
    ylabel('\theta (deg)'); legend('measured', 'identified', 'Quanser', 'Location', 'best');
    title(['Validation - ' valFiles{v}], 'Interpreter', 'none');
    ax(2) = subplot(3,1,2);
    plot(t, y(:,2)*180/pi, 'k', t, yId(:,2)*180/pi, 'b', t, yQ(:,2)*180/pi, 'r--'); grid on;
    ylabel('\psi (deg)');
    ax(3) = subplot(3,1,3);
    plot(t, u(:,1), 'k', t, u(:,2), 'g'); grid on;
    ylabel('V'); legend('Vp', 'Vy', 'Location', 'best'); xlabel('Time (s)');
    linkaxes(ax, 'x');
end

%% ===================== 5. SAVE =====================
% Same variable names as quanser_aero_parameters.m, so tune_lqr_AERO.m can
% use them (load this .mat file instead of calling quanser_aero_parameters).
Ksp = P.Ksp; Dp = P.Dp; Kpp = P.Kpp; Kpy = P.Kpy; Dy = P.Dy; Kyy = P.Kyy; Kyp = P.Kyp;
fits = [R.fit];
save('quanser_aero_parameters_identified.mat', 'Jp', 'Jy', 'Ksp', 'Dp', 'Kpp', 'Kpy', ...
     'Dy', 'Kyy', 'Kyp', 'A', 'B', 'C', 'D', 'tests', 'fits');
fprintf('\nSaved: quanser_aero_parameters_identified.mat\n');

%% ===================== LOCAL FUNCTIONS =====================
function [t, u, y] = loadTest(fileName, inRow, outRow, tPad)
% loadTest  Loads one open-loop test recorded with AERO_basics.slx.
%   fileName : .mat file name without extension (contains data0, 12 rows)
%   inRow    : row of the input  (11 = Vp, 12 = Vy)
%   outRow   : row of the output (4 = pitch, 5 = yaw), in degrees
%   tPad     : seconds of zeros added at the start (AERO at rest)
% Returns the time t (s), the input u (V) and the output y (rad), as columns.
S  = load([fileName '.mat']);
d0 = S.data0;
Ts = d0(1,2) - d0(1,1);                      % sampling period (s)
nPad = round(tPad / Ts);                     % number of zero samples to add
u = [zeros(nPad,1); d0(inRow,:)'];           % input (V)
y = [zeros(nPad,1); d0(outRow,:)'] * pi/180; % output, deg -> rad
t = (0:length(u)-1)' * Ts;                   % new time vector, starts at 0
end

function f = fitPercent(y, ys)
% fitPercent  Same definition as MATLAB compare / CONTSID comparec:
%   fit = 100 * (1 - ||y - ys|| / ||y - mean(y)||)
%   100 % = perfect simulation, 0 % = as good as a constant (the mean),
%   negative = worse than a constant.
f = 100 * (1 - norm(y - ys) / norm(y - mean(y)));
end