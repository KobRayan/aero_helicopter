% setup_decoupled_pd_simulink.m
% 14/10/2026
% Workspace variables for the DECOUPLED PD Simulink model
% (copy of q_aero_2dof_pid_control.slx, e.g. q_aero_2dof_dpd_control.slx).
%
% Run it before the model, or put its name in
%   Model Settings > Model Properties > Callbacks > InitFcn
% so the variables are always right when the model starts.
% No "clear" here: it must not erase the other variables of the workspace.
%
% UNITS - the important point:
%   the two PIV blocks output TORQUES (N.m), not volts. The Gain block W
%   (decoupler) turns the two torques into the two motor voltages.
%   So kp_p, kd_p, kp_y, kd_y below are in N.m/rad and N.m.s/rad
%   (small numbers, ~0.05-0.12). Do NOT run quanser_aero_pd_design.m after
%   this script: it would overwrite kp_p and kd_p with values in volts.
%
% Same design as "Decoupled PD" in decoupled_pd_AERO.m (PO = 7.5 %, tp = 1.75 s).

%% 1) Identified model and specifications
addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'reponse_BO_experimentation'))   % identified parameters (.mat)
identFile = 'quanser_aero_parameters_identified.mat';
P  = load(identFile);
PO = 7.5;                       % overshoot (%)
tp = 1.75;                      % peak time (s)
zeta = -log(PO/100) / sqrt(log(PO/100)^2 + pi^2);
wn   = pi / (tp*sqrt(1 - zeta^2));

%% 2) PIV gains, in TORQUE units (Ki = 0 -> PD)
kp_p = P.Jp*wn^2 - P.Ksp;       % N.m/rad    (pitch)
kd_p = 2*zeta*wn*P.Jp - P.Dp;   % N.m.s/rad  (pitch)
kp_y = P.Jy*wn^2;               % N.m/rad    (yaw)
kd_y = 2*zeta*wn*P.Jy - P.Dy;   % N.m.s/rad  (yaw)
Wn_vel = 50;                    % cut-off of the PIV velocity filter (rad/s), as State X / simulation

%% 3) Spring feedforward and decoupler
Ksp  = P.Ksp;                   % N.m/rad, multiplies the pitch reference (rad)
cKpy = 1;                       % decoupler trims, 1 = identified values
cKyp = 0.75;                       % (calibration on the AERO, see report section 6)
Bt   = [P.Kpp      cKpy*P.Kpy;
        cKyp*P.Kyp P.Kyy];      % torques = Bt * voltages
W    = inv(Bt);                 % voltages = W * torques  (Gain block, Matrix(K*u))
satV = 20;                      % voltage saturation (V)
prio = 1;                       % pitch priority (only for the MATLAB Function version)

%% 4) References (deg): same staggered ramps as the simulation
% One axis moves at a time, so the coupling can be measured.
% refScale = 0.5 for the first, small test; 1 for the real test.
Ts       = 0.002;
Tend     = 38;                  % set the model Stop time to this value
rateLim  = 20;                  % deg/s
refScale = 1;
pitchMoves = [ 1  20;           % [time (s), new pitch target (deg)]
              13 -20;
              26   0];
yawMoves   = [ 6  40;           % [time (s), new yaw target (deg)]
              18 -40;
              31   0];
tRef = (0:Ts:Tend)';
refP_ts = timeseries(refScale*makeReference(tRef, pitchMoves, rateLim), tRef);   % From Workspace blocks
refY_ts = timeseries(refScale*makeReference(tRef, yawMoves,   rateLim), tRef);

%% 5) Display and sanity checks
Keq = W*[kp_p 0 kd_p 0; 0 kp_y 0 kd_y];     % equivalent state feedback (V/rad, V.s/rad)
fprintf('Decoupled PD: zeta = %.3f, wn = %.2f rad/s\n', zeta, wn);
fprintf('PIV pitch: Kp = %.4f N.m/rad, Kv = %.4f N.m.s/rad\n', kp_p, kd_p);
fprintf('PIV yaw  : Kp = %.4f N.m/rad, Kv = %.4f N.m.s/rad\n', kp_y, kd_y);
fprintf('Spring feedforward Ksp = %.4f N.m/rad\n', Ksp);
fprintf('W (V per N.m) =\n');  disp(W)
fprintf('Equivalent Keq (same signs as the LQR K: [+ - + - ; + + + +]) =\n');  disp(Keq)
fprintf('Stop time = %g s, refScale = %g\n', Tend, refScale);

%% ======================= LOCAL FUNCTIONS =======================

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
