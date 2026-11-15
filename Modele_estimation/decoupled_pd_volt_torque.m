% Decoupled PD gains for q_aero_2dof_pid_control (torque units)
addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'reponse_BO_experimentation'))   % identified parameters (.mat)
P  = load('quanser_aero_parameters_identified.mat');
PO = 7.5;  tp = 1.75;                                   % Quanser specs
zeta = -log(PO/100) / sqrt(log(PO/100)^2 + pi^2);
wn   = pi / (tp*sqrt(1 - zeta^2));
kp_p = P.Jp*wn^2 - P.Ksp;   kd_p = 2*zeta*wn*P.Jp - P.Dp;   % 0.0878 N.m/rad, 0.0563 N.m.s/rad
kp_y = P.Jy*wn^2;           kd_y = 2*zeta*wn*P.Jy - P.Dy;   % 0.1191, 0.0536
Ksp  = P.Ksp;                                               % spring feedforward (N.m/rad)
W    = inv([P.Kpp P.Kpy; P.Kyp P.Kyy]);                     % decoupler (V per N.m)