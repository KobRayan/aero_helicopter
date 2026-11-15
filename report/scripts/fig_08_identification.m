% fig_08_identification.m
% Figure 8 (G11 et G22) et figure B1 (G12 et G21, annexe B) du rapport.
% Copie adaptée de reponse_BO_experimentation/identify_quanser_AERO.m.
% Le calcul est inchangé (mêmes fichiers d'essai, mêmes appels CONTSID
% tfsrivc / procsrivc, même formule du fit) ; seuls changent : les chemins,
% les libellés (français), la mise en forme (300 dpi, 16 cm de large, traits
% différents pour le noir et blanc), et l'enregistrement : le fichier
% quanser_aero_parameters_identified.mat du projet n'est PAS réécrit. Les
% paramètres recalculés sont comparés à ceux de ce fichier (jeu retenu) et
% imprimés dans rapport/checks/fig_08_identification_log.txt.
%
% Données (réelles) : custom_pitch_free_8V (G11), custom_yaw_free_8V (G12 et G22),
%                     step_pitch_10V (G21) ; validation 2 DDL : custom_free_2dof1/2.
% Sorties : rapport/figures/fig_08_identification.png, fig_B1_identification_G12_G21.png
% Versions : MATLAB R2025a, CONTSID 7.6.

clear; close all; clc;
root    = 'C:\Users\rayan\Desktop\Files\France\0 - Nancy\5A\S9\Control mini-projet';
proj    = fullfile(root, 'MiniProjetAeroCS');
dataDir = fullfile(proj, 'reponse_BO_experimentation');
outDir  = fullfile(root, 'rapport', 'figures');
logFile = fullfile(root, 'rapport', 'checks', 'fig_08_identification_log.txt');
addpath(fileparts(mfilename('fullpath')));          % frTicks.m
addpath(fullfile(proj, 'CONTSID7_6'));
addpath(fullfile(proj, 'LQR'));                     % quanser_aero_parameters.m (Jp, Jy, valeurs Quanser)
cd(dataDir);
if exist(logFile, 'file'), delete(logFile); end
diary(logFile);

tests = {
  'G11', 'custom_pitch_free_8V',    11, 4, 'TF2';
  'G12', 'custom_yaw_free_8V',      12, 4, 'TF2';
  'G21', 'step_pitch_10V',          11, 5, 'P1I';
  'G22', 'custom_yaw_free_8V',      12, 5, 'P1I';
};
TdMax = 0.1;
tPad  = 0.5;
valFiles = {'custom_free_2dof1', 'custom_free_2dof2'};
tValEnd  = 88;

%% 1) Identification des 4 fonctions de transfert (identique à identify_quanser_AERO.m)
nT = size(tests, 1);
R  = struct('name', {}, 'G', {}, 'fit', {}, 'par', {}, 't', {}, 'y', {}, 'ys', {}, 'u', {});
for i = 1:nT
    [t, u, y] = loadTest(tests{i,2}, tests{i,3}, tests{i,4}, tPad);
    Ts = t(2) - t(1);
    data = iddata(y, u, Ts, 'InterSample', 'zoh');
    switch tests{i,5}
        case 'TF2'
            M  = tfsrivc(data, 2, 0, 'TdMax', TdMax);
            num = M.Numerator;  den = M.Denominator;
            b0 = num(end) / den(1);
            a1 = den(2)   / den(1);
            a0 = den(3)   / den(1);
            par.b0 = b0;  par.a1 = a1;  par.a0 = a0;
            par.wn   = sqrt(a0);
            par.zeta = a1 / (2*sqrt(a0));
            par.Kdc  = b0 / a0;
            par.Td   = M.IODelay;
            G = tf(b0, [1 a1 a0], 'IODelay', M.IODelay);
        case 'P1I'
            M  = procsrivc(data, 'P1I');
            par.K = M.Kp;
            par.T = M.Tp1;
            G = tf(M.Kp, [M.Tp1 1 0]);
    end
    ys = lsim(G, u, t, 'zoh');
    R(i).name = tests{i,1};  R(i).G = G;  R(i).fit = fitPercent(y, ys);  R(i).par = par;
    R(i).t = t;  R(i).y = y;  R(i).ys = ys;  R(i).u = u;
    clear par
end
P11 = R(1).par;  P12 = R(2).par;  P21 = R(3).par;  P22 = R(4).par;

fprintf('===== Fits des 4 fonctions de transfert (recalcul) =====\n');
for i = 1:nT
    fprintf('  %s depuis %-22s (%s) : fit %.1f %%\n', tests{i,1}, tests{i,2}, tests{i,5}, R(i).fit);
end
fprintf('  G11 : wn = %.3f rad/s, zeta = %.3f, Td = %.3f s\n', P11.wn, P11.zeta, P11.Td);
fprintf('  G12 : wn = %.3f rad/s, zeta = %.3f, Td = %.3f s\n', P12.wn, P12.zeta, P12.Td);
fprintf('  G21 : T = %.2f s ; G22 : T = %.2f s\n', P21.T, P22.T);

%% 2) Paramètres physiques
quanser_aero_parameters;                       % Jp, Jy et valeurs Quanser
Q = struct('Ksp', Ksp, 'Dp', Dp, 'Kpp', Kpp, 'Kpy', Kpy, 'Dy', Dy, 'Kyy', Kyy, 'Kyp', Kyp);
P.Ksp = Jp * P11.a0;  P.Dp = Jp * P11.a1;  P.Kpp = Jp * P11.b0;
P.Kpy = P12.Kdc * P.Ksp;
P.Dy  = Jy / P22.T;   P.Kyy = P22.K * P.Dy;  P.Kyp = P21.K * P.Dy;

S = load(fullfile(dataDir, 'quanser_aero_parameters_identified.mat'));    % jeu retenu (lecture seule)
fprintf('\n===== Paramètres : recalcul contre fichier retenu (quanser_aero_parameters_identified.mat) =====\n');
fprintf('  Jp = %.6f, Jy = %.6f kg.m^2 (fichier : %.6f, %.6f)\n', Jp, Jy, S.Jp, S.Jy);
fprintf('  %-4s  %12s  %12s  %12s  %9s\n', '', 'Quanser', 'recalcul', 'fichier', 'ecart rel.');
nm = fieldnames(P);
for k = 1:numel(nm)
    fprintf('  %-4s  %12.6f  %12.6f  %12.6f  %8.2f %%\n', nm{k}, Q.(nm{k}), P.(nm{k}), S.(nm{k}), ...
            100*(P.(nm{k}) - S.(nm{k}))/S.(nm{k}));
end
fprintf('  fits du fichier : %s\n', mat2str(S.fits, 4));

%% 3) Validation sur les essais 2 DDL (non utilisés pour l'identification)
A = [0 0 1 0; 0 0 0 1; -P.Ksp/Jp 0 -P.Dp/Jp 0; 0 0 0 -P.Dy/Jy];
B = [0 0; 0 0; P.Kpp/Jp P.Kpy/Jp; P.Kyp/Jy P.Kyy/Jy];
C = eye(2,4);  D = zeros(2,2);
sysId = ss(A, B, C, D);
AQ = [0 0 1 0; 0 0 0 1; -Q.Ksp/Jp 0 -Q.Dp/Jp 0; 0 0 0 -Q.Dy/Jy];
BQ = [0 0; 0 0; Q.Kpp/Jp Q.Kpy/Jp; Q.Kyp/Jy Q.Kyy/Jy];
sysQ = ss(AQ, BQ, C, D);
fprintf('\n===== Validation sur les essais 2 DDL libres (0-%g s) =====\n', tValEnd);
for v = 1:numel(valFiles)
    Sv = load([valFiles{v} '.mat']);
    d0 = Sv.data0;
    k  = d0(1,:)' <= tValEnd;
    t  = d0(1,k)';  u = [d0(11,k)' d0(12,k)'];  y = [d0(4,k)' d0(5,k)'] * pi/180;
    yId = lsim(sysId, u, t, 'zoh');  yQ = lsim(sysQ, u, t, 'zoh');
    fprintf('  %s : fit tangage identifie %6.1f %% (Quanser %6.1f %%) | fit lacet identifie %6.1f %% (Quanser %6.1f %%)\n', ...
            valFiles{v}, fitPercent(y(:,1), yId(:,1)), fitPercent(y(:,1), yQ(:,1)), ...
            fitPercent(y(:,2), yId(:,2)), fitPercent(y(:,2), yQ(:,2)));
end

%% 4) Figures
ink = [0 0 0];  mod = [0.80 0.33 0.10];
plotPair(R([1 4]), tests([1 4],:), ink, mod, fullfile(outDir, 'fig_08_identification.png'), ...
         {'Tangage \theta (°)', 'Lacet \psi (°)'}, {'V_p', 'V_y'});
plotPair(R([2 3]), tests([2 3],:), ink, mod, fullfile(outDir, 'fig_B1_identification_G12_G21.png'), ...
         {'Tangage \theta (°)', 'Lacet \psi (°)'}, {'V_y', 'V_p'});
diary off;

%% ======================= FONCTIONS LOCALES =======================
function plotPair(Rp, tp, ink, mod, outFile, ylabels, inputs) %#ok<INUSD>
    fig = figure('Color', 'w', 'Units', 'centimeters', 'Position', [2 2 16 11.5]);
    set(fig, 'DefaultAxesFontSize', 10, 'DefaultLegendFontSize', 9);
    tl = tiledlayout(fig, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    for i = 1:2
        ax = nexttile(tl); hold(ax, 'on'); grid(ax, 'on'); ax.Box = 'on';
        r = Rp(i);
        h1 = plot(ax, r.t, r.y*180/pi,  '-',  'Color', ink, 'LineWidth', 1.4);
        h2 = plot(ax, r.t, r.ys*180/pi, '--', 'Color', mod, 'LineWidth', 1.6);
        if i == 1, hh = [h1 h2]; ax1 = ax; end
        ylabel(ax, ylabels{i});
        g = tp{i,1};                                   % 'G11', 'G12', 'G21' ou 'G22'
        switch g
            case 'G11', d = 'créneaux de ±8 V sur le moteur de tangage';  vin = 'V_p';  vout = '\theta';
            case 'G12', d = 'créneaux de ±8 V sur le moteur de lacet';    vin = 'V_y';  vout = '\theta';
            case 'G21', d = 'échelon de 10 V sur le moteur de tangage';   vin = 'V_p';  vout = '\psi';
            case 'G22', d = 'créneaux de ±8 V sur le moteur de lacet';    vin = 'V_y';  vout = '\psi';
        end
        title(ax, sprintf('G_{%s} (%s \\rightarrow %s), %s : fit %s %%', g(2:3), vin, vout, d, ...
              strrep(sprintf('%.1f', r.fit), '.', ',')), 'FontWeight', 'normal', 'FontSize', 9);
        if i == 2, xlabel(ax, 'Temps (s)'); end
        drawnow; frTicks(ax);
        if i == 1, ax.XTickLabel = []; end
    end
    lg = legend(ax1, hh, {'Mesure', 'Modèle identifié'}, 'Orientation', 'horizontal', 'Box', 'off');
    lg.Layout.Tile = 'south';
    exportgraphics(fig, outFile, 'Resolution', 300);
    fprintf('Écrit : %s\n', outFile);
end

function out = ternary(c, a, b)
    if c, out = a; else, out = b; end
end

function [t, u, y] = loadTest(fileName, inRow, outRow, tPad)
S  = load([fileName '.mat']);
d0 = S.data0;
Ts = d0(1,2) - d0(1,1);
nPad = round(tPad / Ts);
u = [zeros(nPad,1); d0(inRow,:)'];
y = [zeros(nPad,1); d0(outRow,:)'] * pi/180;
t = (0:length(u)-1)' * Ts;
end

function f = fitPercent(y, ys)
f = 100 * (1 - norm(y - ys) / norm(y - mean(y)));
end
