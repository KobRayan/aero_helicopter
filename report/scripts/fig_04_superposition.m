% fig_04_superposition.m
% Figure 4 du rapport : vérification de la superposition à 10 V, en français.
% Copie adaptée de reponse_BO_experimentation/step_input_comparasion.m
% (le script appelé plot_all_steps_AERO.m dans le plan). Le calcul est
% inchangé (fonction loadAero, somme des essais seuls interpolée) ; changent :
% les libellés (français, sans noms de fichiers), la mise en forme (légende
% hors du tracé, traits différents pour le noir et blanc, 16 cm, 300 dpi),
% et deux choix d'affichage :
%   - pas de panneau « tensions » (Vp = Vy = 10 V, constantes : dit en légende) ;
%   - pas de panneau « angle de lacet » (il croît sans borne, 1 900° pour la somme :
%     la vitesse de lacet porte la même information) ;
%   - la vitesse de lacet affichée est la vitesse de loadAero (dérivée de l'angle du
%     codeur, lissée 0,1 s) suivie d'une médiane glissante de 1 s, pour retirer les
%     pointes de la dérivée du codeur. La valeur numérique citée dans le texte vient
%     de loadAero, sans médiane.
%
% Données (réelles) : step_both_10V10V_recovered.mat (10 V + 10 V, essai du
% 30/09 à 08:39, rétabli par Rayan le 2/11 ; step_both_10V10V.mat du dossier a été
% réécrit à 13:03 avec un essai à 5 V + 5 V), step_pitch_10V.mat, step_yaw_10V.mat.
% Sortie : rapport/figures/fig_04_superposition.png ; journal : checks/fig_04_superposition_log.txt
% Lancement : matlab -batch "cd('<chemin figure_scripts>'); fig_04_superposition"

clear; close all; clc;
root    = 'C:\Users\rayan\Desktop\Files\France\0 - Nancy\5A\S9\Control mini-projet';
dataDir = fullfile(root, 'MiniProjetAeroCS', 'reponse_BO_experimentation');
outDir  = fullfile(root, 'rapport', 'figures');
logFile = fullfile(root, 'rapport', 'checks', 'fig_04_superposition_log.txt');
addpath(fileparts(mfilename('fullpath')));      % frTicks.m
cd(dataDir);
if exist(logFile, 'file'), delete(logFile); end
diary(logFile);

B  = loadAero('step_both_10V10V_recovered');    % deux moteurs, 10 V + 10 V
Pr = loadAero('step_pitch_10V');                % tangage seul, 10 V
Yr = loadAero('step_yaw_10V');                  % lacet seul, 10 V

sumTheta = interp1(Pr.t, Pr.theta, B.t) + interp1(Yr.t, Yr.theta, B.t);
sumDpsi  = interp1(Pr.t, Pr.dpsi,  B.t) + interp1(Yr.t, Yr.dpsi,  B.t);

fprintf('Essai combine : duree %.1f s, max |Vp - Vy| = %.3f V, Vp moyen %.2f V, Vy moyen %.2f V\n', ...
        B.t(end) - B.t(1), max(abs(B.Vp - B.Vy)), mean(B.Vp), mean(B.Vy));
fprintf('Fin d''essai (moyenne des 5 dernieres s) : theta %.2f deg (deux moteurs) contre %.2f deg (somme) ; dpsi/dt %.2f contre %.2f deg/s\n', ...
        B.thetaEnd, mean(sumTheta(B.t > B.t(end) - 5), 'omitnan'), B.dpsiEnd, mean(sumDpsi(B.t > B.t(end) - 5), 'omitnan'));
fprintf('Pic de theta : %.1f deg (deux moteurs) contre %.1f deg (somme)\n', max(B.theta), max(sumTheta));
for T = [1 2 3 5]
    k = B.t >= 0 & B.t <= T;
    fprintf('Ecart max sur theta pour 0 <= t <= %g s : %.2f deg\n', T, max(abs(B.theta(k) - sumTheta(k))));
end
diary off;

% Affichage : médiane glissante de 1 s sur les vitesses de lacet
Ts = B.t(2) - B.t(1);  nW = round(1/Ts);
dB = movmedian(B.dpsi, nW);  dS = movmedian(sumDpsi, nW, 'omitnan');

fig = figure('Color', 'w', 'Units', 'centimeters', 'Position', [2 2 16 11.5]);
set(fig, 'DefaultAxesFontSize', 10, 'DefaultLegendFontSize', 9);
tl = tiledlayout(fig, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

ax1 = nexttile(tl); hold(ax1, 'on'); grid(ax1, 'on'); ax1.Box = 'on';
h1 = plot(ax1, B.t, B.theta,  'k-',  'LineWidth', 1.6);
h2 = plot(ax1, B.t, sumTheta, '--', 'Color', [0.80 0.33 0.10], 'LineWidth', 1.6);
ylabel(ax1, 'Tangage \theta (°)');
title(ax1, 'Angle de tangage', 'FontWeight', 'normal');

ax2 = nexttile(tl); hold(ax2, 'on'); grid(ax2, 'on'); ax2.Box = 'on';
plot(ax2, B.t, dB, 'k-',  'LineWidth', 1.6);
plot(ax2, B.t, dS, '--', 'Color', [0.80 0.33 0.10], 'LineWidth', 1.6);
ylabel(ax2, 'Vitesse de lacet (°/s)');
xlabel(ax2, 'Temps depuis l''échelon (s)');
title(ax2, 'Vitesse de lacet', 'FontWeight', 'normal');

linkaxes([ax1 ax2], 'x');  xlim(ax1, [-1 B.t(end)]);
drawnow; frTicks(ax1); frTicks(ax2); ax1.XTickLabel = [];
lg = legend(ax1, [h1 h2], {'Deux moteurs à 10 V (mesure)', 'Somme des deux essais à un seul moteur'}, 'Orientation', 'horizontal', 'Box', 'off');
lg.Layout.Tile = 'south';
exportgraphics(fig, fullfile(outDir, 'fig_04_superposition.png'), 'Resolution', 300);
fprintf('Écrit : %s\n', fullfile(outDir, 'fig_04_superposition.png'));

%% ======================= FONCTION LOCALE (reprise de step_input_comparasion.m) =======================
function d = loadAero(fileName)
    S = load(fileName);
    D = S.data0;
    if size(D,1) ~= 12
        error('%s has %d rows instead of 12.', fileName, size(D,1));
    end
    t  = D(1,:)';
    Ts = t(2) - t(1);
    d.name  = fileName;
    d.Vp    = D(11,:)';
    d.Vy    = D(12,:)';
    d.theta = D(4,:)' - D(4,1);
    d.psi   = D(5,:)' - D(5,1);
    V  = abs(d.Vp) + abs(d.Vy);
    k0 = find(V > 0.5*max(V), 1);
    if isempty(k0)
        warning('%s: no voltage step found, time not shifted.', fileName);
        k0 = 1;
    end
    d.t = t - t(k0);
    win      = round(0.1/Ts);
    d.dtheta = gradient(smoothdata(d.theta, 'gaussian', win), Ts);
    d.dpsi   = gradient(smoothdata(d.psi,   'gaussian', win), Ts);
    last       = d.t > d.t(end) - 5;
    d.VpEnd    = mean(d.Vp(last));
    d.VyEnd    = mean(d.Vy(last));
    d.thetaEnd = mean(d.theta(last));
    d.dpsiEnd  = mean(d.dpsi(last));
end
