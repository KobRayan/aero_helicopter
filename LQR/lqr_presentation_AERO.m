% lqr_presentation_AERO.m
% 15/10/2026 - version allégée de tune_lqr_AERO.m pour la soutenance
%
% Compare, sur les MÊMES consignes qu'un essai mesuré sur l'AERO :
%   - trois réglages LQR simulés sur le modèle de Quanser : actuel, plus doux, Bryson
%   - l'essai mesuré
% Deux figures seulement :
%   1) les angles (consigne, trois simulations, mesure)
%   2) quatre indicateurs simples : erreur statique, dépassement, couplage,
%      agitation de la commande
%
% Fichiers nécessaires dans le même dossier :
%   quanser_aero_parameters.m, quanser_aero_state_space.m (sans "clear"),
%   computeMetrics.m, modelError.m, et le fichier de l'essai mesuré (dataFile)
%
% Le script tune_lqr_AERO.m complet (5 réglages, 4 figures) reste inchangé.

clear;
close all;
clc

%% 0) Réglages
dataFile    = 'aero_lqr_rsp_fixed3_sys1';   % essai mesuré (9 lignes), doit finir par sysN
                                            %   fixed2_sysN : échelons (tangage ±20°, lacet ±40°)
                                            %   fixed3_sysN : rampes à 20 °/s
                                            %   N = 1 actuel, 2 plus doux, 4 Bryson
satV        = 20;                           % saturation des tensions (V)
epsSettle   = 0.05;                         % bande de 5 % (utilisée par computeMetrics)
d2r         = pi/180;
exporterPNG = true;                         % enregistre les deux figures en PNG pour les diapositives

% Bryson : erreurs acceptables 5° (tangage) et 10° (lacet), vitesses 60 °/s, tension 15 V
Q_bryson = diag([1/(5*d2r)^2, 1/(10*d2r)^2, 1/(60*d2r)^2, 1/(60*d2r)^2]);
R_bryson = eye(2)/15^2;
% Sur les essais en rampe (fixed3), l'essai Bryson a tourné avec R = 0.005 :
% on simule la même chose pour pouvoir comparer à la mesure.
if contains(dataFile, 'fixed3')
    R_bryson = 0.005*eye(2);
end

% Réglages comparés : {nom, Q, R, numéro N de l'essai sysN correspondant}
% État : x = [theta, psi, theta_point, psi_point]  (rad, rad/s)
designs = {
    'LQR actuel',    diag([200 75 0 0]), 0.005*eye(2), 1
    'LQR plus doux', diag([200 75 0 0]), 0.02*eye(2),  2
    'LQR Bryson',    Q_bryson,           R_bryson,     4
    };
nD       = size(designs,1);
couleurs = [0     0.447 0.741      % bleu   : actuel
            0.850 0.325 0.098      % orange : plus doux
            0.494 0.184 0.556];    % violet : Bryson

% Réglage utilisé pour l'essai mesuré, lu à la fin du nom du fichier (..._sysN)
tok = regexp(dataFile, 'sys(\d+)$', 'tokens', 'once');
if isempty(tok)
    error('Le nom de dataFile doit finir par "sysN".')
end
iUsed = find([designs{:,4}] == str2double(tok{1}));
if isempty(iUsed)
    error('sys%s n''est pas dans la liste : seuls sys1, sys2 et sys4 sont montrés.', tok{1})
end

%% 1) Modèle de Quanser
quanser_aero_state_space;                   % lance quanser_aero_parameters, définit A, B, C, D

%% 2) Essai mesuré
S    = load(dataFile);
fn   = fieldnames(S);
data = S.(fn{1});
if size(data,1) > size(data,2)
    data = data';
end
t          = data(1,:)';
PitchVolt  = data(2,:)';
PitchRef   = data(3,:)';                    % deg
PitchAngle = data(4,:)';                    % deg
YawVolt    = data(5,:)';
YawRef     = data(6,:)';                    % deg
YawAngle   = data(7,:)';                    % deg
Ts         = t(2) - t(1);

ref  = [PitchRef,   YawRef];
yMes = [PitchAngle, YawAngle];
uMes = [PitchVolt,  YawVolt];

% Modèle discret (bloqueur d'ordre zéro), comme la boucle réelle à Ts = 2 ms
sysd = c2d(ss(A, B, C, D), Ts);

%% 3) Simulation des trois réglages, puis indicateurs
Ysim = cell(nD,1);
Usim = cell(nD,1);
for d = 1:nD
    K = lqr(A, B, designs{d,2}, designs{d,3});
    [Ysim{d}, Usim{d}] = simulateLQR(sysd.A, sysd.B, K, t, PitchRef*d2r, YawRef*d2r, satV);
    ind(d) = computeMetrics(t, ref, Ysim{d}, Usim{d}, satV, epsSettle);   %#ok<SAGROW>
end
ind(nD+1) = computeMetrics(t, ref, yMes, uMes, satV, epsSettle);         % la mesure en dernier

% Écart modèle / mesure pour le réglage qui a vraiment tourné sur l'AERO
err = modelError(Ysim{iUsed}, Usim{iUsed}, yMes, uMes);

noms = [designs(:,1); {sprintf('Mesuré (%s)', designs{iUsed,1})}];
nR   = numel(noms);

%% 4) Tableau dans la fenêtre de commande
fprintf('Essai : %s\n\n', dataFile);
fprintf('%-28s | %15s | %15s | %15s | %15s\n', '', 'Erreur stat.(°)', 'Dépassement(%)', ...
        'Couplage (°)', 'Agitation(V/s)');
fprintf('%-28s | %7s %7s | %7s %7s | %7s %7s | %7s %7s\n', '', ...
        'tang.', 'lacet', 'tang.', 'lacet', 'tang.', 'lacet', 'Vp', 'Vy');
for r = 1:nR
    fprintf('%-28s | %7.2f %7.2f | %7.1f %7.1f | %7.2f %7.2f | %7.1f %7.1f\n', noms{r}, ...
            ind(r).ess, ind(r).os, ind(r).cpl, ind(r).tv);
end
fprintf('\nFit du modèle (%s simulé / mesuré) : tangage %.1f %%, lacet %.1f %%\n', ...
        designs{iUsed,1}, err.fit(1), err.fit(2));

%% 5) Figure 1 : angles
fig1 = figure('Name', 'Angles', 'Color', 'w', 'Position', [100 100 1280 720]);

ax1 = subplot(2,1,1);  hold on;  grid on
plot(ax1, t, PitchRef, 'k--', 'LineWidth', 1)
for d = 1:nD
    plot(ax1, t, Ysim{d}(:,1), 'Color', couleurs(d,:), 'LineWidth', 1.3)
end
plot(ax1, t, PitchAngle, 'k', 'LineWidth', 2)
ylabel(ax1, '\theta (°)')
title(ax1, sprintf('Tangage : mesure (noir) et réglages simulés   (fit %s : %.0f %%)', ...
                   designs{iUsed,1}, err.fit(1)))
legend(ax1, [{'Consigne'}, designs(:,1)', {'Mesuré'}], 'Location', 'eastoutside')

ax2 = subplot(2,1,2);  hold on;  grid on
plot(ax2, t, YawRef, 'k--', 'LineWidth', 1)
for d = 1:nD
    plot(ax2, t, Ysim{d}(:,2), 'Color', couleurs(d,:), 'LineWidth', 1.3)
end
plot(ax2, t, YawAngle, 'k', 'LineWidth', 2)
ylabel(ax2, '\psi (°)')
xlabel(ax2, 'Temps (s)')
title(ax2, sprintf('Lacet : mesure (noir) et réglages simulés   (fit %s : %.0f %%)', ...
                   designs{iUsed,1}, err.fit(2)))
legend(ax2, [{'Consigne'}, designs(:,1)', {'Mesuré'}], 'Location', 'eastoutside')

linkaxes([ax1 ax2], 'x')
xlim(ax1, [t(1) t(end)])

%% 6) Figure 2 : quatre indicateurs
% Deux barres par essai : tangage (bleu) et lacet (orange). Plus bas = mieux.
indic = {
    'ess', 'Erreur statique (°)'
    'os',  'Dépassement (%)'
    'cpl', 'Couplage : axe déplacé par l''autre axe (°)'
    'tv',  'Agitation de la commande  \Sigma|\DeltaV| / T  (V/s)'
    };
fig2 = figure('Name', 'Indicateurs', 'Color', 'w', 'Position', [100 100 1280 720]);
for i = 1:size(indic,1)
    cx = subplot(2,2,i);  hold on;  grid on
    vals = vertcat(ind.(indic{i,1}));       % nR x 2 : [tangage, lacet]
    b = bar(cx, vals);
    b(1).FaceColor = [0.30 0.50 0.80];
    b(2).FaceColor = [0.95 0.60 0.20];
    % valeur écrite au-dessus de chaque barre (XEndPoints : MATLAB R2019b ou plus récent)
    for s = 1:2
        for r = 1:nR
            if ~isnan(vals(r,s))
                text(cx, b(s).XEndPoints(r), vals(r,s), sprintf('%.1f', vals(r,s)), ...
                     'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', 10)
            end
        end
    end
    xline(cx, nD + 0.5, 'k:');              % sépare les simulations de la mesure
    set(cx, 'XTick', 1:nR, 'XTickLabel', noms)
    xtickangle(cx, 15)
    title(cx, indic{i,2})
end
legend(cx, {'Tangage', 'Lacet'}, 'Location', 'best')

set(findall(0, 'Type', 'axes'), 'FontSize', 13)

%% 7) Export PNG pour les diapositives (exportgraphics : MATLAB R2020a ou plus récent)
if exporterPNG
    [~, nomEssai] = fileparts(dataFile);
    exportgraphics(fig1, ['pres_angles_' nomEssai '2.png'],      'Resolution', 200);
    exportgraphics(fig2, ['pres_indicateurs_' nomEssai '2.png'], 'Resolution', 200);
    fprintf('Figures enregistrées : pres_angles_%s.png, pres_indicateurs_%s.png\n', nomEssai, nomEssai);
end

%% ======================= FONCTION LOCALE =======================
% (dans un script, les fonctions doivent être à la fin du fichier)

function [Y, U] = simulateLQR(Ad, Bd, K, t, refP, refY, satV)
% Simulation discrète de la boucle fermée :
%   u(k)   = sat( K*(xd(k) - x(k)) ),   xd = [refP; refY; 0; 0]  (rad)
%   x(k+1) = Ad*x(k) + Bd*u(k)
% Renvoie les angles en degrés et les tensions en volts.
    N = length(t);
    x = zeros(4,1);
    Y = zeros(N,2);
    U = zeros(N,2);
    for k = 1:N
        xd = [refP(k); refY(k); 0; 0];
        u  = K*(xd - x);
        u  = min(max(u, -satV), satV);      % saturation ±satV
        Y(k,:) = x(1:2)' * 180/pi;
        U(k,:) = u';
        x = Ad*x + Bd*u;
    end
end
