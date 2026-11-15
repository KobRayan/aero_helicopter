% fig_09_erreur_statique.m
% Figure 9 du rapport : erreur statique de tangage du LQR (4 réglages, consignes
% en rampe) mesurée, prédite par le modèle Quanser et prédite par le modèle identifié.
% Version réduite de LQR/tune_lqr_model_refined.m (série 'fixed3') : les
% fonctions de calcul (aeroModel, loadLQRTest, simulateLQR, computeIndicators,
% findMoves, moveSummary, coupling, fitPercent) sont reprises TELLES QUELLES ;
% seuls changent le pilotage, les libellés (français) et la mise en forme.
%
% POINT DE VIGILANCE (questions.md, Q3) : le script d'origine lit
% LQR/quanser_aero_parameters_identified_2.mat (12/10), pas le jeu retenu
% reponse_BO_experimentation/quanser_aero_parameters_identified.mat (15/10).
% Ce script calcule les deux et les imprime ; la figure utilise le JEU RETENU.
%
% Données (réelles) : LQR/aero_lqr_rsp_fixed3_sys1..4.mat. Simulation : modèle
% Quanser et modèle identifié, même gain K que sur la maquette (calculé sur le
% modèle Quanser), saturation ±20 V, Te = 2 ms.
% Journal : rapport/checks/fig_09_erreur_statique_log.txt
% Sortie  : rapport/figures/fig_09_erreur_statique.png
% Versions : MATLAB R2025a, Control System Toolbox.

clear; close all; clc;
root    = 'C:\Users\rayan\Desktop\Files\France\0 - Nancy\5A\S9\Control mini-projet';
proj    = fullfile(root, 'MiniProjetAeroCS');
lqrDir  = fullfile(proj, 'LQR');
outDir  = fullfile(root, 'rapport', 'figures');
logFile = fullfile(root, 'rapport', 'checks', 'fig_09_erreur_statique_log.txt');
addpath(fileparts(mfilename('fullpath')));            % frTicks.m
cd(lqrDir);
if exist(logFile, 'file'), delete(logFile); end
diary(logFile);

identSets = {fullfile(proj, 'reponse_BO_experimentation', 'quanser_aero_parameters_identified.mat'), ...
             fullfile(lqrDir, 'quanser_aero_parameters_identified_2.mat')};
setNamesTxt = {'jeu retenu (reponse_BO_experimentation, 15/10)', 'jeu lu par tune_lqr_model_refined.m (LQR/_2, 12/10)'};

series = 'fixed3';  satV = 20;  d2r = pi/180;
designs = {
    'Actuel',        diag([200 75 0 0]),   0.005*eye(2)
    'Plus doux',     diag([200 75 0 0]),   0.02*eye(2)
    'Plus amorti',   diag([200 75 20 10]), 0.005*eye(2)
    'Bryson',        diag([1/(5*d2r)^2, 1/(10*d2r)^2, 1/(60*d2r)^2, 1/(60*d2r)^2]), 0.005*eye(2)   % fixed3 : R = 0,005 (voir tune_lqr_model_refined.m l. 98-102)
    };
nD = size(designs, 1);

E = nan(nD, 3, numel(identSets));       % erreur statique de tangage, MOYENNE sur les paliers : mesure | Quanser | identifié, par jeu
Em = nan(nD, 3, numel(identSets));      % idem, MÉDIANE (computeMetrics.m, définition du § 3.1 du rapport)
F = nan(nD, 4, numel(identSets));       % fits : tangage Quanser | tangage identifié | lacet Quanser | lacet identifié
for s = 1:numel(identSets)
    [Aq, Bq] = aeroModel('quanser',    identSets{s});
    [Ai, Bi] = aeroModel('identified', identSets{s});
    for d = 1:nD
        fileName = sprintf('aero_lqr_rsp_%s_sys%d', series, d);
        [t, Vp, rP, yP, Vy, rY, yY] = loadLQRTest(fileName);
        Ts = t(2) - t(1);
        Q = designs{d,2};  R = designs{d,3};
        Kused = lqr(Aq, Bq, Q, R);
        sysQd = c2d(ss(Aq, Bq, eye(4), zeros(4,2)), Ts);
        sysId = c2d(ss(Ai, Bi, eye(4), zeros(4,2)), Ts);
        [YQ, UQ] = simulateLQR(sysQd.A, sysQd.B, Kused, t, rP*d2r, rY*d2r, satV);
        [YI, UI] = simulateLQR(sysId.A, sysId.B, Kused, t, rP*d2r, rY*d2r, satV);
        M1 = computeIndicators(t, rP, rY, yP,      yY,      Vp,      Vy,      satV);
        M2 = computeIndicators(t, rP, rY, YQ(:,1), YQ(:,2), UQ(:,1), UQ(:,2), satV);
        M3 = computeIndicators(t, rP, rY, YI(:,1), YI(:,2), UI(:,1), UI(:,2), satV);
        E(d,:,s) = [M1.essP, M2.essP, M3.essP];
        % AJOUT (rapport, 5/11/2026) : même indicateur avec la définition du § 3.1 (médiane, computeMetrics.m)
        c1 = computeMetrics(t, [rP rY], [yP yY],           [Vp Vy],           satV, 0.05);
        c2 = computeMetrics(t, [rP rY], [YQ(:,1) YQ(:,2)], [UQ(:,1) UQ(:,2)], satV, 0.05);
        c3 = computeMetrics(t, [rP rY], [YI(:,1) YI(:,2)], [UI(:,1) UI(:,2)], satV, 0.05);
        Em(d,:,s) = [c1.ess(1), c2.ess(1), c3.ess(1)];
        F(d,:,s) = [fitPercent(yP, YQ(:,1)), fitPercent(yP, YI(:,1)), fitPercent(yY, YQ(:,2)), fitPercent(yY, YI(:,2))];
    end
end

for s = 1:numel(identSets)
    fprintf('\n===== Erreur statique de tangage (deg), %s =====\n', setNamesTxt{s});
    fprintf('  %-12s %9s %9s %9s | %10s %10s\n', 'reglage', 'mesure', 'Quanser', 'identifie', 'ecart ident.', 'Quanser/mes.');
    for d = 1:nD
        fprintf('  %-12s %9.2f %9.2f %9.2f | %10.2f %10.2f\n', designs{d,1}, E(d,1,s), E(d,2,s), E(d,3,s), ...
                E(d,3,s) - E(d,1,s), E(d,2,s)/E(d,1,s));
    end
    fprintf('  ecart max |identifie - mesure| = %.2f deg ; sous-estimation moyenne du modele Quanser = %.0f %%\n', ...
            max(abs(E(:,3,s) - E(:,1,s))), 100*(1 - mean(E(:,2,s)./E(:,1,s))));
    fprintf('  fits (%%) tangage Quanser/identifie | lacet Quanser/identifie :\n');
    for d = 1:nD
        fprintf('  %-12s %6.1f / %6.1f | %6.1f / %6.1f\n', designs{d,1}, F(d,1,s), F(d,2,s), F(d,3,s), F(d,4,s));
    end
    fprintf('  --- MÉDIANE sur les mouvements (computeMetrics.m, définition du § 3.1 ; utilisée dans le rapport) ---\n');
    fprintf('  %-12s %9s %9s %9s | %10s %10s\n', 'reglage', 'mesure', 'Quanser', 'identifie', 'ecart ident.', 'Quanser/mes.');
    for d = 1:nD
        fprintf('  %-12s %9.2f %9.2f %9.2f | %10.2f %10.2f\n', designs{d,1}, Em(d,1,s), Em(d,2,s), Em(d,3,s), ...
                Em(d,3,s) - Em(d,1,s), Em(d,2,s)/Em(d,1,s));
    end
    fprintf('  ecart max |identifie - mesure| = %.2f deg ; sous-estimation moyenne du modele Quanser = %.0f %%\n', ...
            max(abs(Em(:,3,s) - Em(:,1,s))), 100*(1 - mean(Em(:,2,s)./Em(:,1,s))));
end

%% Figure (jeu retenu, s = 1)
fig = figure('Color', 'w', 'Units', 'centimeters', 'Position', [2 2 16 8]);
set(fig, 'DefaultAxesFontSize', 10, 'DefaultLegendFontSize', 9);
ax = axes(fig); hold(ax, 'on');
b = bar(ax, Em(:,:,1), 'grouped');          % médiane (§ 3.1) ; E = moyenne, dans le journal seulement
fills = [0.15 0.15 0.15; 0.65 0.65 0.65; 0.80 0.33 0.10];
for k = 1:3
    b(k).FaceColor = fills(k,:);  b(k).EdgeColor = 'k';
end
hatchTxt = {'', '', ''};                              %#ok<NASGU>
set(ax, 'XTick', 1:nD, 'XTickLabel', designs(:,1), 'Box', 'on');
ylabel(ax, 'Erreur statique de tangage (°)');
xlabel(ax, 'Réglage du LQR');
grid(ax, 'on');
legend(ax, 'Mesure sur la maquette', 'Modèle Quanser', 'Modèle identifié', 'Location', 'northoutside', 'Orientation', 'horizontal', 'Box', 'off');
for k = 1:3
    xt = b(k).XEndPoints;  yt = b(k).YEndPoints;
    for i = 1:nD
        text(ax, xt(i), yt(i) + 0.1, strrep(sprintf('%.1f', yt(i)), '.', ','), 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', 9);
    end
end
ylim(ax, [0 max(Em(:,:,1), [], 'all') + 0.9]);
drawnow; yt = ax.YTick; ax.YTick = yt;
ax.YTickLabel = arrayfun(@(v) strrep(num2str(v, '%g'), '.', ','), yt, 'UniformOutput', false);
exportgraphics(fig, fullfile(outDir, 'fig_09_erreur_statique.png'), 'Resolution', 300);
fprintf('\nÉcrit : %s\n', fullfile(outDir, 'fig_09_erreur_statique.png'));
diary off;

%% ======================= FONCTIONS LOCALES (reprises de tune_lqr_model_refined.m) =======================
function [A, B, P] = aeroModel(source, identFile)
quanser_aero_parameters;                % Jp, Jy (geometry) + default values
if strcmp(source, 'identified')
    S = load(identFile);                % overwrite with the identified values
    Ksp = S.Ksp; Dp = S.Dp; Kpp = S.Kpp; Kpy = S.Kpy;
    Dy  = S.Dy;  Kyy = S.Kyy; Kyp = S.Kyp;
end
A = [0 0 1 0;
     0 0 0 1;
    -Ksp/Jp 0 -Dp/Jp 0;
     0 0 0 -Dy/Jy];
B = [0 0;
     0 0;
     Kpp/Jp Kpy/Jp;
     Kyp/Jy Kyy/Jy];
P = struct('Ksp', Ksp, 'Dp', Dp, 'Kpp', Kpp, 'Kpy', Kpy, 'Dy', Dy, 'Kyy', Kyy, 'Kyp', Kyp);
end

function [t, Vp, rP, yP, Vy, rY, yY] = loadLQRTest(fileName)
S    = load([fileName '.mat']);
fn   = fieldnames(S);
data = S.(fn{1});
if size(data,1) > size(data,2), data = data'; end
t  = data(1,:)';  Vp = data(2,:)';  rP = data(3,:)';  yP = data(4,:)';
Vy = data(5,:)';  rY = data(6,:)';  yY = data(7,:)';
end

function [Y, U] = simulateLQR(Ad, Bd, K, t, refP, refY, satV)
N = length(t);
x = zeros(4,1);
Y = zeros(N,2);
U = zeros(N,2);
for k = 1:N
    xd = [refP(k); refY(k); 0; 0];
    u  = K*(xd - x);
    u  = min(max(u, -satV), satV);
    Y(k,:) = x(1:2)' * 180/pi;
    U(k,:) = u';
    x = Ad*x + Bd*u;
end
end

function M = computeIndicators(t, rP, rY, yP, yY, Vp, Vy, satV)
T = t(end) - t(1);
M.rmseP = sqrt(mean((rP - yP).^2));
M.rmseY = sqrt(mean((rY - yY).^2));
mvP = findMoves(t, rP);
mvY = findMoves(t, rY);
[M.trP, M.osP, M.essP] = moveSummary(t, yP, mvP);
[M.trY, M.osY, M.essY] = moveSummary(t, yY, mvY);
M.coupPbyY = coupling(yP, rP, mvY);
M.coupYbyP = coupling(yY, rY, mvP);
M.sat  = 100*mean(abs([Vp; Vy]) >= 0.99*satV);
M.actP = sum(abs(diff(Vp))) / T;
M.actY = sum(abs(diff(Vy))) / T;
end

function mv = findMoves(t, ref)
Ts       = t(2) - t(1);
minPlat  = 1.0;
minStep  = 1.0;
newVal   = [true; abs(diff(ref)) > 1e-9];
runStart = find(newVal);
runEnd   = [runStart(2:end) - 1; numel(ref)];
keep     = (runEnd - runStart + 1)*Ts >= minPlat;
pS = runStart(keep);  pE = runEnd(keep);
mv = zeros(0,4);
for j = 2:numel(pS)
    r0 = ref(pE(j-1));  r1 = ref(pS(j));
    if abs(r1 - r0) >= minStep
        mv(end+1,:) = [pE(j-1)+1, pE(j), r0, r1]; %#ok<AGROW>
    end
end
end

function [tr, OS, ess] = moveSummary(t, y, mv)
Ts = t(2) - t(1);
n  = size(mv,1);
trs = nan(n,1);  oss = nan(n,1);  es = nan(n,1);
for m = 1:n
    k0 = mv(m,1);  k1 = mv(m,2);  r1 = mv(m,4);
    pre = max(1, k0 - round(0.2/Ts)):max(1, k0 - 1);
    y0  = mean(y(pre));
    idx = (k0:k1)';
    z   = (y(idx) - y0) / (r1 - y0);
    tt  = t(idx) - t(k0);
    i10 = find(z >= 0.1, 1);  i90 = find(z >= 0.9, 1);
    if ~isempty(i10) && ~isempty(i90), trs(m) = tt(i90) - tt(i10); end
    oss(m) = max(0, 100*(max(z) - 1));
    es(m)  = abs(r1 - mean(y(idx(tt >= tt(end) - 1))));
end
tr  = mean(trs, 'omitnan');
OS  = mean(oss, 'omitnan');
ess = mean(es,  'omitnan');
end

function c = coupling(yOther, rOther, mv)
c = 0;
for m = 1:size(mv,1)
    idx = mv(m,1):mv(m,2);
    if all(rOther(idx) == rOther(idx(1)))
        c = max(c, max(abs(yOther(idx) - yOther(idx(1)))));
    end
end
end

function f = fitPercent(y, ys)
f = 100 * (1 - norm(y - ys) / norm(y - mean(y)));
end
