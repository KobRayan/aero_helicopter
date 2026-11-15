% figures_pd_presentation_AERO.m
% 15/10/2026
% FIGURES DE PRESENTATION : PD DECOUPLE et COMPARAISON AVEC LE LQR
%
% Ce script fait 4 figures propres (titres et axes en francais) et les
% enregistre en PNG (300 dpi) dans le dossier courant :
%   fig_pd_1_suivi.png        PD decouple sur l'AERO (test2) : consigne,
%                             mesure et simulation (modele avec frottement)
%   fig_pd_2_meme_rampe.png   LQR vs PD sur LE MEME MOUVEMENT : rampe de
%                             tangage +20 -> -20 deg, lacet immobile
%   fig_pd_3_indicateurs.png  erreurs statiques et couplages (barres)
%   fig_pd_4_couple.png       limite physique : couple de tangage obtenu /
%                             couple demande, sur les paliers
% Il affiche aussi tous les chiffres utilises dans la presentation.
%
% POURQUOI COMPARER SUR "LE MEME MOUVEMENT" ?
%   L'essai LQR (fixed3_sys1) et les essais PD n'ont pas la meme sequence
%   de consignes (LQR : tangage +/-20 deg toutes les 5 s, lacet en meme
%   temps ; PD : un seul axe bouge a la fois). Une RMSE sur tout l'essai
%   n'est donc PAS comparable (plus il y a de mouvements, plus la RMSE est
%   grande). On compare :
%     - les erreurs statiques sur les paliers (tangage +/-20, lacet +/-40)
%     - une rampe de tangage +20 -> -20 deg presente dans les deux essais,
%       avec le lacet immobile (LQR : t = 20 s, PD : t = 13 s)
%     - le debut d'une rampe de lacet de 80 deg avec le tangage immobile
%       (LQR : t = 12,5 s, seulement 2,5 s car le tangage bouge a 15 s ;
%        PD : t = 18 s)
%
% Fichiers necessaires dans le dossier courant :
%   aero_lqr_rsp_fixed3_sys1.mat, aero_dpd_rsp_test1.mat, aero_dpd_rsp_test2.mat,
%   aero_dpd_rsp_test3.mat, quanser_aero_parameters_identified.mat,
%   aero_friction_parameters.mat
% Control System Toolbox (c2d, ss) pour la simulation.

clear;
close all;
clc

%% 0) Reglages
lqrFile   = 'aero_lqr_rsp_fixed3_sys1';   % LQR "Current", consignes en rampe
pdFiles   = {'aero_dpd_rsp_test1', 'aero_dpd_rsp_test2', 'aero_dpd_rsp_test3'};
addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'reponse_BO_experimentation'))   % identified parameters (.mat)
identFile = 'quanser_aero_parameters_identified.mat';
fricFile  = 'aero_friction_parameters.mat';
cKypTests = [1 1 0.75];                    % cKyp utilise dans test1, test2, test3

% Mouvements choisis pour la comparaison (voir l'en-tete)
tPitchLQR = 20;    tPitchPD = 13;          % rampe de tangage +20 -> -20 deg (lacet immobile)
winPitch  = 5;                             % fenetre d'observation (s)
tYawLQR   = 12.5;  tYawPD   = 18;          % rampe de lacet de 80 deg (tangage immobile)
winYaw    = 2.5;                           % le tangage du LQR bouge a 15 s

% Specifications du PD (les memes que sur l'AERO)
PO = 7.5;  tp = 1.75;                      % depassement (%) et temps de pic (s)
satV = 20;                                 % saturation (V)
wf = 50;  zf = 0.8;                        % filtre de vitesse (State X / PIV)
TdP = 0.05;  TdY = 0.10;                   % retards moteur + helice (s)

% Couleurs (les memes dans toutes les figures)
cRef  = [0 0 0];
cLQR  = [0.11 0.62 0.45];                  % vert
cPD1  = [0.16 0.47 0.84];                  % bleu   : PD, cKyp = 1
cPD2  = [0.92 0.41 0.20];                  % orange : PD, cKyp = 0,75
cSim  = [0.60 0.60 0.60];                  % gris   : simulation
lw    = 2;                                 % epaisseur des courbes
fs    = 14;                                % taille de police

%% 1) Donnees
L  = loadLog(lqrFile);
PD = loadLog(pdFiles{1});                  % tableau de structures, un element par essai
for i = 2:numel(pdFiles)
    PD(i) = loadLog(pdFiles{i});
end
P  = load(identFile);
Fr = load(fricFile);
d2r = pi/180;

% Gains du PD (en couple) et decoupleur, comme dans setup_decoupled_pd_simulink.m
zeta = -log(PO/100) / sqrt(log(PO/100)^2 + pi^2);
wn   = pi / (tp*sqrt(1 - zeta^2));
kp_p = P.Jp*wn^2 - P.Ksp;   kd_p = 2*zeta*wn*P.Jp - P.Dp;
kp_y = P.Jy*wn^2;           kd_y = 2*zeta*wn*P.Jy - P.Dy;
Bt   = [P.Kpp P.Kpy; P.Kyp P.Kyy];

%% 2) Simulation du PD (cKyp = 1) sur la consigne de test2, modele "realiste"
% Modele = parametres avec frottement sec + retards + quantification des codeurs
C.Kt = [kp_p 0 kd_p 0; 0 kp_y 0 kd_y];     % couples = Kt*(xd - x) + Ft*r
C.Ft = [P.Ksp 0; 0 0];
C.W  = inv(Bt);
plant.Jp = P.Jp;  plant.Jy = P.Jy;
plant.Ksp = Fr.fricPitch.Ksp;  plant.Dp = Fr.fricPitch.Dp;  plant.FcP = Fr.fricPitch.Fc;
plant.Dy  = Fr.fricYaw.Dy;     plant.FcY = Fr.fricYaw.Fc;
plant.Bt  = [Fr.fricPitch.Kpp P.Kpy; P.Kyp Fr.fricYaw.Kyy];
plant.TdP = TdP;  plant.TdY = TdY;
S = PD(2);                                  % structure de test2, la mesure est remplacee par la simulation
filt = c2d(ss([0 1; -wf^2 -2*zf*wf], [0; wf^2], eye(2), zeros(2,1)), S.t(2) - S.t(1));
[Ysim, ~] = simulateLoop(C, plant, S.t, S.refP*d2r, S.refY*d2r, filt.A, filt.B, satV);
S.pitch = Ysim(:,1);
S.yaw   = Ysim(:,2);
fitP = 100*(1 - norm(PD(2).pitch - S.pitch) / norm(PD(2).pitch - mean(PD(2).pitch)));
fitY = 100*(1 - norm(PD(2).yaw   - S.yaw)   / norm(PD(2).yaw   - mean(PD(2).yaw)));

%% 3) Indicateurs
% Ordre des colonnes : PD simule | LQR mesure | PD mesure cKyp = 1 | PD mesure cKyp = 0,75
runs   = {S, L, PD(2), PD(3)};
names  = {'PD simulé', 'LQR mesuré', 'PD mesuré (cK_{yp} = 1)', 'PD mesuré (cK_{yp} = 0,75)'};
tPitch = [tPitchPD tPitchLQR tPitchPD tPitchPD];
tYaw   = [tYawPD   tYawLQR   tYawPD   tYawPD];
nR = numel(runs);
essP = zeros(1,nR);  essY = zeros(1,nR);  cplY = zeros(1,nR);  cplP = zeros(1,nR);
errEnd = zeros(1,nR);  delay = zeros(1,nR);
for i = 1:nR
    R = runs{i};
    essP(i) = mean(abs(plateauErrors(R.t, R.refP, R.pitch, 20)));   % paliers +/-20 deg
    essY(i) = mean(abs(plateauErrors(R.t, R.refY, R.yaw,   40)));   % paliers +/-40 deg
    % rampe de tangage +20 -> -20 : lacet deplace, erreur finale, retard a -16 deg
    k0  = find(R.t >= tPitch(i), 1);
    idx = (k0:find(R.t < tPitch(i) + winPitch, 1, 'last'))';
    cplY(i)   = max(abs(R.yaw(idx) - R.yaw(k0)));
    last      = idx(R.t(idx) >= tPitch(i) + winPitch - 1);
    errEnd(i) = mean(R.refP(last) - R.pitch(last));
    delay(i)  = R.t(idx(find(R.pitch(idx) <= -16, 1))) - R.t(idx(find(R.refP(idx) <= -16, 1)));
    % debut de la rampe de lacet de 80 deg : tangage deplace
    k0  = find(R.t >= tYaw(i), 1);
    idx = (k0:find(R.t < tYaw(i) + winYaw, 1, 'last'))';
    cplP(i) = max(abs(R.pitch(idx) - R.pitch(k0)));
end

fprintf('Simulation du PD (modele realiste) vs mesure test2 : fit tangage %.1f %%, lacet %.1f %%\n\n', fitP, fitY);
fprintf('%-28s | %8s %8s | %9s %8s | %9s %9s\n', '', 'e_ss th', 'e_ss psi', 'err fin', 'retard', 'psi<-th', 'th<-psi');
fprintf('%-28s | %8s %8s | %9s %8s | %9s %9s\n', '', '(deg)', '(deg)', '(deg)', '(s)', '(deg)', '(deg)');
for i = 1:nR
    fprintf('%-28s | %8.2f %8.2f | %+9.2f %8.2f | %9.2f %9.2f\n', strrep(names{i}, '_{yp}', 'yp'), ...
            essP(i), essY(i), errEnd(i), delay(i), cplY(i), cplP(i));
end
fprintf(['e_ss : moyenne des |erreurs| sur la derniere seconde des paliers (tangage +/-20, lacet +/-40)\n' ...
         'err fin, retard : rampe de tangage +20 -> -20 deg (retard = quand theta passe -16 deg, par rapport a la consigne)\n' ...
         'psi<-th : lacet deplace pendant cette rampe ; th<-psi : tangage deplace pendant 2,5 s de rampe de lacet\n\n']);

% Couple de tangage obtenu / demande, sur les paliers (PD seulement : paliers de 11 s, bien etablis)
% Au repos : couple obtenu = ressort Ksp*theta ; couple demande = Kpp*Vp + Kpy*Vy (modele identifie)
amps = [10 20 20];
ratio = zeros(1,3);  vHold = zeros(1,3);
for i = 1:3
    [~, lastIdx] = plateauErrors(PD(i).t, PD(i).refP, PD(i).pitch, amps(i));
    r = zeros(numel(lastIdx),1);  v = r;
    for k = 1:numel(lastIdx)
        j = lastIdx{k};
        tauReal = P.Ksp*mean(PD(i).pitch(j))*d2r;
        tauCmd  = P.Kpp*mean(PD(i).Vp(j)) + P.Kpy*mean(PD(i).Vy(j));
        r(k) = tauReal/tauCmd;
        v(k) = mean(abs(PD(i).Vp(j)));
    end
    ratio(i) = 100*mean(r);
    vHold(i) = mean(v);
    fprintf('%s (cKyp = %.2f) : couple obtenu = %.0f %% du couple demande, |Vp| de maintien = %.1f V\n', ...
            pdFiles{i}, cKypTests(i), ratio(i), vHold(i));
end
fprintf('\n');

%% 4) Figure 1 : PD decouple sur l'AERO (test2)
f1 = figure('Name', 'PD decouple : suivi', 'Color', 'w', 'Position', [100 100 1100 650]);
ax1 = subplot(2,1,1);  hold on;  grid on
plot(PD(2).t, PD(2).refP, '--', 'Color', cRef, 'LineWidth', 1.2)
plot(S.t,     S.pitch,    '-',  'Color', cSim, 'LineWidth', lw)
plot(PD(2).t, PD(2).pitch,'-',  'Color', cPD1, 'LineWidth', lw)
ylabel('Tangage \theta (°)')
title('PD découplé sur l''AERO : la mesure suit la simulation')
legend('Consigne', sprintf('Simulation (fit %.0f %%)', fitP), 'Mesure (test2)', 'Location', 'northeast')
ax2 = subplot(2,1,2);  hold on;  grid on
plot(PD(2).t, PD(2).refY, '--', 'Color', cRef, 'LineWidth', 1.2)
plot(S.t,     S.yaw,      '-',  'Color', cSim, 'LineWidth', lw)
plot(PD(2).t, PD(2).yaw,  '-',  'Color', cPD1, 'LineWidth', lw)
ylabel('Lacet \psi (°)')
xlabel('Temps (s)')
legend('Consigne', sprintf('Simulation (fit %.0f %%)', fitY), 'Mesure (test2)', 'Location', 'northeast')
linkaxes([ax1 ax2], 'x')
xlim(ax1, [0 PD(2).t(end)])
set([ax1 ax2], 'FontSize', fs)
exportgraphics(f1, 'fig_pd_1_suivi.png', 'Resolution', 300)

%% 5) Figure 2 : le meme mouvement, LQR vs PD
% Temps compte depuis le debut de la rampe de tangage (+20 -> -20 deg).
f2 = figure('Name', 'Meme rampe : LQR vs PD', 'Color', 'w', 'Position', [100 100 1100 700]);
bx1 = subplot(2,1,1);  hold on;  grid on
bx2 = subplot(2,1,2);  hold on;  grid on
show = {L, PD(2), PD(3)};
tS   = [tPitchLQR tPitchPD tPitchPD];
col  = {cLQR, cPD1, cPD2};
iRun = [2 3 4];                              % colonnes correspondantes dans les indicateurs
k0 = find(PD(2).t >= tPitchPD - 1, 1);
k1 = find(PD(2).t <  tPitchPD + winPitch, 1, 'last');
plot(bx1, PD(2).t(k0:k1) - tPitchPD, PD(2).refP(k0:k1), '--', 'Color', cRef, 'LineWidth', 1.2)
leg1 = {'Consigne'};  leg2 = {};
for i = 1:3
    R  = show{i};
    k0 = find(R.t >= tS(i) - 1, 1);
    k1 = find(R.t <  tS(i) + winPitch, 1, 'last');
    kS = find(R.t >= tS(i), 1);
    plot(bx1, R.t(k0:k1) - tS(i), R.pitch(k0:k1), 'Color', col{i}, 'LineWidth', lw)
    plot(bx2, R.t(k0:k1) - tS(i), R.yaw(k0:k1) - R.yaw(kS), 'Color', col{i}, 'LineWidth', lw)
    leg1{end+1} = sprintf('%s : erreur finale %.1f°', names{iRun(i)}, abs(errEnd(iRun(i)))); %#ok<SAGROW>
    leg2{end+1} = sprintf('%s : %.1f°', names{iRun(i)}, cplY(iRun(i)));                      %#ok<SAGROW>
end
yline(bx2, 0, 'k:');
ylabel(bx1, 'Tangage \theta (°)')
title(bx1, 'Même rampe de tangage +20° \rightarrow -20° (lacet immobile)')
legend(bx1, leg1, 'Location', 'northeast')
ylabel(bx2, '\Delta\psi (°)')
xlabel(bx2, 'Temps depuis le début de la rampe (s)')
title(bx2, 'Couplage : le lacet est déplacé par le mouvement de tangage')
legend(bx2, leg2, 'Location', 'southwest')
linkaxes([bx1 bx2], 'x')
xlim(bx1, [-1 winPitch])
set([bx1 bx2], 'FontSize', fs)
exportgraphics(f2, 'fig_pd_2_meme_rampe.png', 'Resolution', 300)

%% 6) Figure 3 : indicateurs
f3 = figure('Name', 'Indicateurs : LQR vs PD', 'Color', 'w', 'Position', [100 100 1300 480]);
barCol = [cSim; cLQR; cPD1; cPD2];
cx = gobjects(1,3);
cx(1) = subplot(1,3,1);
b = bar(1:nR, essP, 'FaceColor', 'flat');  b.CData = barCol;  grid on
title('Erreur statique de tangage')
ylabel('|erreur| sur les paliers ±20° (°)')
cx(2) = subplot(1,3,2);
b = bar(1:nR, essY, 'FaceColor', 'flat');  b.CData = barCol;  grid on
title('Erreur statique de lacet')
ylabel('|erreur| sur les paliers ±40° (°)')
cx(3) = subplot(1,3,3);
b = bar([cplY; cplP], 'FaceColor', 'flat');  grid on
for i = 1:nR
    b(i).CData = repmat(barCol(i,:), 2, 1);
end
set(gca, 'XTick', 1:2, 'XTickLabel', {'\psi déplacé par \theta', '\theta déplacé par \psi'})
title('Couplage (mouvements identiques)')
ylabel('déplacement maximal (°)')
legend(names, 'Location', 'northeast')
for i = 1:2
    set(cx(i), 'XTick', 1:nR, 'XTickLabel', {'PD sim.', 'LQR', 'PD cK=1', 'PD cK=0,75'})
end
set(cx, 'FontSize', fs-1)
exportgraphics(f3, 'fig_pd_3_indicateurs.png', 'Resolution', 300)

%% 7) Figure 4 : limite physique, couple obtenu / couple demande
f4 = figure('Name', 'Couple obtenu / demande', 'Color', 'w', 'Position', [100 100 900 500]);
dx = axes;  hold on;  grid on
b = bar(1:3, ratio, 0.55, 'FaceColor', 'flat');
b.CData = [cPD1; cPD1; cPD2];
yline(100, 'k--', 'Modèle linéaire identifié (100 %)', 'LineWidth', 1.5, ...
      'LabelHorizontalAlignment', 'left', 'FontSize', fs-2);
for i = 1:3
    text(i, ratio(i) + 3, sprintf('%.0f %%', ratio(i)), 'HorizontalAlignment', 'center', 'FontSize', fs)
end
set(dx, 'XTick', 1:3, 'XTickLabel', { ...
    sprintf('test1 : ±10°\\newline|V_p| ≈ %.0f V', vHold(1)), ...
    sprintf('test2 : ±20°\\newline|V_p| ≈ %.0f V', vHold(2)), ...
    sprintf('test3 : ±20°, cK_{yp} = 0,75\\newline|V_p| ≈ %.0f V', vHold(3))})
ylim([0 115])
ylabel('Couple de tangage obtenu / demandé (%)')
title('Les moteurs donnent moins de couple que prévu, surtout à basse tension')
set(dx, 'FontSize', fs)
exportgraphics(f4, 'fig_pd_4_couple.png', 'Resolution', 300)

fprintf('Figures enregistrees : fig_pd_1_suivi.png, fig_pd_2_meme_rampe.png, fig_pd_3_indicateurs.png, fig_pd_4_couple.png\n');

%% ======================= FONCTIONS LOCALES =======================

function R = loadLog(fileName)
% Charge un essai en boucle fermee (9 lignes, format des essais LQR / PD) :
% 1 temps | 2 Vp | 3 consigne tangage | 4 tangage | 5 Vy | 6 consigne lacet | 7 lacet | 8-9 helices
    S    = load(fileName);
    fn   = fieldnames(S);
    data = S.(fn{1});
    if size(data,1) > size(data,2)
        data = data';
    end
    R.t     = data(1,:)';
    R.Vp    = data(2,:)';
    R.refP  = data(3,:)';
    R.pitch = data(4,:)';
    R.Vy    = data(5,:)';
    R.refY  = data(6,:)';
    R.yaw   = data(7,:)';
end

function [err, lastIdx] = plateauErrors(t, ref, y, amp)
% Erreur (consigne - mesure) moyennee sur la derniere seconde de chaque
% palier ou la consigne vaut +/-amp. Les paliers de moins de 1,5 s sont ignores.
    Ts = t(2) - t(1);
    moving = [false; abs(diff(ref)) > 1e-9];
    kS = find(diff([false; moving]) == 1);      % debut de chaque rampe
    kE = find(diff([moving; false]) == -1);     % fin de chaque rampe
    err = [];  lastIdx = {};
    for i = 1:numel(kS)
        if i < numel(kS), k1 = kS(i+1) - 1; else, k1 = length(t); end
        if abs(abs(ref(k1)) - amp) > 1e-6 || (k1 - kE(i))*Ts < 1.5
            continue
        end
        last = (k1 - round(1/Ts) + 1):k1;
        err(end+1) = mean(ref(last) - y(last));   %#ok<AGROW>
        lastIdx{end+1} = last;                    %#ok<AGROW>
    end
end

function V = allocateVoltages(tau, W, satV)
% Couples -> tensions par le decoupleur, puis saturation +/-satV
    V = W*tau;
    V = min(max(V, -satV), satV);
end

function [Y, U] = simulateLoop(C, plant, t, refP, refY, Af, Bf, satV)
% Simulation de la boucle fermee (meme code que decoupled_pd_AERO.m) :
% correcteur a Ts, procede integre avec 4 sous-pas, frottement sec,
% retards moteur, quantification des codeurs, vitesses par le filtre State X.
% Renvoie les angles (deg) et les tensions (V).
    Ts   = t(2) - t(1);
    nSub = 4;
    h    = Ts/nSub;
    N    = length(t);
    nP   = round(plant.TdP/Ts);
    nY   = round(plant.TdY/Ts);
    qP   = 2*pi/2048;
    qY   = 2*pi/4096;
    th = 0;  ps = 0;  thd = 0;  psd = 0;
    zf = zeros(2,2);
    Y  = zeros(N,2);
    U  = zeros(N,2);
    for k = 1:N
        ym  = [round(th/qP)*qP; round(ps/qY)*qY];   % angles vus par les codeurs
        xh  = [ym; zf(2,1); zf(2,2)];
        zf  = Af*zf + Bf*ym';
        r   = [refP(k); refY(k)];
        tau = C.Kt*([r; 0; 0] - xh) + C.Ft*r;
        Vc  = allocateVoltages(tau, C.W, satV);
        U(k,:) = Vc';
        Y(k,:) = [th ps]*180/pi;
        Vd = [0; 0];
        if k - nP >= 1, Vd(1) = U(k-nP,1); end
        if k - nY >= 1, Vd(2) = U(k-nY,2); end
        T = plant.Bt*Vd;
        for s = 1:nSub
            [th, thd] = axisStep(th, thd, T(1), plant.Jp, plant.Ksp, plant.Dp, plant.FcP, h);
            [ps, psd] = axisStep(ps, psd, T(2), plant.Jy, 0,         plant.Dy, plant.FcY, h);
        end
    end
end

function [x, v] = axisStep(x, v, T, J, Ks, Dv, Fc, h)
% Un pas de  J*x'' = T - Ks*x - Dv*x' - Fc*sign(x')  avec frottement statique
    Td = T - Ks*x;
    if v == 0
        if abs(Td) <= Fc
            a = 0;
        else
            a = (Td - Fc*sign(Td)) / J;
        end
    else
        a = (Td - Dv*v - Fc*sign(v)) / J;
    end
    vNew = v + a*h;
    if v ~= 0 && sign(vNew) ~= sign(v)
        vNew = 0;
    end
    v = vNew;
    x = x + v*h;
end
