% homogeneite_lacet.m  (rapport/checks ; lecture seule des données du projet)
% Test d'homogénéité du lacet (§ 4.2, figures 10, 11a, 11b, A) : pour chaque impulsion 0 -> X V sur le
% moteur de lacet (Vp = 0, deux axes libres), vitesse de lacet atteinte après 5 s, gain par volt K,
% vitesse d'hélice par volt. Remplace le script d'origine (plot_yaw_linearity_report_AERO.m, absent
% du dossier du projet : il n'existait que dans claude/) pour les chiffres cités dans le rapport.
% Définitions : vitesse = moyenne sur la dernière demi-seconde de l'impulsion ; K = vitesse / V (°/s/V) ;
% ω/V = vitesse d'hélice moyenne sur la même fenêtre divisée par V (rad/s/V, tachymètre, ligne 7).
% Données (réelles) : Modele_estimation/custom_yaw_0to20.mat, custom_yaw_0to12-12.mat (data0, 12 lignes).
% Coupure de custom_yaw_0to20 au premier saut de plus de 10° en un échantillon (déconnexion à 17 V).
% Sortie : rapport/checks/homogeneite_lacet_log.txt   (MATLAB R2025a)
root = 'C:\Users\rayan\Desktop\Files\France\0 - Nancy\5A\S9\Control mini-projet';
logFile = fullfile(root, 'rapport', 'checks', 'homogeneite_lacet_log.txt');
if exist(logFile, 'file'), delete(logFile); end
diary(logFile);
dir0 = fullfile(root, 'MiniProjetAeroCS', 'Modele_estimation');
P = load(fullfile(root, 'MiniProjetAeroCS', 'reponse_BO_experimentation', 'quanser_aero_parameters_identified.mat'));
Kss = P.Kyy/P.Dy*180/pi;  Tm = P.Jy/P.Dy;
fprintf('Modèle linéaire identifié : K en régime établi = %.2f deg/s/V, constante de temps = %.2f s, K a 5 s = %.2f deg/s/V\n\n', ...
        Kss, Tm, Kss*(1 - exp(-5/Tm)));
files = {'custom_yaw_0to20', 'custom_yaw_0to12-12'};
R = [];                                            % V | vitesse | K | omega/V | fichier
for f = 1:numel(files)
    S = load(fullfile(dir0, [files{f} '.mat']));  d = S.data0;
    t = d(1,:)';  psi = d(5,:)';  om = d(7,:)';  vy = d(12,:)';
    jump = find(abs(diff(psi)) > 10, 1);
    if ~isempty(jump)
        fprintf('%s : saut de %.0f deg en un échantillon à t = %.1f s (Vy = %.1f V) -> données coupées\n', files{f}, abs(psi(jump+1)-psi(jump)), t(jump), vy(jump));
        t = t(1:jump);  psi = psi(1:jump);  om = om(1:jump);  vy = vy(1:jump);
    end
    Ts = t(2) - t(1);
    lvl = round(vy);                               % niveau entier de tension
    starts = find(diff([0; lvl]) ~= 0 & lvl ~= 0);
    for k = 1:numel(starts)
        i0 = starts(k);  V = lvl(i0);
        i1 = find(lvl(i0:end) ~= V, 1) + i0 - 2;  if isempty(i1), i1 = numel(lvl); end
        if (i1 - i0)*Ts < 4.5, continue, end
        w  = (i1 - round(0.5/Ts)):i1;
        vel = (psi(i1) - psi(w(1))) / (t(i1) - t(w(1)));        % deg/s, moyenne sur 0,5 s
        R(end+1,:) = [V, vel, vel/V, mean(om(w))/V, f]; %#ok<AGROW>
    end
end
[~, ord] = sortrows(R(:,[5 1]));  R = R(ord,:);
for f = 1:numel(files)
    fprintf('\n===== %s =====\n   V (V) | vitesse apres 5 s (deg/s) | K (deg/s/V) | omega/V (rad/s/V)\n', files{f});
    X = R(R(:,5) == f,:);
    for k = 1:size(X,1), fprintf('  %+5d | %10.1f | %8.2f | %8.1f\n', X(k,1), X(k,2), X(k,3), X(k,4)); end
end
% Caractéristique statique : vitesse = a (V^2 - V0^2) pour V > 0, niveaux >= 8 V, essai 0to20
X = R(R(:,5) == 1 & R(:,1) >= 8,:);
c = polyfit(X(:,1).^2, X(:,2), 1);
fprintf('\nAjustement vitesse = a (V^2 - V0^2), V = 8 à %d V (custom_yaw_0to20, une impulsion par niveau) : a = %.2f deg/s/V^2, V0 = %.1f V\n', max(X(:,1)), c(1), sqrt(max(0, -c(2)/c(1))));
% Rapport au modèle linéaire à 16 V
k16 = R(R(:,5) == 1 & R(:,1) == 16, 2);
if ~isempty(k16), fprintf('A 16 V : %.0f deg/s mesurés contre %.0f deg/s pour le modèle linéaire (rapport %.1f)\n', k16, Kss*(1-exp(-5/Tm))*16, k16/(Kss*(1-exp(-5/Tm))*16)); end
% Moteur seul : vitesse d'hélice par volt
Z = R(abs(R(:,1)) >= 2,:);
fprintf('omega/V, |V| >= 2 V : %.1f à %.1f rad/s/V (moyenne %.1f) ; V>0 : %.1f ; V<0 : %.1f\n', min(abs(Z(:,4))), max(abs(Z(:,4))), mean(abs(Z(:,4))), mean(Z(Z(:,1)>0,4)), mean(abs(Z(Z(:,1)<0,4))));
% Dissymétrie des signes (essai en ±V) : rapport |vitesse(-V)| / vitesse(+V)
X2 = R(R(:,5) == 2,:);
Vs = unique(abs(X2(:,1)));  Vs = Vs(Vs >= 8);
for v = Vs'
    a = mean(X2(X2(:,1) == v,2));  b = mean(X2(X2(:,1) == -v,2));
    if ~isnan(a) && ~isnan(b), fprintf('  +/-%2d V : +V %.1f deg/s ; -V %.1f deg/s ; rapport %.2f\n', v, a, b, abs(b)/a); end
end
diary off;
