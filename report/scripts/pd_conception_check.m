% pd_conception_check.m  (rapport/checks ; lecture seule des données du projet)
% Recalcule les grandeurs de conception du PD découplé cités au § 3.6 et à l'annexe D :
% ζ et ωn exacts, gains kp et kd (en N·m), matrice W, RGA, conditionnement de Bt,
% matrice équivalente Keq et pré-commande Nff, pôles en boucle fermée (PD découplé et
% PD indépendant sans découpleur, modèle linéaire identifié), effet de cKyp sur le dosage
% des deux moteurs. Aucune simulation temporelle : calcul direct à partir du jeu retenu.
% Données : reponse_BO_experimentation/quanser_aero_parameters_identified.mat (jeu retenu).
% Sortie : rapport/checks/pd_conception_check_log.txt   (MATLAB R2025a)
root = 'C:\Users\rayan\Desktop\Files\France\0 - Nancy\5A\S9\Control mini-projet';
logFile = fullfile(root, 'rapport', 'checks', 'pd_conception_check_log.txt');
if exist(logFile, 'file'), delete(logFile); end
diary(logFile);
P = load(fullfile(root, 'MiniProjetAeroCS', 'reponse_BO_experimentation', 'quanser_aero_parameters_identified.mat'));
Jp = P.Jp; Jy = P.Jy; Ksp = P.Ksp; Dp = P.Dp; Kpp = P.Kpp; Kpy = P.Kpy; Dy = P.Dy; Kyy = P.Kyy; Kyp = P.Kyp;

PO = 7.5; tp = 1.75;                           % spécification de quanser_aero_pd_design_Ksp.m
zeta = -log(PO/100) / sqrt(log(PO/100)^2 + pi^2);
wn   = pi / (tp*sqrt(1 - zeta^2));
fprintf('zeta = %.4f, wn = %.4f rad/s, poles visés = %.3f +/- %.3fj\n', zeta, wn, -zeta*wn, wn*sqrt(1-zeta^2));

kp_p = Jp*wn^2 - Ksp;   kd_p = 2*zeta*wn*Jp - Dp;
kp_y = Jy*wn^2;         kd_y = 2*zeta*wn*Jy - Dy;
fprintf('kp_p = %.4f  kd_p = %.4f  kp_y = %.4f  kd_y = %.4f   (N.m/rad, N.m.s/rad)\n', kp_p, kd_p, kp_y, kd_y);

Bt  = [Kpp Kpy; Kyp Kyy];
W   = inv(Bt);
RGA = Bt .* inv(Bt)';
fprintf('Bt = [%.6f %.6f; %.6f %.6f]\n', Bt');
fprintf('W  = [%.0f %.0f; %.0f %.0f] V/(N.m)\n', W');
fprintf('RGA lambda11 = %.3f  (lambda12 = %.3f), cond(Bt) = %.2f\n', RGA(1,1), RGA(1,2), cond(Bt));

Ksc = [kp_p 0 kd_p 0; 0 kp_y 0 kd_y];
Keq = W*Ksc;   Nff = W*[Ksp 0; 0 0];
fprintf('Keq = [%.1f %.1f %.1f %.1f; %.1f %.1f %.1f %.1f] V/rad, V.s/rad\n', Keq');
fprintf('Nff = [%.1f %.1f; %.1f %.1f] V/rad\n', Nff');

A = [0 0 1 0; 0 0 0 1; -Ksp/Jp 0 -Dp/Jp 0; 0 0 0 -Dy/Jy];
B = [0 0; 0 0; Kpp/Jp Kpy/Jp; Kyp/Jy Kyy/Jy];
pDec = eig(A - B*Keq);
Kind = [kp_p/Kpp 0 kd_p/Kpp 0; 0 kp_y/Kyy 0 kd_y/Kyy];            % formules Quanser, sans découpleur
pInd = eig(A - B*Kind);
fprintf('Poles PD découplé (modèle identifié)  : %s\n', mat2str(round(pDec,2).'));
fprintf('Gains PD indépendant (V/rad, V.s/rad) : kp_p %.1f kd_p %.1f kp_y %.1f kd_y %.1f\n', Kind(1,1), Kind(1,3), Kind(2,2), Kind(2,4));
fprintf('Poles PD indépendant (modèle identifié): %s  -> zeta equivalent du couple dominant = %.2f\n', mat2str(round(pInd,2).'), ...
        min(abs(real(pInd(imag(pInd)>0)))./abs(pInd(imag(pInd)>0))));

fprintf('\nSans anticipation : e_ss = Ksp/(Ksp+kp_p) = %.1f %% de la consigne (%.1f deg a 20 deg)\n', 100*Ksp/(Ksp+kp_p), 20*Ksp/(Ksp+kp_p));
fprintf('Couple de maintien a 20 deg : Ksp*20deg = %.2f mN.m\n', 1e3*Ksp*20*pi/180);

fprintf('\ncKyp : dosage Vy/Vp pour un couple de tangage pur, et tensions pour tenir 20 deg\n');
for c = [1 0.75 0.5]
    Wc = inv([Kpp Kpy; c*Kyp Kyy]);
    V  = Wc*[Ksp*20*pi/180; 0];
    fprintf('  cKyp = %.2f : Vy/Vp = %.2f ; Vp = %.1f V, Vy = %.1f V\n', c, Wc(2,1)/Wc(1,1), V(1), V(2));
end
fprintf('\nLQR actuel, modèle identifié : pôles de A - B*K = %s\n', mat2str(round(eig(A - B*[98.21 -103.06 32.26 -29.08; 156.35 66.16 45.51 17.11]),2).'));
fprintf('Plateau du modèle avec frottement, lacet, V = 10 V : %.1f deg/s ((Kyy V - Fc)/Dy avec le jeu frottement)\n', ...
        ((0.000710208*10 - 0.00414507)/0.0037153)*180/pi);
diary off;
