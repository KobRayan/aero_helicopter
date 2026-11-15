% Lecture seule : paramètres du tableau du § 2.5 (Quanser / identifié / avec frottement), valeurs exactes.
root = 'C:\Users\rayan\Desktop\Files\France\0 - Nancy\5A\S9\Control mini-projet\MiniProjetAeroCS\reponse_BO_experimentation';
diary(fullfile(fileparts(mfilename('fullpath')),'params_tableau_25_log.txt'));
P = load(fullfile(root,'quanser_aero_parameters_identified.mat'));
F = load(fullfile(root,'aero_friction_parameters.mat'));
disp('--- identifie ---'); for n={'Jp','Jy','Ksp','Dp','Kpp','Kpy','Dy','Kyy','Kyp'}, fprintf('%s = %.6g\n',n{1},P.(n{1})); end
disp('--- frottement (struct) ---'); disp(F.fricPitch); disp(F.fricYaw);
wn=sqrt(P.Ksp/P.Jp); fprintf('wn identifie = %.3f ; zeta = %.3f\n',wn,P.Dp/(2*P.Jp*wn));
diary off;

%% Grandeurs de lecture du tableau (ajout) : pulsation propre, amortissement, gains statiques, rapports identifié / Quanser
diary(fullfile(fileparts(mfilename('fullpath')),'params_tableau_25_log.txt'));
Q = struct('Jp',0.02189,'Jy',0.02199,'Ksp',0.037463,'Dp',0.0071116,'Kpp',0.0011,'Kpy',0.0021,'Dy',0.0220,'Kyy',0.0022,'Kyp',-0.0027);
% valeurs Quanser recopiées de LQR/quanser_aero_parameters.m (Jp, Jy recalculés plus haut : 0.02189, 0.02199)
Id = P;  Fr = struct('Ksp',F.fricPitch.Ksp,'Dp',F.fricPitch.Dp,'Kpp',F.fricPitch.Kpp,'Dy',F.fricYaw.Dy,'Kyy',F.fricYaw.Kyy,'Jp',P.Jp,'Jy',P.Jy);
sets = {'Quanser',Q; 'Identifie',Id; 'Frottement',Fr};
for k=1:3
    s = sets{k,2};
    wn = sqrt(s.Ksp/s.Jp); ze = s.Dp/(2*sqrt(s.Ksp*s.Jp));
    fprintf('%-10s : wn = %.3f rad/s, zeta = %.3f, Kpp/Ksp = %.3f deg/V, Jy/Dy = %.2f s, Kyy/Dy = %.2f deg/s/V\n', ...
        sets{k,1}, wn, ze, rad2deg(s.Kpp/s.Ksp), s.Jy/s.Dy, rad2deg(s.Kyy/s.Dy));
end
fprintf('Rapports identifie/Quanser : Kpp %.2f, Kpy %.2f, Kyy %.2f, |Kyp| %.2f, Ksp %.2f, Dp %.2f, Dy %.2f\n', ...
    Id.Kpp/Q.Kpp, Id.Kpy/Q.Kpy, Id.Kyy/Q.Kyy, abs(Id.Kyp)/abs(Q.Kyp), Id.Ksp/Q.Ksp, Id.Dp/Q.Dp, Id.Dy/Q.Dy);
fprintf('Kpy/Kpp : Quanser %.2f, identifie %.2f\n', Q.Kpy/Q.Kpp, Id.Kpy/Id.Kpp);
% gains PD recalculés avec le jeu identifie (zeta, wn visés du plan: 0,64 ; 2,33)
for wnv = [2.33]
  zv = 0.64;
  fprintf('PD pitch (jeu identifie) : kp = %.4f, kd = %.4f ; PD yaw : kp = %.4f, kd = %.4f\n', ...
      Id.Jp*wnv^2-Id.Ksp, 2*zv*wnv*Id.Jp-Id.Dp, Id.Jy*wnv^2, 2*zv*wnv*Id.Jy-Id.Dy);
end
diary off;
