% pd_ess_mediane.m  (rapport/checks, lecture seule des données du projet)
% Erreur statique de tangage et de lacet du LQR de référence et des 3 essais du PD,
% avec la définition du § 3.1 (Pierre) : médiane sur les mouvements de computeMetrics.m.
% Sert au § 4.5 : le plan donnait des moyennes (2,74 / 1,34 / 1,51°), le § 3.1 impose la médiane.
% Données (réelles) : LQR/aero_lqr_rsp_fixed3_sys1.mat, Modele_estimation/aero_dpd_rsp_test1..3.mat
% Sortie : rapport/checks/pd_ess_mediane_log.txt   (MATLAB R2025a)
root = 'C:\Users\rayan\Desktop\Files\France\0 - Nancy\5A\S9\Control mini-projet';
proj = fullfile(root, 'MiniProjetAeroCS');
addpath(fullfile(proj, 'LQR'));
logFile = fullfile(root, 'rapport', 'checks', 'pd_ess_mediane_log.txt');
if exist(logFile, 'file'), delete(logFile); end
diary(logFile);
files = { fullfile(proj, 'LQR', 'aero_lqr_rsp_fixed3_sys1.mat'), ...
          fullfile(proj, 'Modele_estimation', 'aero_dpd_rsp_test1.mat'), ...
          fullfile(proj, 'Modele_estimation', 'aero_dpd_rsp_test2.mat'), ...
          fullfile(proj, 'Modele_estimation', 'aero_dpd_rsp_test3.mat') };
names = {'LQR fixed3_sys1', 'PD test1 (cKyp=1)', 'PD test2 (cKyp=1)', 'PD test3 (cKyp=0.75)'};
fprintf('%-22s | ess med [th psi] | ess moy [th psi] | cpl [th<-psi psi<-th] | sat [Vp Vy] %% | tv V/s | duree s\n', '');
for i = 1:numel(files)
    S = load(files{i});  fn = fieldnames(S);  d = S.(fn{1});
    if size(d,1) > size(d,2), d = d'; end
    t = d(1,:)';  u = [d(2,:)' d(5,:)'];  ref = [d(3,:)' d(6,:)'];  y = [d(4,:)' d(7,:)'];
    m = computeMetrics(t, ref, y, u, 20, 0.05);
    % moyenne sur les paliers (définition de figures_pd_presentation_AERO.m), pour comparaison
    ep = plat(t, ref(:,1), y(:,1), max(abs(ref(:,1)))); ey = plat(t, ref(:,2), y(:,2), max(abs(ref(:,2))));
    fprintf('%-22s | %5.2f %5.2f | %5.2f %5.2f | %5.2f %5.2f | %4.1f %4.1f | %5.1f | %4.1f\n', names{i}, ...
        m.ess(1), m.ess(2), mean(abs(ep)), mean(abs(ey)), m.cpl(1), m.cpl(2), m.sat(1), m.sat(2), sum(m.tv), t(end));
    fprintf('    erreurs de tangage par palier (deg) : %s\n', mat2str(round(abs(ep),2)));
    fprintf('    erreurs de lacet par palier (deg)   : %s\n', mat2str(round(abs(ey),2)));
    fprintf('    rmse [th psi] = [%.2f %.2f]  os [th psi] = [%.1f %.1f] %%  tension max |Vp|,|Vy| = [%.1f %.1f] V\n', ...
        m.rmse(1), m.rmse(2), m.os(1), m.os(2), max(abs(u(:,1))), max(abs(u(:,2))));
end
diary off;

function err = plat(t, ref, y, amp)
    Ts = t(2) - t(1);
    moving = [false; abs(diff(ref)) > 1e-9];
    kS = find(diff([false; moving]) == 1);
    kE = find(diff([moving; false]) == -1);
    err = [];
    for k = 1:numel(kS)
        if k < numel(kS), k1 = kS(k+1) - 1; else, k1 = length(t); end
        if abs(abs(ref(k1)) - amp) > 1e-6 || (k1 - kE(k))*Ts < 1.5, continue, end
        last = (k1 - round(1/Ts) + 1):k1;
        err(end+1) = mean(ref(last) - y(last)); %#ok<AGROW>
    end
end
