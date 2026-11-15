% Lecture seule : compare les 5 versions de quanser_aero_parameters_identified*.mat
% (aucun fichier du projet n'est modifié). Sortie : tableau texte dans la console.
root  = 'C:\Users\rayan\Desktop\Files\France\0 - Nancy\5A\S9\Control mini-projet\MiniProjetAeroCS';
files = {'reponse_BO_experimentation\quanser_aero_parameters_identified.mat', ...
         'reponse_BO_experimentation\quanser_aero_parameters_identified_2.mat', ...
         'reponse_BO_experimentation\quanser_aero_parameters_identified_a.mat', ...
         'reponse_BO_experimentation\quanser_aero_parameters_identified_b.mat', ...
         'LQR\quanser_aero_parameters_identified_2.mat'};
names = {'Jp','Jy','Ksp','Dp','Dy','Kpp','Kpy','Kyp','Kyy'};
fprintf('%-66s', 'fichier'); fprintf('%11s', names{:}); fprintf('\n');
for f = 1:numel(files)
    S = load(fullfile(root, files{f}));
    fprintf('%-66s', files{f});
    for k = 1:numel(names)
        if isfield(S, names{k}), fprintf('%11.6f', S.(names{k})); else, fprintf('%11s', '-'); end
    end
    fprintf('\n');
    others = setdiff(fieldnames(S), names);
    fprintf('    autres variables : %s\n', strjoin(others', ', '));
end
