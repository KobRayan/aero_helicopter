%% plot_all_tests_AERO.m
% =========================================================================
%  PLOT OF ALL THE IDENTIFICATION TESTS (before any identification)
% =========================================================================
% ONE window with one TAB per .mat file, plus a last tab "Résumé" with a
% summary table. Each test tab contains a short description (in French) and
% 3 plots with linked time axes:
%   1. tensions : pitch motor voltage Vp and yaw motor voltage Vy (V)
%   2. tangage  : pitch angle theta (deg)
%   3. lacet    : yaw angle psi (deg)
% The descriptions are also printed in the Command Window.
%
% Data format (AERO_basics.slx, variable data0, 12 rows x N samples, Ts = 2 ms):
%   1 time (s) | 2 pitch motor current (A) | 3 yaw motor current (A)
%   4 pitch angle (deg) | 5 yaw angle (deg)
%   6 pitch propeller speed (rad/s) | 7 yaw propeller speed (rad/s)
%   8-10 gyro X, Y, Z (rad/s) | 11 pitch voltage Vp (V) | 12 yaw voltage Vy (V)
%
% The .mat files must be in the current folder.
% Requires MATLAB R2019b or newer (tiledlayout inside a tab).

clear; close all; clc;

%% ===================== 0. LIST OF THE TESTS =====================
% One line per file:
%   file name (without .mat) | tab name | title | description (French)
%   | time of a vertical marker (NaN = no marker)
tests = {
  'custom_pitch_8V_blocked', 'G11 - lacet bloqué', ...
  'Lacet BLOQUÉ - créneau sur Vp', ...
  ['Lacet bloqué avec la clé Allen. Vp = 0 / -8 / 0 / +8 V, 10 s par palier, ' ...
   'deux fois (80 s). Vy = 0. Seul le tangage bouge : essai utilisé pour ' ...
   'G11 = theta/Vp. Attendu : tangage d''environ -12° à -8 V et +10° à +8 V, ' ...
   'avec des oscillations à chaque changement ; le lacet reste dans +/-1° ' ...
   '(jeu du blocage).'], NaN;

  'custom_yaw_blocked_8V', 'G12 - lacet bloqué', ...
  'Lacet BLOQUÉ - créneau sur Vy', ...
  ['Lacet bloqué. Vy = 0 / -8 / 0 / +8 V, 10 s par palier (80 s). Vp = 0. ' ...
   'L''hélice de lacet tourne toujours et son couple de réaction agit sur ' ...
   'le tangage : essai utilisé pour G12 = theta/Vy. Attendu : une grande ' ...
   'première oscillation du tangage à chaque changement (mise en vitesse de ' ...
   'l''hélice), puis un angle stabilisé plus petit.'], NaN;

  'custom_pitch_free_8V', 'G21 - libre', ...
  'LIBRE - créneau sur Vp', ...
  ['Aucun axe bloqué. Vp = 0 / -8 / 0 / +8 V, 10 s par palier (80 s). Vy = 0. ' ...
   'Sortie lacet = G21 = psi/Vp. Attendu : le lacet ne bouge que par à-coups ' ...
   '(10 à 25°) pendant que l''hélice de tangage accélère ou ralentit, puis ' ...
   'le frottement sec l''arrête.'], NaN;

  'custom_yaw_free_8V', 'G22 - libre', ...
  'LIBRE - créneau sur Vy', ...
  ['Aucun axe bloqué. Vy = 0 / -8 / 0 / +8 V, 10 s par palier (80 s). Vp = 0. ' ...
   'Sortie lacet = G22 = psi/Vy. Attendu : le lacet tourne d''environ 110 à ' ...
   '135° pendant chaque palier à +/-8 V et s''arrête en ~2 s quand Vy revient ' ...
   'à 0 (frottement sec). Le tangage bouge aussi (effet G12).'], NaN;

  'custom_pitchsteps_8V_locked', 'Escalier - lacet bloqué', ...
  'Lacet BLOQUÉ - escalier sur Vp', ...
  ['Lacet bloqué. Vp = 0, 3, 6, 9, 12, 9, 6, 3, 0, -3, -6, -9, -12, -9, -6, -3, 0 V, ' ...
   '10 s par palier (160 s). Vy = 0. Caractéristique statique : angle de ' ...
   'tangage stabilisé en fonction de la tension (poussée ~ V², tensions ' ...
   'négatives plus fortes) et frottement du tangage (le tangage ne revient ' ...
   'pas exactement à 0 à 0 V).'], NaN;

  'custom_freeramp_pitch', 'Escalier 1 V - libre', ...
  'LIBRE - escalier de 1 V sur Vp', ...
  ['Aucun axe bloqué. Vp de 0 à 10 V par pas de 1 V toutes les 10 s (110 s), ' ...
   'puis 0. Vy = 0. Caractéristique statique avec le lacet libre, et tension ' ...
   'à partir de laquelle la traînée de l''hélice de tangage fait tourner le ' ...
   'lacet (environ 10 V).'], NaN;

  'custom_free_2dof1', '2 DDL - essai 1', ...
  'LIBRE 2 DDL - deux créneaux (essai 1)', ...
  ['Aucun axe bloqué. Vp = +/-6 V (période 20 s) et Vy = +/-9 V (période 26 s) ' ...
   'en même temps. Données de validation du modèle à 2 entrées. Vers 91 s, ' ...
   'les deux tensions deviennent négatives à 1 s d''intervalle : le tangage ' ...
   'plonge vers -55°, touche la base et QUARC s''arrête. N''utiliser que les ' ...
   '88 premières secondes (trait rouge).'], 88;

  'custom_free_2dof2', '2 DDL - essai 2', ...
  'LIBRE 2 DDL - deux créneaux (essai 2)', ...
  ['Même essai que l''essai 1, répété : vérifie la répétabilité (les deux ' ...
   'essais doivent être presque identiques) et donne un second jeu de ' ...
   'validation. N''utiliser que les 88 premières secondes (trait rouge).'], 88;
};
nT = size(tests, 1);

% Summary table (one line per test), shown in the last tab
summary = cell(nT, 7);

%% ===================== 1. ONE WINDOW, ONE TAB PER TEST =====================
fig = figure('Name', 'Essais d''identification - Quanser AERO', ...
             'NumberTitle', 'off', 'Position', [80 40 1000 850]);
tabGroup = uitabgroup(fig);

for i = 1:nT
    fileName = tests{i,1};

    % --- Skip the file if it is not in the current folder
    if ~isfile([fileName '.mat'])
        fprintf('Fichier introuvable, ignoré : %s.mat\n', fileName);
        continue;
    end

    % --- Load the 12-row matrix and name each row (same style as our scripts)
    S     = load([fileName '.mat']);
    data0 = S.data0;
    t               = data0(1,:)';    % time (s)
    PitchMotor      = data0(2,:)';    % pitch motor current (A)
    YawMotor        = data0(3,:)';    % yaw motor current (A)
    PitchEncoder    = data0(4,:)';    % pitch angle (deg)
    YawEncoder      = data0(5,:)';    % yaw angle (deg)
    PitchMotorSpeed = data0(6,:)';    % pitch propeller speed (rad/s)
    YawMotorSpeed   = data0(7,:)';    % yaw propeller speed (rad/s)
    PitchVoltage    = data0(11,:)';   % pitch motor voltage Vp (V)
    YawVoltage      = data0(12,:)';   % yaw motor voltage Vy (V)
    Ts = t(2) - t(1);                 % sampling period (s)

    % --- Description in the Command Window
    fprintf('\n=== %d. %s  (%s) ===\n', i, tests{i,3}, fileName);
    fprintf('%s\n', tests{i,4});

    % --- New tab, with a 3 x 1 tiled layout inside it
    tab = uitab(tabGroup, 'Title', sprintf('%d. %s', i, tests{i,2}));
    tl  = tiledlayout(tab, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    % Title of the tab = test name + description (wrapped at ~120 characters)
    title(tl, [{sprintf('%d. %s  (%s)', i, tests{i,3}, fileName)}; ...
               wrapText(tests{i,4}, 120)], 'Interpreter', 'none', 'FontSize', 9);

    ax = gobjects(3, 1);

    ax(1) = nexttile(tl);
    plot(ax(1), t, PitchVoltage, 'b', t, YawVoltage, 'r'); grid(ax(1), 'on');
    ylabel(ax(1), 'Tension (V)');
    legend(ax(1), 'Vp (moteur tangage)', 'Vy (moteur lacet)', 'Location', 'best');

    ax(2) = nexttile(tl);
    plot(ax(2), t, PitchEncoder, 'b'); grid(ax(2), 'on');
    ylabel(ax(2), '\theta tangage (°)');

    ax(3) = nexttile(tl);
    plot(ax(3), t, YawEncoder, 'r'); grid(ax(3), 'on');
    ylabel(ax(3), '\psi lacet (°)');
    xlabel(ax(3), 'Temps (s)');

    linkaxes(ax, 'x');
    xlim(ax(1), [t(1) t(end)]);

    % --- Vertical marker (end of the valid data for the 2-DOF tests)
    if ~isnan(tests{i,5})
        for a = 1:3
            xline(ax(a), tests{i,5}, 'r--', 'LineWidth', 1);
        end
    end

    % --- Unit check: smallest non-zero change of an angle = one encoder count
    dP = abs(diff(PitchEncoder)); dP = dP(dP > 1e-9);
    dY = abs(diff(YawEncoder));   dY = dY(dY > 1e-9);
    if ~isempty(dP), fprintf('  tangage : 1 incrément = %.4f° (attendu 0.1758)\n', min(dP)); end
    if ~isempty(dY), fprintf('  lacet   : 1 incrément = %.4f° (attendu 0.0879)\n', min(dY)); end
    fprintf('  Ts = %.4f s, durée %.1f s\n', Ts, t(end));

    % --- Values for the summary table
    yawLocked = max(abs(YawEncoder)) < 2;     % less than 2 deg of yaw motion
    lockText = 'non';
    if yawLocked, lockText = 'OUI'; end
    summary(i,:) = {fileName, round(t(end), 1), levelsText(PitchVoltage), ...
                    levelsText(YawVoltage), ...
                    sprintf('%.1f à %.1f', min(PitchEncoder), max(PitchEncoder)), ...
                    sprintf('%.1f à %.1f', min(YawEncoder), max(YawEncoder)), lockText};
end

%% ===================== 2. SUMMARY TAB =====================
summary = summary(~cellfun(@isempty, summary(:,1)), :);   % remove skipped files
tabSum = uitab(tabGroup, 'Title', 'Résumé');
uitable(tabSum, 'Data', summary, 'Units', 'normalized', 'Position', [0.02 0.02 0.96 0.96], ...
        'ColumnName', {'Fichier', 'Durée (s)', 'Paliers Vp (V)', 'Paliers Vy (V)', ...
                       'Tangage (°)', 'Lacet (°)', 'Lacet bloqué ?'}, ...
        'ColumnWidth', {200, 70, 170, 110, 120, 130, 100});

% Same table in the Command Window
fprintf('\n===== Résumé =====\n');
fprintf('%-28s %8s  %-24s %-14s %-16s %-18s %s\n', 'Fichier', 'Durée(s)', 'Paliers Vp (V)', ...
        'Paliers Vy (V)', 'Tangage (°)', 'Lacet (°)', 'Lacet bloqué ?');
for i = 1:size(summary, 1)
    fprintf('%-28s %8.1f  %-24s %-14s %-16s %-18s %s\n', summary{i,:});
end

%% ===================== LOCAL FUNCTIONS =====================
function txt = levelsText(v)
% levelsText  Lists the different constant levels of a voltage signal,
% e.g. '-8 0 8'. If the signal has too many levels, gives its range.
lv = unique(round(v, 1))';
if numel(lv) <= 8
    txt = strtrim(sprintf('%g ', lv));
else
    txt = sprintf('%g à %g', min(lv), max(lv));
end
end

function lines = wrapText(str, n)
% wrapText  Cuts a long sentence into lines of about n characters (at the
% spaces), returned as a cell array (one cell per line) for the title.
words = strsplit(str, ' ');
lines = {};
current = '';
for k = 1:numel(words)
    if length(current) + length(words{k}) + 1 > n
        lines{end+1, 1} = current;    %#ok<AGROW>
        current = words{k};
    elseif isempty(current)
        current = words{k};
    else
        current = [current ' ' words{k}]; %#ok<AGROW>
    end
end
lines{end+1, 1} = current;
end