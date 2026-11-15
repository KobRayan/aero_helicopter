% fig_01_02_05_schemas.m
% Figures 1, 2 et 5 du rapport (schémas, en français).
%   fig_01_maquette_aero.png : photo de l'AERO annotée (hélices, axes)
%       Source de la photo : Mini-project_AERO.pdf, figure 1.2 (page 3), image native
%       1414 x 1206 px extraite avec pdfimages -> aero_photo_base.png. Photo du
%       constructeur (Quanser) : citer la source dans la légende.
%       Les sens positifs de theta et psi NE SONT PAS dessinés : ils ne sont pas
%       donnés par un document consulté (à ajouter d'après le Lab Guide Quanser).
%   fig_02_methodologie.png : boucle de conception, redessinée en français d'après la
%       figure 1.1 de Mini-project_AERO.pdf (elle-même tirée de Dorf et Bishop,
%       Modern Control Systems, Pearson, 2022). Texte traduit, structure identique.
%   fig_05_schema_lqr.png : schéma bloc du LQR, d'après la diapo 7 de
%       Kobrossly_Baudin_presentation_AERO.pdf (x_ref, K, saturation ±20 V, AERO,
%       State X). Projet de figure pour la section 3.5 (Pierre) : à valider par lui.
% Versions : MATLAB R2025a.

clear; close all; clc;
root   = 'C:\Users\rayan\Desktop\Files\France\0 - Nancy\5A\S9\Control mini-projet';
outDir = fullfile(root, 'rapport', 'figures');
here   = fileparts(mfilename('fullpath'));

%% ---------- Figure 1 : photo annotée ----------
I = imread(fullfile(here, 'aero_photo_base.png'));
fig = figure('Color', 'w', 'Units', 'centimeters', 'Position', [2 2 14 12.2]);
ax = axes(fig, 'Position', [0 0 1 1]); hold(ax, 'on');
image(ax, I); axis(ax, 'image', 'off'); set(ax, 'YDir', 'reverse');
blue = [0.00 0.45 0.80];
plot(ax, [10 1400], [258 258], '--', 'Color', blue, 'LineWidth', 2);      % axe de tangage
plot(ax, [712 712], [30 1150], '--', 'Color', blue, 'LineWidth', 2);      % axe de lacet
fs = 10;
text(ax, 30, 760, {'Hélice de lacet', '(disque vertical)'}, 'FontSize', fs, 'VerticalAlignment', 'top');
annotation_arrow(ax, [150 235], [740 480]);
text(ax, 1000, 640, {'Hélice de tangage', '(disque horizontal)'}, 'FontSize', fs, 'VerticalAlignment', 'top');
annotation_arrow(ax, [1100 1140], [630 490]);
text(ax, 625, 570, {'Axe de lacet', '(vertical)'}, 'FontSize', fs, 'Color', blue, 'FontWeight', 'bold', 'VerticalAlignment', 'top', 'HorizontalAlignment', 'right');
text(ax, 1000, 195, {'Axe de tangage', '(horizontal)'}, 'FontSize', fs, 'Color', blue, 'FontWeight', 'bold', 'VerticalAlignment', 'bottom');
exportgraphics(fig, fullfile(outDir, 'fig_01_maquette_aero.png'), 'Resolution', 300);

%% ---------- Figure 2 : méthodologie de conception ----------
fig = figure('Color', 'w', 'Units', 'centimeters', 'Position', [2 2 16 13.5]);
ax = axes(fig, 'Position', [0 0 1 1]); hold(ax, 'on'); axis(ax, [0 160 -16 135], 'off');
labels = {'Établir les objectifs de commande', ...
          'Identifier les variables à commander', ...
          'Écrire les spécifications', ...
          'Établir la configuration du système', ...
          'Obtenir un modèle du procédé, de l''actionneur et du capteur', ...
          'Décrire un correcteur et choisir les paramètres à ajuster', ...
          'Optimiser les paramètres et analyser les performances'};
fills = [0.80 0.80 0.80; repmat([0.92 0.92 0.92], 6, 1)];
x0 = 28; w = 62; h = 12; ys = [122 105 88 70 53 36 19];      % bord haut de chaque boîte
for k = 1:7
    rectangle(ax, 'Position', [x0 ys(k)-h w h], 'FaceColor', fills(k,:), 'EdgeColor', 'k', 'LineWidth', 1);
    text(ax, x0 + w/2, ys(k) - h/2, wrapText(labels{k}, 34), 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'FontSize', 9);
    if k < 7, arrow(ax, [x0 + w/2, ys(k) - h], [x0 + w/2, ys(k+1)], 'k'); end
end
% accolades (1) (2) (3)
brace(ax, 96, ys(1), ys(3) - h, '(1) Objectifs, variables à commander et spécifications');
brace(ax, 96, ys(4), ys(5) - h, '(2) Définition et modélisation du système');
brace(ax, 96, ys(6), ys(7) - h, '(3) Conception, simulation et analyse du système de commande');
% boucle d'itération
yb = ys(7) - h;
plot(ax, [x0 + 12, x0 + 12, 8, 8, x0 - 0.5], [yb, 2, 2, ys(4) - h/2, ys(4) - h/2], 'k-', 'LineWidth', 1);
arrow(ax, [x0 - 6, ys(4) - h/2], [x0, ys(4) - h/2], 'k');
text(ax, 4, 0, wrapText('Si les performances ne répondent pas aux spécifications : itérer sur la configuration', 40), ...
     'FontSize', 8.5, 'VerticalAlignment', 'top');
arrow(ax, [x0 + 46, yb], [x0 + 46, 1], 'k');
text(ax, x0 + 38, 0, wrapText('Si les performances répondent aux spécifications : finaliser la conception', 40), 'FontSize', 8.5, 'VerticalAlignment', 'top');
exportgraphics(fig, fullfile(outDir, 'fig_02_methodologie.png'), 'Resolution', 300);

%% ---------- Figure 5 : schéma bloc du LQR ----------
fig = figure('Color', 'w', 'Units', 'centimeters', 'Position', [2 2 16 5.2]);
ax = axes(fig, 'Position', [0 0 1 1]); hold(ax, 'on'); axis(ax, [0 160 0 52], 'off');
yc = 28;  hh = 14;
% sommateur
cx = 20; r = 3.5;
rectangle(ax, 'Position', [cx - r yc - r 2*r 2*r], 'Curvature', [1 1], 'EdgeColor', 'k', 'LineWidth', 1);
text(ax, cx, yc, '+', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'FontSize', 10);
text(ax, cx - 4.5, yc - r - 1.5, '-', 'HorizontalAlignment', 'center', 'FontSize', 12);
block(ax, 32, yc, 16, hh, 'K', [0.80 0.80 0.80]);
block(ax, 58, yc, 22, hh, {'Saturation', '±20 V'}, [0.92 0.92 0.92]);
block(ax, 94, yc, 18, hh, 'AERO', [0.80 0.80 0.80]);
block(ax, 130, yc, 32, hh, {'State X', '(angles et vitesses', 'filtrées)'}, [0.92 0.92 0.92]);
arrow(ax, [5, yc], [cx - r, yc], 'k');
text(ax, 2, yc + 5, 'x_{ref}', 'FontSize', 10);
arrow(ax, [cx + r, yc], [24, yc], 'k');
arrow(ax, [40, yc], [47, yc], 'k');
arrow(ax, [69, yc], [85, yc], 'k');
text(ax, 77, yc + 4, 'V_p, V_y', 'HorizontalAlignment', 'center', 'FontSize', 10);
arrow(ax, [103, yc], [114, yc], 'k');
text(ax, 108.5, yc + 4, '\theta, \psi', 'HorizontalAlignment', 'center', 'FontSize', 10);
plot(ax, [146, 155], [yc, yc], 'k-', 'LineWidth', 1);
text(ax, 156.5, yc, 'x', 'FontSize', 10, 'VerticalAlignment', 'middle');
plot(ax, [151, 151, cx, cx], [yc, 8, 8, yc - r], 'k-', 'LineWidth', 1);
arrow(ax, [cx, 8], [cx, yc - r], 'k');
exportgraphics(fig, fullfile(outDir, 'fig_05_schema_lqr.png'), 'Resolution', 300);
fprintf('Figures 1, 2 et 5 écrites dans %s\n', outDir);

%% ======================= FONCTIONS LOCALES =======================
function arrow(ax, p0, p1, c)
    plot(ax, [p0(1) p1(1)], [p0(2) p1(2)], '-', 'Color', c, 'LineWidth', 1);
    d = p1 - p0;  d = d / norm(d);  n = [-d(2) d(1)];
    L = 2.2;  W = 0.9;
    tip = p1;  b = tip - L*d;
    patch(ax, [tip(1) b(1)+W*n(1) b(1)-W*n(1)], [tip(2) b(2)+W*n(2) b(2)-W*n(2)], c, 'EdgeColor', c);
end

function annotation_arrow(ax, p0, p1)
    % flèche en coordonnées image (x : [x0 x1], y : [y0 y1]) ; p0 = [x0 x1], p1 = [y0 y1]
    x = p0;  y = p1;
    plot(ax, x, y, 'k-', 'LineWidth', 1.2);
    d = [x(2) - x(1), y(2) - y(1)];  d = d / norm(d);  n = [-d(2) d(1)];
    tip = [x(2) y(2)];  b = tip - 28*d;
    patch(ax, [tip(1) b(1)+10*n(1) b(1)-10*n(1)], [tip(2) b(2)+10*n(2) b(2)-10*n(2)], 'k', 'EdgeColor', 'k');
end

function block(ax, xc, yc, w, h, txt, fc)
    rectangle(ax, 'Position', [xc - w/2, yc - h/2, w, h], 'FaceColor', fc, 'EdgeColor', 'k', 'LineWidth', 1);
    text(ax, xc, yc, txt, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'FontSize', 9);
end

function brace(ax, x, yTop, yBot, txt)
    plot(ax, [x x+2 x+2 x+4 x+2 x+2 x], [yTop yTop (yTop+yBot)/2+1 (yTop+yBot)/2 (yTop+yBot)/2-1 yBot yBot], 'k-', 'LineWidth', 1);
    text(ax, x + 6, (yTop + yBot)/2, wrapText(txt, 26), 'FontSize', 8.5, 'VerticalAlignment', 'middle');
end

function out = wrapText(s, n)
    words = strsplit(s, ' ');  out = {};  cur = '';
    for k = 1:numel(words)
        if isempty(cur), cand = words{k}; else, cand = [cur ' ' words{k}]; end
        if numel(cand) > n && ~isempty(cur), out{end+1} = cur; cur = words{k}; else, cur = cand; end %#ok<AGROW>
    end
    out{end+1} = cur;
end
