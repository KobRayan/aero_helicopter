%% aero_data_viewer.m
% Detailed view of ONE AERO record (step, staircase, square wave, PRBS...).
% Uses aero_load.m (row map + units) and aero_step_metrics.m.
% Computations are in rad / rad/s (model units); plots in deg if cfg.plotDeg.
%
%   Fig 1  Overview: inputs, pitch, yaw, rotor speeds (shared time axis, steps marked)
%   Fig 2  Axis velocities: encoder derivative vs gyroscope (auto axis/sign/scale)
%   Fig 3  Every detected step, normalised by its amplitude (2x2 coupling matrix)
%   Fig 4  Period overlay for square waves (linearity / repeatability check)
%   Fig 5  Grey-box model(s) vs measurement with fit %  (needs the VOLTAGE input)
%   Console: step table + first estimate of the grey-box entries
%            Ksp/Jp, Dp/Jp, Kpp/Jp, Kpy/Jp, Dy/Jy, Kyp/Jy, Kyy/Jy
%   Workspace: struct "aero" (+ iddata aero.z with the System Identification Toolbox)

clear; close all; clc
hasCST = ~isempty(ver('control'));      % Control System Toolbox
hasSIT = ~isempty(ver('ident'));        % System Identification Toolbox

%% ===================== USER SETTINGS =====================
cfg.file       = 'data_step_AERO';   % .mat file
cfg.plotDeg    = true;               % plot angles in deg
cfg.fcDeriv    = 4;                  % [Hz] smoothing bandwidth for derivatives
cfg.minStepGap = 0.5;                % [s] min time between two detected steps
cfg.period     = [];                 % [s] square-wave period -> Fig 4 ([] = off)
cfg.tStart     = 0;                  % [s] initial transient to discard
cfg.greyBox    = 'quanser_aero_state_space';   % script defining A,B(,C,D); '' = skip
cfg.saveOut    = '';                 % e.g. 'sq_run1_pre.mat'

col.pitch = [0.00 0.30 0.75];  col.yaw = [0.80 0.15 0.10];  col.gyro = [0.10 0.60 0.20];

%% ===================== LOAD =====================
d    = aero_load(cfg.file, 'fc', cfg.fcDeriv, 'tStart', cfg.tStart);
t    = d.t;  Ts = d.Ts;  N = d.N;
u    = d.u;  ang = d.y;  dang = d.dy;  wMot = d.wMot;  gyro = d.gyro;
uU   = d.units.u;
if cfg.plotDeg, sc = 180/pi; pU = 'deg'; else, sc = 1; pU = 'rad'; end
inName = {'Pitch motor','Yaw motor'};

%% ===================== GYRO MATCHING =====================
gyroRate = nan(N,2);  axName = {'X','Y','Z'};
nb = max(2, min(N, round(0.5/Ts)));                     % first 0.5 s for the bias
for i = 1:2
    if std(dang(:,i)) < 1e-3, continue; end             % no motion on this axis
    r = zeros(1,3);
    for j = 1:3, R = corrcoef(gyro(:,j), dang(:,i));  r(j) = R(1,2); end
    [~, j] = max(abs(r));
    g = gyro(:,j) - mean(gyro(1:nb,j));
    kS = g \ dang(:,i);
    gyroRate(:,i) = kS*g;
    fprintf('%s rate <-> gyro %s (corr %.2f, scale %.4g)\n', ...
        inName{i}(1:end-6), axName{j}, r(j), kS);
end
fprintf(['  scale ~ 1: encoder scaling in aero_load is right and gyro is rad/s;\n' ...
         '  ~ 0.01745: gyro in deg/s; anything else: fix the angle scale in aero_load MAP.\n\n']);

%% ===================== STEPS & METRICS =====================
steps = struct('j',{},'k0',{});
for j = 1:2
    kk = detect_steps(u(:,j), Ts, cfg.minStepGap);
    for m = 1:numel(kk), steps(end+1) = struct('j',j,'k0',kk(m)); end %#ok<SAGROW>
end
[~, o] = sort([steps.k0]);  steps = steps(o);

res = {};  resp = cell(numel(steps),1);
for s = 1:numel(steps)
    j = steps(s).j;  k0 = steps(s).k0;
    if s < numel(steps), k1 = steps(s+1).k0 - 1; else, k1 = N; end
    pre  = max(1, k0 - round(0.2/Ts)) : max(1, k0 - 1);
    idx  = k0:k1;
    tail = idx(max(1, round(0.8*numel(idx))):end);
    du   = mean(u(tail,j)) - mean(u(pre,j));
    tt   = t(idx) - t(k0);
    yP   = ang(idx,1)  - mean(ang(pre,1));      % pitch angle deviation (rad)
    yR   = dang(idx,2) - mean(dang(pre,2));     % yaw RATE deviation (rad/s)
    mp   = aero_step_metrics(tt, yP, du, 2);
    my   = aero_step_metrics(tt, yR, du, 1);
    resp{s} = struct('j',j,'tt',tt,'yP',yP,'yR',yR,'du',du);
    res(end+1,:) = {s, j, t(k0), du, mp.K, mp.tr, 100*mp.OS, mp.zeta, mp.wn, ...
                    mp.settled, my.K, my.tau, my.settled}; %#ok<SAGROW>
end

T = table();  sysStep = [];
if isempty(res)
    fprintf('No input step detected.\n\n');
else
    T = cell2table(res, 'VariableNames', {'step','input','t0','du','K_theta','tr_theta', ...
        'OS_pct','zeta','wn','theta_ok','K_psidot','tau_psidot','psidot_ok'});
    fprintf('Steps (input 1 = pitch rotor, 2 = yaw rotor) | K_theta rad/%s, K_psidot rad/s/%s\n', uU, uU);
    disp(T)

    % grey-box entries, Quanser structure x = [theta psi dtheta dpsi]
    P = T(T.input==1,:);  Y = T(T.input==2,:);
    wn_p = median(P.wn,'omitnan');   ze_p = median(P.zeta,'omitnan');
    a_p  = wn_p^2;                   d_p  = 2*ze_p*wn_p;
    b_pp = median(P.K_theta,'omitnan')*a_p;
    b_yp = median(P.K_psidot./P.tau_psidot,'omitnan');
    d_y  = median(1./Y.tau_psidot,'omitnan');
    b_yy = median(Y.K_psidot./Y.tau_psidot,'omitnan');
    b_py = median(Y.K_theta,'omitnan')*a_p;
    fprintf('Grey-box entries from the steps (median):\n');
    fprintf('  Ksp/Jp = %8.4g   Dp/Jp  = %8.4g   Kpp/Jp = %8.4g   Kpy/Jp = %8.4g\n', a_p, d_p, b_pp, b_py);
    fprintf('  Dy/Jy  = %8.4g   Kyp/Jy = %8.4g   Kyy/Jy = %8.4g\n', d_y, b_yp, b_yy);
    if ~strcmp(uU,'V'), fprintf('  (!) input in %s, not V: B entries not comparable with Quanser.\n', uU); end
    if any(~[T.theta_ok; T.psidot_ok]), fprintf('  (!) some responses not settled -> biased values.\n'); end
    fprintf('\n');
    Ag = [0 0 1 0; 0 0 0 1; -a_p 0 -d_p 0; 0 0 0 -d_y];
    Bg = [0 0; 0 0; b_pp b_py; b_yp b_yy];
    if all(isfinite([Ag(:); Bg(:)])) && hasCST
        sysStep = ss(Ag, Bg, [eye(2) zeros(2)], zeros(2));
    end
end

%% ===================== FIG 1: OVERVIEW =====================
figure('Name','Overview','Color','w','Position',[60 60 1000 820]);
tl = tiledlayout(4,1,'TileSpacing','compact','Padding','compact');
title(tl, sprintf('%s', d.meta.name), 'Interpreter','none');
ax = gobjects(4,1);
ax(1) = nexttile;  plot(t, u(:,1), 'Color', col.pitch); hold on; plot(t, u(:,2), 'Color', col.yaw);
ylabel(['Input (' uU ')']);  legend(inName, 'Location','northeast');
ax(2) = nexttile;  plot(t, sc*ang(:,1), 'Color', col.pitch, 'LineWidth', 1.5);
ylabel(['\theta pitch (' pU ')']);
ax(3) = nexttile;  plot(t, sc*ang(:,2), 'Color', col.yaw, 'LineWidth', 1.5);
ylabel(['\psi yaw (' pU ')']);
ax(4) = nexttile;  plot(t, wMot(:,1), 'Color', col.pitch); hold on; plot(t, wMot(:,2), 'Color', col.yaw);
ylabel(['Rotor speed (' d.units.wMot ')']);  xlabel('Time (s)');
legend({'Pitch rotor','Yaw rotor'}, 'Location','northeast');
for a = 1:4
    style_ax(ax(a));
    for s = 1:numel(steps)
        xline(ax(a), t(steps(s).k0), ':', 'Color', [0.5 0.5 0.5], 'HandleVisibility','off');
    end
end
linkaxes(ax, 'x');  xlim(ax(1), [t(1) t(end)]);

%% ===================== FIG 2: VELOCITIES =====================
figure('Name','Axis velocities','Color','w','Position',[90 90 1000 650]);
tl = tiledlayout(3,1,'TileSpacing','compact','Padding','compact');
title(tl, 'Axis velocities: smoothed encoder derivative vs gyroscope');
bx = gobjects(3,1);  lbl = {'d\theta/dt','d\psi/dt'};  cc = {col.pitch, col.yaw};
for i = 1:2
    bx(i) = nexttile;  plot(t, sc*dang(:,i), 'Color', cc{i}, 'LineWidth', 1.3);  hold on
    leg = {'encoder derivative'};
    if all(isfinite(gyroRate(:,i)))
        plot(t, sc*gyroRate(:,i), 'Color', [col.gyro 0.6]);  leg{end+1} = 'gyro (matched)'; %#ok<SAGROW>
    end
    legend(leg, 'Location','northeast');  ylabel([lbl{i} ' (' pU '/s)']);
end
bx(3) = nexttile;  plot(t, gyro);  ylabel(['Raw gyro (' d.units.gyro ')']);  xlabel('Time (s)');
legend({'X','Y','Z'}, 'Location','northeast');
for a = 1:3, style_ax(bx(a)); end
linkaxes(bx, 'x');  xlim(bx(1), [t(1) t(end)]);

%% ===================== FIG 3: NORMALISED STEPS =====================
if ~isempty(steps)
    figure('Name','Step responses','Color','w','Position',[120 120 1000 650]);
    tl = tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
    title(tl, 'Each step divided by its amplitude (overlap = linear & repeatable)');
    ttl = {['\theta / \Delta u_{pitch} (' pU '/' uU ')'],        ['\theta / \Delta u_{yaw} (' pU '/' uU ')']; ...
           ['d\psi/dt / \Delta u_{pitch} (' pU '/s/' uU ')'],    ['d\psi/dt / \Delta u_{yaw} (' pU '/s/' uU ')']};
    cx = gobjects(2,2);
    for i = 1:2
        for j = 1:2
            cx(i,j) = nexttile((i-1)*2 + j);  hold on;  title(ttl{i,j});  style_ax(cx(i,j));
        end
    end
    for s = 1:numel(resp)
        w = resp{s};
        if abs(w.du) < eps, continue; end
        plot(cx(1,w.j), w.tt, sc*w.yP/w.du, 'LineWidth', 1.2);
        plot(cx(2,w.j), w.tt, sc*w.yR/w.du, 'LineWidth', 1.2);
    end
    for j = 1:2
        if ~any([steps.j] == j)
            text(cx(1,j), 0.5, 0.5, 'no step on this input', 'Units','normalized', ...
                'HorizontalAlignment','center', 'Color', [0.5 0.5 0.5]);
        end
        xlabel(cx(2,j), 'Time since step (s)');
    end
end

%% ===================== FIG 4: PERIOD OVERLAY =====================
if ~isempty(cfg.period)
    Pn = round(cfg.period/Ts);  nP = floor(N/Pn);
    if nP >= 2
        figure('Name','Period overlay','Color','w','Position',[150 150 1000 600]);
        tl = tiledlayout(2,1,'TileSpacing','compact','Padding','compact');
        title(tl, sprintf('%d periods of %.2f s overlaid (dark = early, yellow = late)', nP, cfg.period));
        cmap = parula(nP);  sig = {sc*ang(:,1), sc*dang(:,2)};
        ylab = {['\theta (' pU ')'], ['d\psi/dt (' pU '/s)']};
        for i = 1:2
            dx = nexttile;  hold on
            for p = 1:nP
                id = (p-1)*Pn + (1:Pn);
                plot((0:Pn-1)*Ts, sig{i}(id), 'Color', cmap(p,:));
            end
            ylabel(ylab{i});  style_ax(dx);
        end
        xlabel('Time within period (s)');
    else
        warning('Record shorter than 2 periods: no period overlay.');
    end
end

%% ===================== FIG 5: MODELS VS MEASUREMENT =====================
if ~strcmp(uU,'V')
    fprintf('Fig 5 skipped: model comparison needs the VOLTAGE input (unit in aero_load MAP).\n');
elseif ~hasCST
    fprintf('Fig 5 skipped: Control System Toolbox not found.\n');
else
    models = {};  mnames = {};
    if ~isempty(cfg.greyBox) && exist(cfg.greyBox,'file') == 2
        try
            models{end+1} = load_greybox(cfg.greyBox);  mnames{end+1} = 'Quanser grey-box';
        catch ME
            warning('Could not build the model from %s: %s', cfg.greyBox, ME.message);
        end
    end
    if ~isempty(sysStep), models{end+1} = sysStep;  mnames{end+1} = 'Step-tuned grey-box'; end
    if ~isempty(models)
        tu = (0:N-1).'*Ts;  du = u - u(1,:);  dy = ang - ang(1,:);
        ysim = cellfun(@(m) lsim(m, du, tu), models, 'UniformOutput', false);
        figure('Name','Model vs measurement','Color','w','Position',[180 180 1000 600]);
        tl = tiledlayout(2,1,'TileSpacing','compact','Padding','compact');
        title(tl, 'Simulated vs measured (deviation from the initial point)');
        ylab = {['\theta (' pU ')'], ['\psi (' pU ')']};  ex = gobjects(2,1);
        for i = 1:2
            ex(i) = nexttile;  plot(t, sc*dy(:,i), 'k', 'LineWidth', 1.5);  hold on
            leg = {'measured'};
            for m = 1:numel(models)
                e   = dy(:,i) - ysim{m}(:,i);
                fit = 100*(1 - norm(e)/norm(dy(:,i) - mean(dy(:,i))));
                plot(t, sc*ysim{m}(:,i), 'LineWidth', 1.2);
                leg{end+1} = sprintf('%s (fit %.1f %%)', mnames{m}, fit); %#ok<SAGROW>
            end
            legend(leg, 'Location','best');  ylabel(ylab{i});  style_ax(ex(i));
        end
        xlabel('Time (s)');  linkaxes(ex, 'x');
    end
end

%% ===================== EXPORT =====================
aero = d;  aero.steps = T;  aero.gyroRate = gyroRate;  aero.sysStep = sysStep;
if hasSIT
    aero.z = iddata(ang, u, Ts, 'InputName',{'u_pitch','u_yaw'}, 'OutputName',{'theta','psi'}, ...
        'InputUnit',{uU,uU}, 'OutputUnit',{'rad','rad'});
end
if ~isempty(cfg.saveOut), save(cfg.saveOut, 'aero');  fprintf('Saved to %s\n', cfg.saveOut); end

%% ===================== LOCAL FUNCTIONS =====================
function style_ax(ax)
    grid(ax,'on');  box(ax,'on');  set(ax,'FontSize',11);
end

function k = detect_steps(x, Ts, minGap)
% Indices where the median-smoothed signal jumps by > 30 % of its range
    k   = [];
    xs  = smoothdata(x, 'movmedian', max(3, round(0.05/Ts)));
    rng = max(xs) - min(xs);
    if rng < 1e-6, return; end
    L    = max(1, round(0.02/Ts));
    dd   = [zeros(L,1); xs(1+L:end) - xs(1:end-L)];
    cand = find(abs(dd) > 0.3*rng);
    if isempty(cand), return; end
    k = cand(1);
    for c = cand(2:end).'
        if (c - k(end))*Ts > minGap, k(end+1) = c; end %#ok<AGROW>
    end
end

function sys = load_greybox(name)
% Runs the manufacturer script in its own workspace (safe even if it calls clear)
    run(name);
    Am = eval('A');  Bm = eval('B');
    if exist('C','var'), Cm = eval('C'); else, Cm = [eye(2) zeros(2)]; end
    if exist('D','var'), Dm = eval('D'); else, Dm = zeros(size(Cm,1), size(Bm,2)); end
    sys = ss(Am, Bm, Cm, Dm);
end