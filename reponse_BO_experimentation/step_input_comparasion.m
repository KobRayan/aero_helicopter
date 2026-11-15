% plot_all_steps_AERO.m
% 30/09/2026
% Compares all the open-loop step tests of the Quanser AERO.
%
%   Figure 1 : steps on the PITCH motor (5, 10, 15 V)
%   Figure 2 : steps on the YAW motor   (5, 10, 15 V)
%   Figure 3 : cross-coupling matrix, every response divided by its voltage
%              -> if the 5/10/15 V curves overlap, the system is linear
%   Figure 4 : both motors at 10 V, compared with (pitch 10 V alone + yaw 10 V alone)
%              -> if the two curves match, superposition holds, i.e. linear
%   Command window : final values and gains of every test
%
% Every file has the same 12-row data0 as in step_response_AERO.m:
%   1 time | 2-3 currents | 4 pitch (deg) | 5 yaw (deg) | 6-7 rotor speeds
%   8-10 gyro X Y Z | 11 pitch voltage (V) | 12 yaw voltage (V)
% The .mat files must be in the current folder.

clear;
close all;
clc

%% 0) Names of the files to compare (without .mat) - edit if yours differ
pitchFiles = {'step_pitch_5V', 'step_pitch_10V', 'step_pitch_15V'};
yawFiles   = {'step_yaw_05V',   'step_yaw_10V',   'step_yaw_15V'};
bothFile   =  'step_both_10V10V';

% Single-motor tests used for the superposition check (same voltages as bothFile)
pitchRef   =  'step_pitch_10V';
yawRef     =  'step_yaw_10V';

% One colour per file in each list (1st = blue, 2nd = orange, 3rd = green)
colors = [0.00 0.45 0.74;
          0.85 0.33 0.10;
          0.47 0.67 0.19;
          lines(7)];                % extra colours if a list has more files

%% 1) Load every test (see the function loadAero at the end of the file)
for k = 1:numel(pitchFiles)
    P(k) = loadAero(pitchFiles{k});         %#ok<SAGROW>  pitch-motor tests
end
for k = 1:numel(yawFiles)
    Y(k) = loadAero(yawFiles{k});           %#ok<SAGROW>  yaw-motor tests
end
B  = loadAero(bothFile);                    % both motors
Pr = loadAero(pitchRef);                    % pitch alone, for superposition
Yr = loadAero(yawRef);                      % yaw alone, for superposition

%% 2) Figures 1 and 2: raw responses of each group of tests
plotGroup(P, 'Pitch', colors);
plotGroup(Y, 'Yaw',   colors);

%% 3) Figure 3: cross-coupling matrix, responses per volt
%  Rows    : output (pitch angle, yaw rate)
%  Columns : motor that received the step (pitch, yaw)
%  Diagonal = direct effect, off-diagonal = coupling between the axes.
%  The yaw RATE is used instead of the yaw angle because the yaw angle keeps
%  growing (the yaw axis is an integrator): its rate reaches a final value.
figure('Name', 'Cross-coupling per volt', 'Color', 'w');
groups   = {P, Y};
colTitle = {'Pitch motor', 'Yaw motor'};
for j = 1:2
    runs = groups{j};
    axT  = subplot(2,2,j);      hold on;  grid on    % top row    : pitch angle
    axR  = subplot(2,2,2+j);    hold on;  grid on    % bottom row : yaw rate
    h    = gobjects(numel(runs),1);
    lbl  = cell(numel(runs),1);
    for k = 1:numel(runs)
        r = runs(k);
        if j == 1, V = r.VpEnd; else, V = r.VyEnd; end
        h(k)   = plot(axT, r.t, r.theta/V, 'Color', colors(k,:), 'LineWidth', 1.5);
        plot(axR, r.t, r.dpsi/V, 'Color', colors(k,:), 'LineWidth', 1.5);
        lbl{k} = sprintf('%.0f V', V);
    end
    if j == 1
        title(axT, 'Pitch motor \rightarrow pitch angle (direct)')
        title(axR, 'Pitch motor \rightarrow yaw rate (coupling)')
    else
        title(axT, 'Yaw motor \rightarrow pitch angle (coupling)')
        title(axR, 'Yaw motor \rightarrow yaw rate (direct)')
    end
    ylabel(axT, '\theta / V  (deg/V)')
    ylabel(axR, 'd\psi/dt / V  (deg/s/V)')
    xlabel(axR, 'Time since the step (s)')
    legend(axT, h, lbl, 'Location', 'best')
    linkaxes([axT axR], 'x')
    xlim(axT, [-1, max(arrayfun(@(r) r.t(end), runs))])
end
sgtitle('Responses divided by the voltage: overlapping curves = linear')

%% 4) Figure 4: superposition check with both motors at 10 V
% For a linear system: response(pitch 10 V + yaw 10 V)
%                    = response(pitch 10 V) + response(yaw 10 V)
% The single tests are interpolated on the time vector of the combined test
% (all times are relative to the step, so the curves are aligned).
% NaN (gaps in the plot) where a single test has no data at that time.
sumTheta = interp1(Pr.t, Pr.theta, B.t) + interp1(Yr.t, Yr.theta, B.t);
sumPsi   = interp1(Pr.t, Pr.psi,   B.t) + interp1(Yr.t, Yr.psi,   B.t);
sumDpsi  = interp1(Pr.t, Pr.dpsi,  B.t) + interp1(Yr.t, Yr.dpsi,  B.t);

figure('Name', 'Superposition check', 'Color', 'w');
cx = gobjects(4,1);
cx(1) = subplot(4,1,1);
plot(B.t, B.Vp, 'b', B.t, B.Vy, 'r', 'LineWidth', 1.5);  grid on
title(['Inputs of ' bothFile], 'Interpreter', 'none')
ylabel('Voltage (V)')
legend('Pitch motor', 'Yaw motor', 'Location', 'southeast')

cx(2) = subplot(4,1,2);
plot(B.t, B.theta, 'k', B.t, sumTheta, 'm--', 'LineWidth', 1.5);  grid on
title('Pitch angle \theta')
ylabel('\theta (deg)')
legend('Both motors (measured)', 'Pitch alone + yaw alone', 'Location', 'best')

cx(3) = subplot(4,1,3);
plot(B.t, B.psi, 'k', B.t, sumPsi, 'm--', 'LineWidth', 1.5);  grid on
title('Yaw angle \psi')
ylabel('\psi (deg)')

cx(4) = subplot(4,1,4);
plot(B.t, B.dpsi, 'k', B.t, sumDpsi, 'm--', 'LineWidth', 1.5);  grid on
title('Yaw rate d\psi/dt')
ylabel('d\psi/dt (deg/s)')
xlabel('Time since the step (s)')

linkaxes(cx, 'x')
xlim(cx(1), [-1, B.t(end)])

%% 5) Table of final values in the command window
% "end" = mean over the last 5 s of the record.
% Gains (per volt) are only computed for single-motor tests.
% settled = the yaw rate changed by less than 5 % (or 1 deg/s) between the last two 5 s windows;
%           if "no", the test was too short and the final yaw rate is underestimated.
allRuns = [P, Y, B];
fprintf('\n%-18s %6s %6s | %9s %9s %11s | %10s %12s | %7s\n', 'File', 'Vp', 'Vy', ...
        'theta end', 'theta pk', 'dpsi/dt end', 'theta/V', 'dpsi/dt / V', 'settled');
fprintf('%-18s %6s %6s | %9s %9s %11s | %10s %12s | %7s\n', '', '(V)', '(V)', ...
        '(deg)', '(deg)', '(deg/s)', '(deg/V)', '(deg/s/V)', '');
for r = allRuns
    if abs(r.VpEnd) > 0.1 && abs(r.VyEnd) < 0.1
        V = r.VpEnd;                        % pitch motor only
    elseif abs(r.VyEnd) > 0.1 && abs(r.VpEnd) < 0.1
        V = r.VyEnd;                        % yaw motor only
    else
        V = NaN;                            % both motors: no single gain
    end
    if r.settled, s = 'yes'; else, s = 'no'; end
    fprintf('%-18s %6.1f %6.1f | %9.2f %9.2f %11.2f | %10.3f %12.3f | %7s\n', ...
            r.name, r.VpEnd, r.VyEnd, r.thetaEnd, r.thetaPeak, r.dpsiEnd, ...
            r.thetaEnd/V, r.dpsiEnd/V, s);
    if abs(r.thetaPeak) > 58
        fprintf('   (!) %s: pitch reached %.1f deg, close to the +/-62 deg stop\n', ...
                r.name, r.thetaPeak);
    end
end
fprintf('\nSuperposition at the end of the test: theta %.2f (both) vs %.2f (sum) deg, ', ...
        B.thetaEnd, mean(sumTheta(B.t > B.t(end) - 5), 'omitnan'));
fprintf('dpsi/dt %.2f (both) vs %.2f (sum) deg/s\n', ...
        B.dpsiEnd, mean(sumDpsi(B.t > B.t(end) - 5), 'omitnan'));

%% ======================= LOCAL FUNCTIONS =======================
% (in a script, the functions must be at the end of the file)

function d = loadAero(fileName)
% Loads one test and returns its signals.
% Time is shifted so that t = 0 is the instant of the voltage step.
    S = load(fileName);
    D = S.data0;
    if size(D,1) ~= 12
        error('%s has %d rows instead of 12.', fileName, size(D,1));
    end
    t  = D(1,:)';
    Ts = t(2) - t(1);

    d.name  = fileName;
    d.Vp    = D(11,:)';                 % pitch motor voltage (V)
    d.Vy    = D(12,:)';                 % yaw motor voltage (V)
    d.theta = D(4,:)' - D(4,1);         % pitch angle (deg), starts at 0
    d.psi   = D(5,:)' - D(5,1);         % yaw angle (deg), starts at 0

    % Step instant = first sample where the voltages reach half of their maximum
    V  = abs(d.Vp) + abs(d.Vy);
    k0 = find(V > 0.5*max(V), 1);
    if isempty(k0)
        warning('%s: no voltage step found, time not shifted.', fileName);
        k0 = 1;
    end
    d.t = t - t(k0);                    % time relative to the step (s)

    % Angular velocities: encoder derivative after a 0.1 s zero-phase smoothing
    win      = round(0.1/Ts);
    d.dtheta = gradient(smoothdata(d.theta, 'gaussian', win), Ts);   % deg/s
    d.dpsi   = gradient(smoothdata(d.psi,   'gaussian', win), Ts);   % deg/s

    % Final values = mean over the last 5 s
    last       = d.t > d.t(end) - 5;
    before     = d.t > d.t(end) - 10 & ~last;          % the 5 s before that
    d.VpEnd    = mean(d.Vp(last));
    d.VyEnd    = mean(d.Vy(last));
    d.thetaEnd = mean(d.theta(last));
    d.dpsiEnd  = mean(d.dpsi(last));
    [~, i]      = max(abs(d.theta));
    d.thetaPeak = d.theta(i);           % largest pitch angle, with its sign
    d.settled   = abs(d.dpsiEnd - mean(d.dpsi(before))) < 0.05*abs(d.dpsiEnd) + 1;
end

function plotGroup(runs, motor, colors)
% One figure for a group of tests on the same motor (motor = 'Pitch' or 'Yaw').
    figure('Name', ['Steps on the ' motor ' motor'], 'Color', 'w');
    titles  = {[motor ' motor voltage (input)'], 'Pitch angle \theta', ...
               'Yaw angle \psi', 'Yaw rate d\psi/dt'};
    ylabels = {'V (V)', '\theta (deg)', '\psi (deg)', 'd\psi/dt (deg/s)'};
    ax = gobjects(4,1);
    for i = 1:4
        ax(i) = subplot(4,1,i);  hold on;  grid on
        title(titles{i})
        ylabel(ylabels{i})
    end
    xlabel(ax(4), 'Time since the step (s)')

    h   = gobjects(numel(runs),1);
    lbl = cell(numel(runs),1);
    for k = 1:numel(runs)
        r = runs(k);
        if strcmp(motor, 'Pitch'), V = r.Vp; Vend = r.VpEnd;
        else,                      V = r.Vy; Vend = r.VyEnd; end
        c = colors(k,:);
        h(k) = plot(ax(1), r.t, V,       'Color', c, 'LineWidth', 1.5);
        plot(ax(2),        r.t, r.theta, 'Color', c, 'LineWidth', 1.5);
        plot(ax(3),        r.t, r.psi,   'Color', c, 'LineWidth', 1.5);
        plot(ax(4),        r.t, r.dpsi,  'Color', c, 'LineWidth', 1.5);
        lbl{k} = sprintf('%.0f V', Vend);
    end
    legend(ax(1), h, lbl, 'Location', 'southeast')
    linkaxes(ax, 'x')
    xlim(ax(1), [-1, max(arrayfun(@(r) r.t(end), runs))])
end