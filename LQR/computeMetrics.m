% computeMetrics.m
% 05/10/2026
% Performance metrics of ONE run of the AERO (simulated or measured), for ANY
% controller (LQR, LQI, PID...): only the logged signals are used.
%
% Usage:
%   m = computeMetrics(t, ref, y, u, satV, epsSettle)
%   t         : time (s), N x 1
%   ref, y, u : N x 2, column 1 = pitch, column 2 = yaw
%               references (deg), angles (deg), motor voltages (V)
%   satV      : voltage saturation (V), e.g. 20
%   epsSettle : settling band as a fraction of the step size, e.g. 0.05
%
% Called by tune_lqr_AERO.m. Must be in the same folder as the scripts that use it.
% Only the first function of this file (computeMetrics) can be called from
% outside; stepMetrics, coupling and moveStarts are helpers used only inside this file.
%
% 06/10/2026: works for step AND ramp references (Rate Limiter). A "move" of
% the reference is either a step (1 sample) or a ramp (many samples in a row);
% moveStarts finds the first sample of each move. Before this fix, every sample
% of a ramp was seen as a new step, which gave NaN rise/settling times, a wrong
% overshoot and a wrong coupling. On step references the results are unchanged.
% With a ramp, tr and ts are counted from the START of the ramp, so they
% include the ramp duration (set by the slew rate, not by the controller).

function m = computeMetrics(t, ref, y, u, satV, epsSettle)
% Performance metrics of ONE run (simulated or measured), for ANY controller:
% only the logged signals are used. Inputs are N x 2, column 1 = pitch, 2 = yaw:
%   ref : references (deg)    y : angles (deg)    u : motor voltages (V)
% Every field of m is 1 x 2 ([pitch, yaw]):
%   rmse : tracking RMSE over the whole test, sqrt(mean((ref - y).^2))     (deg)
%   tr, os, ts, ess : rise time 10-90 % (s), overshoot (%), settling time (s)
%          and steady-state error (deg), median over the steps of each axis
%   rmsU : RMS voltage of each motor, sqrt(mean(u.^2))                      (V)
%   sat  : time spent at the saturation, per motor                          (%)
%   tv   : voltage activity = total variation per second, sum|du| / T     (V/s)
%          (large when the voltage is noisy or oscillates)
%   cpl  : coupling, move of an axis when only the OTHER reference steps    (deg)
%          cpl(1) = pitch moved by the yaw steps, cpl(2) = yaw moved by the pitch steps
    T = t(end) - t(1);

    m.rmse = sqrt(mean((ref - y).^2));

    m.tr  = nan(1,2);
    m.os  = nan(1,2);
    m.ts  = nan(1,2);
    m.ess = nan(1,2);
    for a = 1:2
        [tr, os, ts, ess] = stepMetrics(t, ref(:,a), y(:,a), epsSettle);
        m.tr(a)  = tr;
        m.os(a)  = os;
        m.ts(a)  = ts;
        m.ess(a) = ess;
    end

    m.rmsU = sqrt(mean(u.^2));
    m.sat  = 100*mean(abs(u) >= 0.99*satV);
    m.tv   = sum(abs(diff(u))) / T;

    m.cpl = [coupling(t, ref(:,2), ref(:,1), y(:,1)), ...    % yaw steps   -> pitch
             coupling(t, ref(:,1), ref(:,2), y(:,2))];       % pitch steps -> yaw
end

function [tr, OS, ts, ess] = stepMetrics(t, ref, y, epsSettle)
% Median over all the steps of ref of:
%   tr  : rise time 10-90 % (s)
%   OS  : overshoot (%) of the step size
%   ts  : settling time (s): time after which the angle stays within
%         +/- epsSettle*step size of its FINAL value (the static error is
%         measured separately by ess, so a step with a static error can settle)
%   ess : absolute steady-state error (deg), mean over the last second
% Median rather than mean: a step disturbed by the other axis does not dominate.
% Segments shorter than 1 s are ignored (e.g. a reference changing on the last sample).
% Each segment goes from the start of a move to the start of the next one;
% the target r1 is the plateau reached at the end of the segment.
    Ts    = t(2) - t(1);
    edges = [moveStarts(ref); length(t) + 1];
    n     = length(edges) - 1;
    trs = nan(n,1);  oss = nan(n,1);  tss = nan(n,1);  es = nan(n,1);
    for s = 1:n
        k0  = edges(s);
        idx = (k0:edges(s+1) - 1)';
        if length(idx)*Ts < 1
            continue                            % too short to be a real step
        end
        if abs(ref(idx(end)) - ref(idx(end-1))) > 1e-6
            continue                            % ramp still running when the test ends: no plateau
        end
        pre = max(1, k0 - round(0.2/Ts)):k0 - 1;
        y0  = mean(y(pre));                     % angle just before the move
        r1  = ref(idx(end));                    % target = plateau at the end of the move
                                                % (for a step, same as ref(k0))
        z   = (y(idx) - y0) / (r1 - y0);        % normalised response: goes from 0 to 1
        tt  = t(idx) - t(k0);

        i10 = find(z >= 0.1, 1);
        i90 = find(z >= 0.9, 1);
        if ~isempty(i10) && ~isempty(i90)
            trs(s) = tt(i90) - tt(i10);
        end

        oss(s) = max(0, 100*(max(z) - 1));

        yEnd  = mean(y(idx(tt >= tt(end) - 1)));     % final value: mean of the last second
        es(s) = abs(r1 - yEnd);

        band = epsSettle * abs(r1 - ref(k0-1));      % +/- 5 % of the step size
                                                     % (ref(k0-1) = plateau before the move)
        iOut = find(abs(y(idx) - yEnd) > band, 1, 'last');
        if isempty(iOut)
            tss(s) = 0;                         % always inside the band
        elseif iOut < length(idx)
            tss(s) = tt(iOut + 1);              % first sample after the last exit
        end                                     % else: never settled -> NaN
    end
    tr  = median(trs, 'omitnan');
    OS  = median(oss, 'omitnan');
    ts  = median(tss, 'omitnan');
    ess = median(es,  'omitnan');
end

function c = coupling(t, refA, refB, yB)
% Move of axis B (deg) after each move of reference A, only when reference B
% stays constant (from 0.2 s before the move to the end of the window).
% This skips moves of both references at the same time, and also a move of A
% that starts while B is still ramping (with steps this second case never happens).
% For each move: largest |yB - value just before the move| until the next
% move of either reference. Returns the median over the moves (NaN if none).
    Ts     = t(2) - t(1);
    movesA = moveStarts(refA);
    movesB = moveStarts(refB);
    dev    = [];
    for k0 = movesA'
        kEnd = min([movesA(movesA > k0); movesB(movesB > k0); length(t) + 1]) - 1;
        pre  = max(1, k0 - round(0.2/Ts)):k0 - 1;
        refBwin = refB(pre(1):kEnd);            % reference B over the whole window
        if max(refBwin) - min(refBwin) > 1e-6 || (kEnd - k0 + 1)*Ts < 1
            continue                            % reference B moves too, or window too short
        end
        dev(end+1) = max(abs(yB(k0:kEnd) - mean(yB(pre))));   %#ok<AGROW>
    end
    c = median(dev);                            % median([]) = NaN
end

function k = moveStarts(ref)
% First sample of each move of the reference (column vector of indices).
% A step changes the reference on 1 sample, a ramp on many samples in a row:
% consecutive changing samples count as ONE move.
% The 1e-6 deg threshold ignores rounding errors of the Rate Limiter.
    changing = [false; abs(diff(ref)) > 1e-6];          % true if ref(k) ~= ref(k-1)
    k = find(changing & ~[false; changing(1:end-1)]);   % changing now, not at k-1
end