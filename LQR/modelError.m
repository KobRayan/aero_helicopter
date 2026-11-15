% modelError.m
% 05/10/2026
% Compares a simulation with the measurement of the SAME controller
% (any controller: LQR, LQI, PID...).
%
% Usage:
%   e = modelError(ySim, uSim, yMeas, uMeas)
%   ySim, yMeas : angles (deg), N x 2 [pitch, yaw]
%   uSim, uMeas : voltages (V), N x 2 [Vp, Vy]
%
% Called by tune_lqr_AERO.m. Must be in the same folder as the scripts that use it.

function e = modelError(ySim, uSim, yMeas, uMeas)
% How far a simulation is from the measurement of the SAME controller
% (N x 2 inputs, column 1 = pitch, 2 = yaw). Every field is 1 x 2:
%   fit   : 100*(1 - ||yMeas - ySim|| / ||yMeas - mean(yMeas)||)   (%)
%           100 % = perfect, 0 % = no better than a constant
%   rmseY : RMS of yMeas - ySim  (deg)
%   rmseU : RMS of uMeas - uSim  (V)
    e.fit   = 100*(1 - sqrt(sum((yMeas - ySim).^2)) ./ sqrt(sum((yMeas - mean(yMeas)).^2)));
    e.rmseY = sqrt(mean((yMeas - ySim).^2));
    e.rmseU = sqrt(mean((uMeas - uSim).^2));
end
