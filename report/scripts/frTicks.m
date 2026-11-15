function frTicks(ax)
% frTicks  French decimal comma on the numeric tick labels of an axes.
% Freezes the current ticks, so call it after the limits are set.
%   (helper written for the report, not part of the original project)
    xt = ax.XTick;  ax.XTick = xt;
    ax.XTickLabel = arrayfun(@(v) strrep(num2str(v, '%g'), '.', ','), xt, 'UniformOutput', false);
    yt = ax.YTick;  ax.YTick = yt;
    ax.YTickLabel = arrayfun(@(v) strrep(num2str(v, '%g'), '.', ','), yt, 'UniformOutput', false);
end
