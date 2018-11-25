function [] = plot_single(filename, double, varargin)
%plot_single plot bennchmark results
%
%  Usage: plot_single ( FILENAME , DOUBLE )
%         plot_single ( FILENAMES , DOUBLE, METRICS )
%
%  FILENAME   CSV file with benchmark results
%  DOUBLE     boolean indicating whether tests in input file represent
%             scaling analysis over a parameter provided as powers of 2
%  METRICS    cell array with selected metrics
%             if not specified, all available metrics are used
%
%  Available metrics: 
%    Step_Total
%    Step_Advance
%    Step_Update
%    LS_Jacobian
%    LS_Setup
%    LS_Solve
%    CD_Total
%    CD_Broad
%    CD_Narrow
%
%  Example usage
%    plot_single('pend_1.csv', true, {'Step_Total', 'CD_Total');
%
%  See also: plot_history

%% ------------------------------------------------------------------------

if nargin > 2
    metrics = varargin{1};
else
    metrics = {'Step_Total', 'Step_Advance', 'Step_Update', 'LS_Jacobian', 'LS_Setup', 'LS_Solve', 'CD_Total', 'CD_Broad', 'CD_Narrow'};
end

%% ------------------------------------------------------------------------
[tnames, tmu, tmd, tsd, tci] = process(filename, metrics);

% Plot results
figure
set(gcf,'defaultTextInterpreter','none');
hold on

ntests = length(tnames);
if double == true
    x = (2.^(0:ntests-1))';
else
    x = 1:ntests;
end

for i = 1:length(metrics)
    errorbar(x, tmu(:,i), tci(:,i), '.-');
end
box on, grid on
ylabel('Time (ms)')
set(gca,'xtick',x)
set(gca,'xticklabel', tnames)
set(gca,'xticklabelrotation',30)
yl = get(gca, 'ylim');
yl(1) = -yl(2)/10;
ylim(yl);
legend(metrics, 'location', 'northwest', 'Interpreter', 'none');
title(filename)
