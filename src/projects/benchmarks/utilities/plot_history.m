function [] = plot_history(filenames, varargin)
%plot_history plot bennchmark history
%
%  Usage: plot_history ( FILENAMES )
%         plot_history ( FILENAMES , METRICS )
%
%  FILENAMES  cell array with CSV benchmark results
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
%    d = dir('pend*.csv');
%    plot_history({d.name});
%
%  See also: plot_single


%filenames = {'pend_1.csv', 'pend_2.csv', 'pend_3.csv', 'pend_4.csv'};
%filenames = {'mixerSMC_1.csv', 'mixerSMC_2.csv'};
%filenames = {'hmmwv_1.csv', 'hmmwv_2.csv', 'hmmwv_3.csv', 'hmmwv_4.csv', 'hmmwv_5.csv'};

%% ------------------------------------------------------------------------

if nargin > 1
    metrics = varargin{1};
else
    metrics = {'Step_Total', 'Step_Advance', 'Step_Update', 'LS_Jacobian', 'LS_Setup', 'LS_Solve', 'CD_Total', 'CD_Broad', 'CD_Narrow'};
end

%% ------------------------------------------------------------------------

nfiles = length(filenames);
nmetrics = length(metrics);

tnames = cell(nfiles,1);
tmu = cell(nfiles,1);
tmd = cell(nfiles,1);
tsd = cell(nfiles,1);
tci = cell(nfiles,1);

for i = 1:nfiles
	[tnames{i}, tmu{i}, tmd{i}, tsd{i}, tci{i}] = process(filenames{i}, metrics);
end

% First file is assumed to contain the reference.
% Ensure all other files include the reference benchmark tests.
for i = 2:nfiles
	check = ismember(tnames{1}, tnames{i});
	if ~all(check)
		fprintf('Inconsistent set of benchmarks in file %s\n\n\n', filenames{i});
		return;
	end
end

titlestring = sprintf('Reference file: %s\n', filenames{1});

for i = 1:nfiles
	[p, n, e] = fileparts(filenames{i});
	filenames{i} = n;
end

% Number of distinct tests in each file
ntests = length(tnames{1});

% Mean, std.dev., and confidence interval for the specified metrics
mu = reshape(cell2mat(tmu), ntests, nfiles, nmetrics);
sd = reshape(cell2mat(tsd), ntests, nfiles, nmetrics);
ci = reshape(cell2mat(tci), ntests, nfiles, nmetrics);

% Figure width, subplot grid, location of legend
if (nmetrics == 9)
    width = 1600;
    num_rows = 3;
    num_cols = 3;
    i_legend = 3;
else
    width = 640;
    num_rows = nmetrics;
    num_cols = 1;
    i_legend = 1;
end

figure('position',[80 180 width 920]);
set(0,'defaultTextInterpreter','none');

for i = 1:nmetrics
   hS = subplot(num_rows,num_cols,i);
   %plot(mu(:,:,i)', '.-')
   hold on
   for j = 1:ntests
     errorbar(mu(j,:,i), ci(j,:,i), '.-')
   end
   box on, grid on
   xlim([0.5, nfiles+0.5])
   set(gca,'TickLabelInterpreter','none')
   set(gca,'xtick', 1:nfiles)
   set(gca,'xticklabel', filenames)
   set(gca,'xticklabelrotation', 30)
   ylabel('Time (ms)')
   title(metrics{i})
   if (i == i_legend)
       hL = legend(tnames{1});
       pos_S = get(hS, 'position');
       pos_L = get(hL, 'position');
   end
%    if ntests > 1
%        hL = legend(tnames{1},'Location','East');
%        if i ~= i_legend ; set(hL, 'Visible', 'off'); end
%    end
end

set(hL,'Position',[pos_S(1)+pos_S(3)-pos_L(3)...
    pos_L(2)+pos_L(4)+0.015...
    pos_L(3)...
    0.5*pos_L(4)]);

suptitle(titlestring)

