function [tnames, tmu, tmd, tsd, tci] = process(filename, metrics)

% Read CSV with benchmark results as a Matlab table
a = readtable(filename,'ReadVariableNames',1);

% Check that the specified metrics are valid
nmetrics = length(metrics);
for i = 1:nmetrics
    assert(ismember(metrics{i}, a.Properties.VariableNames), '"%s" is not a valid metric.', metrics{i})
end

% Find sub-tables corresponding to mean, median, and stddev values
amu = a(contains(a.name,'_mean'),:);
amd = a(contains(a.name,'_median'),:);
asd = a(contains(a.name,'_stddev'),:);

% Extract names of unique tests
tnames = cellfun(@(x) extractBefore(x,'/'), amu.name, 'un', 0);
ntests = length(tnames);

% Extract strings with test repetitions
sreps = cellfun(@(x) extractBefore(extractAfter(x,':'),'_'), amu.name, 'un', 0);
reps = cell2mat(cellfun(@str2num, sreps, 'un', 0));

% Extract mean, median, and stddev for specified metrics
tmu = zeros(ntests, nmetrics);
tmd = zeros(ntests, nmetrics);
tsd = zeros(ntests, nmetrics);
for i = 1:nmetrics
    tmu(:,i) = amu.(metrics{i});
    tmd(:,i) = amd.(metrics{i});
    tsd(:,i) = asd.(metrics{i});
end

% Calculate 99% confidence interval   z*s/sqrt(n)
z = 2.576;
tci = repmat(z./sqrt(reps), 1, size(tsd, 2)) .* tsd;
