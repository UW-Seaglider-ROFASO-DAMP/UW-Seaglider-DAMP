function Diagnose = missioncompare(DT_Output, MI_Output)
% missioncompare
% Seaglider Mission-Sim Comparator for Diagnoser
% Author: Dante Weerasooriya
% Date: April 2026

%% Extract variable names
varNames_DT = string(DT_Output(1, :));
varNames_MI = string(MI_Output(1, :));

%% Check variable count
if size(DT_Output, 2) ~= size(MI_Output, 2)
    error('MissionCompare:VariableCountMismatch', ...
          'Digital Twin and Mission data have different numbers of variables.');
end

%% Check variable names match
if ~isequal(varNames_DT, varNames_MI)
    error('MissionCompare:VariableNameMismatch', ...
          'Variable names do not match between DT and MI data.');
end

%% Extract time vectors
time_DT = cell2mat(DT_Output(2:end, 1));
time_MI = cell2mat(MI_Output(2:end, 1));

%% Check time length
if size(time_DT, 1) ~= size(time_MI, 1)
    error('MissionCompare:TimeMismatch', ...
          'Digital Twin and Mission data have different numbers of time samples.');
end

%% Build DT and MI structs
DT = struct();
MI = struct();

for i = 1:size(DT_Output, 2)
    varName = varNames_DT(i);
    DT.(varName) = cell2mat(DT_Output(2:end, i));
    MI.(varName) = cell2mat(MI_Output(2:end, i));
end

%% TRIMMING LOGIC
time = time_DT;
T = time(end) - time(1);
t0 = time(1);
trimMask = (time > t0 + 0.2*T) & (time < t0 + 0.8*T);

%% Initialize Diagnose struct
Diagnose = struct();
Diagnose.VariableNames = varNames_DT;
Diagnose.Diff = struct();
Diagnose.Failures = struct();
Diagnose.Plots = struct();
Diagnose.Excursions = struct();

%% Create folder for plots
saveFolder = fullfile(pwd, 'MS_Comparator_Plots');
if ~exist(saveFolder, 'dir')
    mkdir(saveFolder);
end

%% LOOP THROUGH VARIABLES
for i = 2:length(varNames_DT)   % skip time
    varName = varNames_DT(i);

    DT_trim = DT.(varName)(trimMask);
    MI_trim = MI.(varName)(trimMask);
    t_trim = time(trimMask);

    %% Percent error
    E = abs((MI_trim - DT_trim) ./ DT_trim) * 100;

    %% Excursion mask
    excursionMask = E > 2;

    %% Final Diff metric
    Diff = sum(E(excursionMask));
    Diagnose.Diff.(varName) = Diff;

    %% Threshold logic
    if Diff > 40
        Diagnose.Failures.(varName) = Diff;
    end

    %% Record excursion time intervals
    idx = find(excursionMask);
    intervals = [];
    if ~isempty(idx)
        d = diff(idx);
        breaks = [0; find(d > 1); length(idx)];
        for b = 1:length(breaks)-1
            seg = idx(breaks(b)+1 : breaks(b+1));
            intervals = [intervals; t_trim(seg(1)) t_trim(seg(end))];
        end
    end
    Diagnose.Excursions.(varName) = intervals;

    %% PLOT (hidden)
    fig = figure('Visible','off');
    plot(t_trim, DT_trim, 'b', 'LineWidth', 1.5); hold on;
    plot(t_trim, MI_trim, 'r', 'LineWidth', 1.5);

    % Error bands ±2%
    lower = DT_trim * 0.98;
    upper = DT_trim * 1.02;
    plot(t_trim, lower, 'k--', 'LineWidth', 1);
    plot(t_trim, upper, 'k--', 'LineWidth', 1);

    xlabel('Time');
    ylabel(varName);
    legend('DT','MI','-2%','+2%');
    title(['DT vs MI: ' varName]);
    grid on;

    % Save plot
    filePath = fullfile(saveFolder, varName + ".png");
    saveas(fig, filePath);
    close(fig);

    Diagnose.Plots.(varName) = filePath;
end

%% SUMMARY OUTPUT
if isempty(fieldnames(Diagnose.Failures))
    fprintf('\n✓ All variables within acceptable limits.\n\n');
else
    fprintf('\n ⚠  FAILURES DETECTED:\n');
    failVars = fieldnames(Diagnose.Failures);
    for k = 1:length(failVars)
        name = failVars{k};
        value = Diagnose.Failures.(name);
        fprintf('   %s exceeded threshold with Diff = %.2f%%\n', name, value);
    end
    fprintf('\n');
end

end









