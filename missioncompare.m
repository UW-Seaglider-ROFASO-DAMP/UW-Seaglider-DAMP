function Diagnose = missioncompare(DT_Output, MI_Output)
% missioncompare
% Seaglider Mission-Sim Comparator for Diagnoser
% Author: Dante Weerasooriya
% Date: April 2026
%
% Data format:
%   Row 1          = variable names
%   Column 1       = time
%   Rows 2..end    = time samples
%   Columns 2..end = variables

%%  Extract variable names

varNames_DT = string(DT_Output(1, :));
varNames_MI = string(MI_Output(1, :));

%% Check variable count (columns)

if size(DT_Output, 2) ~= size(MI_Output, 2)
    error('MissionCompare:VariableCountMismatch', ...
          'Digital Twin and Mission data have different numbers of variables.');
end

%% Check variable names match

if ~isequal(varNames_DT, varNames_MI)
    error('MissionCompare:VariableNameMismatch', ...
          'Variable names do not match between DT and MI data.');
end

%% Extract time vectors (column 1)

time_DT = cell2mat(DT_Output(2:end, 1));
time_MI = cell2mat(MI_Output(2:end, 1));

%% Check time length (rows)

if size(time_DT, 1) ~= size(time_MI, 1)
    error('MissionCompare:TimeMismatch', ...
          'Digital Twin and Mission data have different numbers of time samples.');
end

%% Build DT and MI structs

DT = struct();
MI = struct();

% Loop through each variable column (starting at 1 = time)
for i = 1:size(DT_Output, 2)
    varName = varNames_DT(i);                     % name from row 1
    DT.(varName) = cell2mat(DT_Output(2:end, i)); % data from rows 2 to end
    MI.(varName) = cell2mat(MI_Output(2:end, i));
end


%% TRIMMING LOGIC (middle 60% of dive)

time = time_DT;  % same as MI.time

T = time(end) - time(1);
t0 = time(1);

trimMask = (time > t0 + 0.2*T) & (time < t0 + 0.8*T);

%% Initialize Diagnose fields

Diagnose = struct();
Diagnose.VariableNames = varNames_DT;
Diagnose.Diff = struct();
Diagnose.Failures = struct();
Diagnose.Plots = struct();

%% LOOP THROUGH EACH VARIABLE

for i = 2:length(varNames_DT)   % skip time (column 1)
    
    varName = varNames_DT(i);

    % Extract trimmed data
    DT_trim = DT.(varName)(trimMask);
    MI_trim = MI.(varName)(trimMask);

    % Percent error
    E = abs((MI_trim - DT_trim) ./ DT_trim) * 100;

    % Boolean excursion mask (error > 2%)
    excursionMask = E > 2;

    % Final Diff metric
    Diff = sum(E(excursionMask));

    % Store Diff
    Diagnose.Diff.(varName) = Diff;

    % Threshold logic (Diff > 40%)
    if Diff > 40
        Diagnose.Failures.(varName) = Diff;
    end

    % PLOT DT vs MI FOR THIS VARIABLE
    figure;
    plot(time(trimMask), DT_trim, 'b', 'LineWidth', 1.5); hold on;
    plot(time(trimMask), MI_trim, 'r', 'LineWidth', 1.5);
    xlabel('Time');
    ylabel(varName);
    legend('DT', 'MI');
    title(['DT vs MI: ' varName]);
    grid on;

end

%% SUMMARY OUTPUT TO COMMAND WINDOW

if isempty(fieldnames(Diagnose.Failures))
    fprintf('\n✓ All variables within acceptable limits.\n\n');
else
    fprintf('\n⚠️  FAILURES DETECTED:\n');
    failVars = fieldnames(Diagnose.Failures);
    for k = 1:length(failVars)
        name = failVars{k};
        value = Diagnose.Failures.(name);
        fprintf('   %s exceeded threshold with Diff = %.2f%%\n', name, value);
    end
    fprintf('\n');
end


end









