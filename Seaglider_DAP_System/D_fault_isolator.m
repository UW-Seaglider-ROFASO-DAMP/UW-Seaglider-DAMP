function [RankedTable, BestMatch, ResultsLibrary] = D_fault_isolator(Mission_Data_Matrix, Wing50_Sim_Matrix, Wing25_Sim_Matrix, Rudder50_Sim_Matrix)
% D_FAULT_ISOLATOR
%
% Compares off-nominal simulated cases against mission data and determines
% which fault case most closely matches the real mission data.
%
% INPUTS:
%   Mission_Data_Matrix  - Actual mission data matrix
%   Wing50_Sim_Matrix    - Digital twin simulation using 50% wing fault coefficients
%   Wing25_Sim_Matrix    - Digital twin simulation using 25% wing fault coefficients
%   Rudder50_Sim_Matrix  - Digital twin simulation using 50% rudder fault coefficients
%
% OUTPUTS:
%   RankedTable     - Table sorted from closest match to worst match
%   BestMatch       - Name of closest matching fault case
%   ResultsLibrary  - Comparator results for each case

%% Case setup

CaseNames = {
    '50% Wing Fault'
    '25% Wing Fault'
    '50% Rudder Fault'
};

SimMatrices = {
    Wing50_Sim_Matrix
    Wing25_Sim_Matrix
    Rudder50_Sim_Matrix
};

numCases = numel(SimMatrices);

finalScores = zeros(numCases, 1);
ResultsLibrary = cell(numCases, 1);

%% Run mission comparator for each off-nominal case

for i = 1:numCases

    fprintf('\nComparing case: %s\n', CaseNames{i});

    % Compare simulated fault case to actual mission data
    ResultsLibrary{i} = missioncompare_version_4(SimMatrices{i}, Mission_Data_Matrix);

    % Pull percent difference data from comparator output
    compStruct = ResultsLibrary{i}.difference;
    vars = fieldnames(compStruct);

    error_sum = 0;
    count = 0;

    for v = 1:numel(vars)
        arr = compStruct.(vars{v});

        if isempty(arr)
            continue;
        end

        mean_error = mean(arr, 'omitnan');

        if ~isnan(mean_error) && isfinite(mean_error)
            error_sum = error_sum + mean_error;
            count = count + 1;
        end
    end

    % Lower score means closer match to mission data
    if count > 0
        finalScores(i) = error_sum / count;
    else
        finalScores(i) = Inf;
    end

end

%% Create ranked table

RankedTable = table(CaseNames, finalScores, ...
    'VariableNames', {'Fault_Case', 'Average_Percent_Error'});

RankedTable = sortrows(RankedTable, 'Average_Percent_Error', 'ascend');

%% Best match

BestMatch = RankedTable.Fault_Case{1};

fprintf('\n====================================\n');
fprintf('FAULT ISOLATION COMPLETE\n');
fprintf('Best Match: %s\n', BestMatch);
fprintf('Average Percent Error: %.3f\n', RankedTable.Average_Percent_Error(1));
fprintf('====================================\n\n');

end