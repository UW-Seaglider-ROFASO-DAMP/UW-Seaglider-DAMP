function Diagnose = missioncompare_version_4(DT_Output, MI_Output)
% MISSIONCOMPARE_VERSION_4
% Seaglider Mission-Sim Comparator for Diagnoser
%
% INPUTS
%   DT_Output : Nominal/Digital Twin output matrix with headers in row 1
%   MI_Output : Mission data matrix with headers in row 1
%
% OUTPUT
%   Diagnose  : struct containing difference metrics, failures, plots, and
%               metadata.
%
% NOTES
%   This version is updated to handle mission/log variable names such as
%   '$RHO', '$C_PITCH', and '$MHEAD_RNG_PITCHd_Wd'. Those are valid matrix
%   headers, but they are NOT valid MATLAB struct field names. This code
%   keeps the original names for reporting and creates safe internal field
%   names using matlab.lang.makeValidName().
%
%   Example:
%       '$RHO' becomes 'x_RHO' internally.
%
%   The original-to-safe name mapping is saved in Diagnose.NameMap.

%% 0. Input checks
if nargin < 2
    error('MissionCompare:NotEnoughInputs', ...
        'missioncompare_version_4 requires DT_Output and MI_Output.');
end

if ~iscell(DT_Output) || ~iscell(MI_Output)
    error('MissionCompare:InputTypeError', ...
        'DT_Output and MI_Output must be cell matrices with headers in row 1.');
end

if size(DT_Output, 1) < 2 || size(MI_Output, 1) < 2
    error('MissionCompare:InputSizeError', ...
        'DT_Output and MI_Output must include a header row and at least one data row.');
end

%% 1. Extract original variable names from row 1
varNames_DT_original = cleanHeaderRow(DT_Output(1, :));
varNames_MI_original = cleanHeaderRow(MI_Output(1, :));

%% 2. Basic consistency checks
if size(DT_Output, 2) ~= size(MI_Output, 2)
    error('MissionCompare:VariableCountMismatch', ...
        ['Digital Twin and Mission data have different numbers of variables. ', ...
         'DT has %d columns and MI has %d columns.'], ...
        size(DT_Output, 2), size(MI_Output, 2));
end

if ~isequal(varNames_DT_original, varNames_MI_original)
    mismatchIdx = find(varNames_DT_original ~= varNames_MI_original, 1, 'first');
    error('MissionCompare:VariableNameMismatch', ...
        ['Variable names do not match between DT and MI data. ', ...
         'First mismatch is column %d: DT = "%s", MI = "%s".'], ...
        mismatchIdx, varNames_DT_original(mismatchIdx), varNames_MI_original(mismatchIdx));
end

%% 3. Create safe MATLAB struct field names
% Headers like '$RHO' are invalid struct fields. makeUniqueStrings prevents
% duplicate safe names if two original headers clean to the same field name.
varNames_safe = matlab.lang.makeValidName(varNames_DT_original, ...
    'ReplacementStyle', 'underscore', 'Prefix', 'x_');
varNames_safe = matlab.lang.makeUniqueStrings(varNames_safe);

%% 4. Find time column
% Prefer eng_elaps_t because it is usually mission-relative time. If missing,
% fall back to eng_elaps_t_0000, then column 1.
timeIdx = find(strcmp(varNames_DT_original, 'eng_elaps_t'), 1, 'first');
if isempty(timeIdx)
    timeIdx = find(strcmp(varNames_DT_original, 'eng_elaps_t_0000'), 1, 'first');
end
if isempty(timeIdx)
    timeIdx = 1;
    warning('MissionCompare:NoNamedTimeColumn', ...
        'No eng_elaps_t or eng_elaps_t_0000 column found. Using column 1 as time.');
end

%% 5. Convert all data columns to numeric arrays
DT_numeric = cellMatrixToNumeric(DT_Output(2:end, :));
MI_numeric = cellMatrixToNumeric(MI_Output(2:end, :));

%% 6. Normalize to common row count
len_DT = size(DT_numeric, 1);
len_MI = size(MI_numeric, 1);
minLen = min(len_DT, len_MI);

DT_numeric = DT_numeric(1:minLen, :);
MI_numeric = MI_numeric(1:minLen, :);

time_DT = DT_numeric(:, timeIdx);
time_MI = MI_numeric(:, timeIdx);

if all(isnan(time_DT)) || all(isnan(time_MI))
    error('MissionCompare:BadTimeColumn', ...
        'Selected time column "%s" could not be converted to numeric data.', ...
        varNames_DT_original(timeIdx));
end

% Use DT time as reference.
time = time_DT;

% Total mission duration and start time used for middle-60-percent trim.
t0 = time(1);
T = time(end) - time(1);

if ~isfinite(T) || T <= 0
    warning('MissionCompare:BadTimeSpan', ...
        'Time span is invalid or non-positive. Trim will use all valid data.');
end

%% 7. Build DT and MI structs using safe field names
DT = struct();
MI = struct();

for i = 1:length(varNames_safe)
    fieldName = char(varNames_safe(i));
    DT.(fieldName) = DT_numeric(:, i);
    MI.(fieldName) = MI_numeric(:, i);
end

%% 8. Initialize Diagnose output
Diagnose = struct();

Diagnose.VariableNamesOriginal = varNames_DT_original;
Diagnose.VariableNamesSafe     = varNames_safe;
Diagnose.TimeVariableOriginal  = varNames_DT_original(timeIdx);
Diagnose.TimeVariableSafe      = varNames_safe(timeIdx);

Diagnose.NameMap = table(varNames_DT_original(:), varNames_safe(:), ...
    'VariableNames', {'OriginalName', 'SafeFieldName'});

Diagnose.Diff         = struct();
Diagnose.Failures     = struct();
Diagnose.Plots        = struct();
Diagnose.Excursions   = struct();
Diagnose.Count        = struct();
Diagnose.Excess       = struct();
Diagnose.WindowScores = struct();
Diagnose.RemovedZeros = struct();
Diagnose.difference   = struct();
Diagnose.Tolerances   = struct();

%% 9. Define per-variable tolerances
% These keys use the ORIGINAL header names, not the safe field names.
toleranceMap = containers.Map( ...
    {'eng_pitchAng', 'eng_rollAng', 'eng_head', 'eng_depth'}, ...
    [7.5,           187.5,         13.5,       14.5] ...
);

defaultTolerance = 2.0;

%% 10. Define window settings
windowSize       = 20;
failureThreshold = 40;

Diagnose.Settings.windowSize       = windowSize;
Diagnose.Settings.failureThreshold = failureThreshold;
Diagnose.Settings.defaultTolerance = defaultTolerance;
Diagnose.Settings.trimMiddle60     = true;

%% 11. Create plot folder
saveFolder = fullfile(pwd, 'MS_Comparator_Plots');
if ~exist(saveFolder, 'dir')
    mkdir(saveFolder);
end
Diagnose.PlotFolder = saveFolder;

%% 12. Main comparison loop
for i = 1:length(varNames_safe)
    % Skip time column.
    if i == timeIdx
        continue;
    end

    originalName = varNames_DT_original(i);
    fieldName    = char(varNames_safe(i));

    DT_raw = DT.(fieldName);
    MI_raw = MI.(fieldName);

    % Remove NaN/Inf first.
    finiteMask = isfinite(DT_raw) & isfinite(MI_raw) & isfinite(time);

    % Remove samples where either signal is zero to avoid division by zero.
    % If this removes everything, the fallback below uses absolute error.
    nonzeroMask = (DT_raw ~= 0) & (MI_raw ~= 0);
    validMask = finiteMask & nonzeroMask;

    numRemoved = sum(~validMask);
    Diagnose.RemovedZeros.(fieldName) = sum(finiteMask & ~nonzeroMask);

    DT_clean = DT_raw(validMask);
    MI_clean = MI_raw(validMask);
    t_clean  = time(validMask);

    % Fallback: if zero removal removed all samples, keep finite samples and
    % compare by absolute difference rather than percent difference.
    useAbsoluteDifference = false;
    if isempty(DT_clean) || isempty(MI_clean)
        validMask = finiteMask;
        DT_clean = DT_raw(validMask);
        MI_clean = MI_raw(validMask);
        t_clean  = time(validMask);
        useAbsoluteDifference = true;
    end

    if isempty(DT_clean) || isempty(MI_clean)
        Diagnose = storeEmptyResult(Diagnose, fieldName);
        continue;
    end

    % Trim to middle 60% of mission when time span is valid.
    if isfinite(T) && T > 0
        trimMask = (t_clean > t0 + 0.2*T) & (t_clean < t0 + 0.8*T);

        % If the trim removes everything, use all cleaned data instead.
        if any(trimMask)
            DT_trim = DT_clean(trimMask);
            MI_trim = MI_clean(trimMask);
            t_trim  = t_clean(trimMask);
        else
            DT_trim = DT_clean;
            MI_trim = MI_clean;
            t_trim  = t_clean;
        end
    else
        DT_trim = DT_clean;
        MI_trim = MI_clean;
        t_trim  = t_clean;
    end

    if isempty(DT_trim) || isempty(MI_trim)
        Diagnose = storeEmptyResult(Diagnose, fieldName);
        continue;
    end

    % Get tolerance using original variable name.
    if isKey(toleranceMap, char(originalName))
        tolerance = toleranceMap(char(originalName));
    else
        tolerance = defaultTolerance;
    end
    Diagnose.Tolerances.(fieldName) = tolerance;

    % Compute difference.
    if useAbsoluteDifference
        % This branch is only used when percent difference is impossible
        % because all values are zero in at least one signal.
        E = abs(MI_trim - DT_trim);
    else
        E = abs((MI_trim - DT_trim) ./ DT_trim) * 100;
    end

    Diagnose.difference.(fieldName) = E;

    excursionMask = E > tolerance;
    Diagnose.Count.(fieldName) = sum(excursionMask);

    excessDifference = max(E - tolerance, 0);
    Diagnose.Excess.(fieldName) = sum(excessDifference);

    %% 12.1 Windowed binary detector
    errorMask = excursionMask;
    numSamples = length(errorMask);
    numWindows = ceil(numSamples / windowSize);

    windowErrorRates = zeros(numWindows, 1);
    windowFlags = false(numWindows, 1);

    for w = 1:numWindows
        idxStart = (w - 1)*windowSize + 1;
        idxEnd = min(w*windowSize, numSamples);

        windowErrors = errorMask(idxStart:idxEnd);
        windowErrorRates(w) = (sum(windowErrors) / length(windowErrors)) * 100;
        windowFlags(w) = windowErrorRates(w) > failureThreshold;
    end

    Diagnose.WindowScores.(fieldName) = windowErrorRates;

    %% 12.2 Run-to-end failure rule
    fail = false;
    for w = 1:numWindows
        if windowFlags(w)
            if w == numWindows || all(windowFlags(w:end))
                fail = true;
                break;
            end
        end
    end

    if isempty(windowErrorRates)
        Diagnose.Diff.(fieldName) = NaN;
    else
        Diagnose.Diff.(fieldName) = max(windowErrorRates);
    end

    if fail
        Diagnose.Failures.(fieldName) = max(windowErrorRates);
    end

    %% 12.3 Excursion intervals
    idx = find(excursionMask);
    intervals = [];

    if ~isempty(idx)
        d = diff(idx);
        breaks = [0; find(d > 1); length(idx)];

        for b = 1:length(breaks)-1
            seg = idx(breaks(b)+1 : breaks(b+1));
            intervals = [intervals; t_trim(seg(1)) t_trim(seg(end))]; %#ok<AGROW>
        end
    end

    Diagnose.Excursions.(fieldName) = intervals;

    %% 12.4 Plot DT vs MI
    fig = figure('Visible', 'off');

    t_plot  = t_trim(:);
    DT_plot = DT_trim(:);
    MI_plot = MI_trim(:);

    plot(t_plot, DT_plot, 'b-s', 'LineWidth', 1.5, ...
        'MarkerSize', 3, 'MarkerFaceColor', 'w', 'MarkerEdgeColor', 'b');
    hold on;

    plot(t_plot, MI_plot, 'r-o', 'LineWidth', 1.5, ...
        'MarkerSize', 2, 'MarkerFaceColor', 'w', 'MarkerEdgeColor', 'r');

    if ~useAbsoluteDifference
        lower = DT_plot * (1 - tolerance/100);
        upper = DT_plot * (1 + tolerance/100);
        plot(t_plot, lower, 'k--', 'LineWidth', 1);
        plot(t_plot, upper, 'k--', 'LineWidth', 1);
        legend('DT', 'MI', sprintf('-%g%%', tolerance), ...
            sprintf('+%g%%', tolerance), 'Location', 'best');
    else
        legend('DT', 'MI', 'Location', 'best');
    end

    xlabel(sprintf('Time: %s', char(varNames_DT_original(timeIdx))), ...
        'Interpreter', 'none');
    ylabel(char(originalName), 'Interpreter', 'none');
    title(sprintf('DT vs MI: %s', char(originalName)), 'Interpreter', 'none');
    grid on;

    safePlotName = matlab.lang.makeValidName(char(originalName), ...
        'ReplacementStyle', 'underscore', 'Prefix', 'x_');
    filePath = fullfile(saveFolder, safePlotName + ".png");

    saveas(fig, filePath);
    close(fig);

    Diagnose.Plots.(fieldName) = filePath;
end

%% 13. Summary output
if isempty(fieldnames(Diagnose.Failures))
    fprintf('\nAll variables within acceptable windowed limits.\n\n');
else
    fprintf('\nFAILURES DETECTED (window-based Diff > %.2f):\n', failureThreshold);
    failVars = fieldnames(Diagnose.Failures);

    for k = 1:length(failVars)
        safeName = failVars{k};
        originalIdx = find(varNames_safe == string(safeName), 1, 'first');

        if ~isempty(originalIdx)
            displayName = char(varNames_DT_original(originalIdx));
        else
            displayName = safeName;
        end

        value = Diagnose.Failures.(safeName);
        fprintf('   %s failed with Diff (max window score) = %.2f\n', ...
            displayName, value);
    end
    fprintf('\n');
end

end

%% LOCAL HELPER FUNCTIONS
function headers = cleanHeaderRow(headerCells)
% Convert a cell header row into a clean string array.
headers = strings(1, numel(headerCells));

for j = 1:numel(headerCells)
    value = headerCells{j};

    if isstring(value)
        headers(j) = strtrim(value(1));
    elseif ischar(value)
        headers(j) = strtrim(string(value));
    elseif iscell(value)
        headers(j) = cleanHeaderRow(value);
    else
        headers(j) = strtrim(string(value));
    end
end
end

function numericData = cellMatrixToNumeric(cellData)
% Convert a cell matrix of mixed numeric/string data into a numeric matrix.
[nRows, nCols] = size(cellData);
numericData = NaN(nRows, nCols);

for r = 1:nRows
    for c = 1:nCols
        value = cellData{r, c};

        if isnumeric(value)
            if isempty(value)
                numericData(r, c) = NaN;
            else
                numericData(r, c) = double(value(1));
            end
        elseif islogical(value)
            numericData(r, c) = double(value(1));
        elseif isstring(value) || ischar(value)
            temp = str2double(string(value));
            if ~isnan(temp)
                numericData(r, c) = temp;
            else
                numericData(r, c) = NaN;
            end
        elseif iscell(value)
            try
                numericData(r, c) = cellMatrixToNumeric(value);
            catch
                numericData(r, c) = NaN;
            end
        else
            numericData(r, c) = NaN;
        end
    end
end
end

function Diagnose = storeEmptyResult(Diagnose, fieldName)
% Store consistent empty values when a variable cannot be compared.
Diagnose.Diff.(fieldName)         = NaN;
Diagnose.Count.(fieldName)        = 0;
Diagnose.Excess.(fieldName)       = NaN;
Diagnose.WindowScores.(fieldName) = [];
Diagnose.Excursions.(fieldName)   = [];
Diagnose.Plots.(fieldName)        = '';
Diagnose.difference.(fieldName)   = [];
end
