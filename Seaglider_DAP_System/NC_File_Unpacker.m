function SelectedMatrix = NC_File_Unpacker(filename)
% NC_FILE_UNPACKER
%
% Reads selected variables from a NetCDF (.nc) file.
% First row of output is variable names.
% Numeric data starts on row 2.
% Shorter variables are repeated to match the longest variable length.

%% SELECT VARIABLES HERE

selectedNames = {...
    'eng_elaps_t_0000', ...
    'eng_elaps_t', ...
    'eng_depth', ...
    'eng_head', ...
    'eng_pitchAng', ...
    'eng_rollAng', ...
    'eng_pitchCtl', ...
    'eng_rollCtl', ...
    'eng_vbdCC', ...
    'temperature', ...
    'log_RHO', ...
    'pressure', ...
    'density', ...
    'vert_speed_gsm', ...
    
    
    };

%% LOAD FILE INFO

info = ncinfo(filename);
allVarNames = {info.Variables.Name};

%% READ SELECTED VARIABLES

dataCell = {};
headers = {};
rowLengths = [];

for i = 1:length(selectedNames)

    variableName = selectedNames{i};

    if ~ismember(variableName, allVarNames)
        warning('Variable "%s" not found. Skipping.', variableName);
        continue;
    end

    data = ncread(filename, variableName);

    % Force into a column vector
    data = data(:);

    dataCell{end+1} = data;
    headers{end+1} = variableName;
    rowLengths(end+1) = length(data);

end

%% CHECK THAT DATA WAS FOUND

if isempty(dataCell)
    error('No selected variables were found in the .nc file.');
end

%% FIND LONGEST ROW LENGTH

targetLength = max(rowLengths);

%% REPEAT SHORTER VARIABLES TO MATCH LONGEST LENGTH

for i = 1:length(dataCell)

    currentData = dataCell{i};
    currentLength = length(currentData);

    if currentLength < targetLength

        repeatFactor = ceil(targetLength / currentLength);
        repeatedData = repmat(currentData, repeatFactor, 1);

        dataCell{i} = repeatedData(1:targetLength);

        warning('Variable "%s" length %d repeated to match %d rows.', ...
            headers{i}, currentLength, targetLength);

    end

end

%% BUILD OUTPUT MATRIX WITH HEADER ROW

NumericMatrix = cell2mat(dataCell);

SelectedMatrix = [headers; num2cell(NumericMatrix)];

%% DISPLAY SUMMARY

fprintf('\n====================================\n');
fprintf('NC File Successfully Parsed\n');
fprintf('====================================\n');
fprintf('Rows Loaded: %d numeric rows + 1 header row\n', targetLength);
fprintf('Columns Loaded: %d\n', length(headers));

fprintf('\nSelected Variables Used:\n');

for i = 1:length(headers)
    fprintf('%2d : %s\n', i, headers{i});
end

fprintf('====================================\n');

end