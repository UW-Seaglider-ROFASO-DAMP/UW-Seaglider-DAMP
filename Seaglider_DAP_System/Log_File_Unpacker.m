function Selected_Matrix = Log_File_Unpacker(filename)
% LOG_FILE_UNPACKER
%
% Column 1 = variable names
% Columns 2:end = values
%
% USER CAN SELECT WHICH VARIABLES TO KEEP

%% =========================================================
% SELECT VARIABLES TO KEEP HERE
% ==========================================================

selectedVariables = { ...
    '$RHO', ...
    '$C_PITCH', ...
    '$C_ROLL_DIVE', ...
    '$C_ROLL_CLIMB', ...
    '$MHEAD_RNG_PITCHd_Wd', ...
    '$C_VBD', ...
    
    };

%% OPEN FILE

fid = fopen(filename,'r');

if fid == -1
    error('Could not open file: %s', filename);
end

%% STORAGE

Log_Matrix = {};

%% READ FILE

while ~feof(fid)

    line = strtrim(fgetl(fid));

    if isempty(line)
        continue;
    end

    %% =====================================================
    % CASE 1: COLON-SEPARATED
    %% =====================================================

    if contains(line, ':') && ~startsWith(line, '$')

        splitLine = strsplit(line, ':');

        paramName  = strtrim(splitLine{1});
        paramValue = strtrim(strjoin(splitLine(2:end), ':'));

        % ONLY KEEP SELECTED VARIABLES
        if ismember(paramName, selectedVariables)

            row = [{paramName}, {convertValue(paramValue)}];

            Log_Matrix = appendRow(Log_Matrix, row);

        end

    %% =====================================================
    % CASE 2: COMMA-SEPARATED
    %% =====================================================

    elseif contains(line, ',')

        splitLine = strsplit(line, ',');

        paramName = strtrim(splitLine{1});
        paramVals = splitLine(2:end);

        % ONLY KEEP SELECTED VARIABLES
        if ismember(paramName, selectedVariables)

            row = cell(1, length(paramVals) + 1);

            row{1} = paramName;

            for k = 1:length(paramVals)

                row{k+1} = convertValue(strtrim(paramVals{k}));

            end

            Log_Matrix = appendRow(Log_Matrix, row);

        end

    end

end

%% CLOSE FILE

fclose(fid);

%% OUTPUT

Selected_Matrix = Log_Matrix;

%% DISPLAY SUMMARY

fprintf('\n====================================\n');
fprintf('Selected Log Variables Loaded\n');
fprintf('====================================\n');

fprintf('Rows Loaded: %d\n', size(Selected_Matrix,1));
fprintf('Columns Loaded: %d\n', size(Selected_Matrix,2));

fprintf('\nVariables Included:\n');

for i = 1:size(Selected_Matrix,1)
    fprintf('%2d : %s\n', i, string(Selected_Matrix{i,1}));
end

fprintf('====================================\n');

end

%% =========================================================
% HELPER FUNCTION: CONVERT VALUES
%% =========================================================

function value = convertValue(inputValue)

numericValue = str2double(inputValue);

if isnan(numericValue)
    value = inputValue;
else
    value = numericValue;
end

end

%% =========================================================
% HELPER FUNCTION: APPEND UNEVEN ROWS
%% =========================================================

function MatrixOut = appendRow(MatrixIn, row)

currentCols = size(MatrixIn,2);
newCols = length(row);

maxCols = max(currentCols, newCols);

% Expand existing matrix
if currentCols < maxCols && ~isempty(MatrixIn)

    MatrixIn(:, currentCols+1:maxCols) = {[]};

end

% Expand new row
if newCols < maxCols

    row(newCols+1:maxCols) = {[]};

end

MatrixOut = [MatrixIn; row];

end