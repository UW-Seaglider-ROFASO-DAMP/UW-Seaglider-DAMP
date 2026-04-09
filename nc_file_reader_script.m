%% NetCDF to Excel Worksheet
% Author: Henry Hong
% Contact: hmhh@uw.edu
% Last Updated: April 9th, 2026
% IMPORTANT:
% For script to work, the nc_files folders must be downloaded on computer.
% In Student-Seaglider-Center Github
% raw_data > nc_files

%% Clear
clear
clc

%% Toggle Excel Export
runExcelExport = true; % set to true to enable, otherwise false to disable

%% NetCDF OVERVIEW EXAMPLE GUIDE (can delete if preferred)
% Imports example netCDF File for viewer
example_read = 'p1950001example.nc';
example_ncinfo = ncinfo(example_read); % View struct files

% IMPORTANT: Double Click
% Workspace > example_ncinfo > variables > copy & paste name
example_var = ncread(example_read,'time'); % Reads selected variable from dataset

%% Variable Selector (USER EDIT THIS)
% Edit List for selected variables to read
vars = {'time',...
    'eng_elaps_t',...
    'gc_pitch_ctl',...
    'gc_pitch_volts',...
    'gc_pitch_i',...
    'depth'};

%% nc_file_reader function (USER EDIT THIS)
% FOR read_nc_folder function
% allData = read_nc_folder(folder, varNames, maxFiles, startIndex)
%
% Inputs:
%   folder     - path to folder containing .nc files ([] to select manually)
%   varNames   - cell array of variable names
%   maxFiles   - (optional) maximum number of files to read
%   startIndex - (optional) file index to start from (default = 1)
%
% Output:
%   allData - cell array of structs (one per file)
% [] is empty

data = nc_file_reader([],vars,[],[]);  % will prompt folder

%% Excel Import
if runExcelExport
    [file, path] = uiputfile('*.xlsx', 'Save Excel File As');
    Excel_Sheet_Title = fullfile(path, file);
    
    for k = 1:length(data)
        S = data{k};
    
        % Find max length among variables
        maxLen = 0;
        for v = 1:length(vars)
            varName = vars{v};
            if isfield(S, varName)
                maxLen = max(maxLen, length(S.(varName)));
            end
        end
    
        % Build table with padded columns
        tblData = table();
        for v = 1:length(vars)
            varName = vars{v};
            if isfield(S, varName)
                col = S.(varName);
                if length(col) < maxLen
                    col(end+1:maxLen,1) = NaN;  % pad with NaN
                end
                tblData.(varName) = col;
            else
                tblData.(varName) = NaN(maxLen,1);
            end
        end
    
        % Info to show once
        info = {'Filename', S.filename; 'Folder', S.folder};
    
        % Write file/folder info at top
        writecell(info, Excel_Sheet_Title, 'Sheet', ['File_', num2str(k)], 'Range', 'A1');
    
        % Write table starting from row 3
        writetable(tblData, Excel_Sheet_Title, 'Sheet', ['File_', num2str(k)], 'Range', 'A3', 'WriteRowNames', false);
    end
end