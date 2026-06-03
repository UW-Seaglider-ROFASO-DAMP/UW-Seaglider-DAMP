%% SEAGLIDER D.A.P. system 

% The goal of this code is to diagnose and isolate a fault on a seaglider

%% Inputs
% Log and Eng files from desired mission

%% Outputs
% fault diagnose of wing or rudder and percentage loss

%% Authors
% Jordan Cummings, Henry Hong, Holland Kanter, Letizia Laura, Edward Park, 
% Oleksiy Polyakov, Geenadie Rathnayake, Joshua Rolfe, Mak Sukimoto, 
% Dante Weerasooriya, Kyle Wittcoff

%% Updated on
% 05/08/2026

%% Files required
%% User Inputs
% log_files, eng_files

%% D.A.P. Folder files
% DAP_Main.m, dynamicsV3.m,  FaultIsolator.m, log_file_unpacker.m,
% missioncompare.m, nc_file_reader_function.m, nc_file_reader_script.m,
% parameters.m

clear
clc

%% =========================================================
% SELECT LOG FILE
%% ==========================================================

logMsg = msgbox( ...
    'Please Select Log File', ...
    'Log File Selection', ...
    'modal');

uiwait(logMsg);

[logFile, logPath] = uigetfile( ...
    {'*.log','Log Files (*.log)'}, ...
    'Select Log File');

if isequal(logFile,0)
    error('No Log file selected.');
end

logFullPath = fullfile(logPath, logFile);

%% =========================================================
% SELECT NC FILE
%% ==========================================================

ncMsg = msgbox( ...
    'Please Select NC File', ...
    'NC File Selection', ...
    'modal');

uiwait(ncMsg);

[ncFile, ncPath] = uigetfile( ...
    {'*.nc','NetCDF Files (*.nc)'}, ...
    'Select NC File');

if isequal(ncFile,0)
    error('No NC file selected.');
end

ncFullPath = fullfile(ncPath, ncFile);

% start timer
tic

%% ==========================================================
% UNPACK Log FILES
%% ==========================================================

Log_Matrix = Log_File_Unpacker(logFullPath);

NC_Matrix = NC_File_Unpacker(ncFullPath); 

% Mission Data Matrix

Mission_Data_Matrix = Create_Mission_Data_Matrix(Log_Matrix, NC_Matrix);

% Columns to extract
cols = [2 3 4 5 6 7 10];

% Create compare matrix
Mission_Data_Matrix_Compare = Mission_Data_Matrix(:, cols);

% Fault Deviation Gain

prompt = {'Enter Fault Deviation Gain (%) :'};
dlgtitle = 'Fault Deviation Gain Input';
dims = [1 35];
definput = {'0'}; % default value

answer = inputdlg(prompt, dlgtitle, dims, definput);

if isempty(answer)
    error('No Fault Deviation Gain entered.');
end

% Convert user input to number
Fault_Deviation_Gain = str2double(answer{1});

if isnan(Fault_Deviation_Gain)
    error('Invalid Fault Deviation Gain entered.');
end

% Create multiplier
Fault_Multiplier = 1 + (Fault_Deviation_Gain / 100);

% Apply gain ONLY to columns 3:end of the compare matrix
Mission_Data_Matrix_Faulted = Mission_Data_Matrix_Compare;

Mission_Data_Matrix_Faulted(2:end, 3:end) = ...
    cellfun(@(x) x * Fault_Multiplier, ...
    Mission_Data_Matrix_Compare(2:end, 3:end), ...
    'UniformOutput', false);


%% Digital Twin

Nominal_Sim_Matrix = Digital_Twin(Mission_Data_Matrix, coefficientsV4_nominal, parametersV5);

Nominal_Sim_Matrix_Compare = Nominal_Sim_Matrix(:, cols); 

%% ==========================================================
%% Mission - Nominal Simulation Comparator
%% ==========================================================

Mis_Nom_results = missioncompare_detector(Nominal_Sim_Matrix_Compare, Mission_Data_Matrix_Faulted);


if isempty(fieldnames(Mis_Nom_results.Failures))
    fprintf('\nNo sustained failures detected.\n');
    fprintf('Seaglider is in a nominal state.\n');

    return;   % Ends DAP_Main here

else
    fprintf('\nSustained failures detected.\n');
    fprintf('Beginning off-nominal fault simulations...\n');

    % Continue into off-nominal simulations below
end


%% ==========================================================
%% Fault Iterator
%% ==========================================================

Wing_50_Sim_Matrix = Digital_Twin(Mission_Data_Matrix, coefficientsV4_50wing, parametersV5);

Wing_25_Sim_Matrix = Digital_Twin(Mission_Data_Matrix, coefficientsV4_25wing, parametersV5);

Rudder_50_Sim_Matrix = Digital_Twin(Mission_Data_Matrix, coefficientsV4_50rudder, parametersV5);

%% ==========================================================
%% Fault Isolator
%% ==========================================================
[RankingTable, BestMatch] = D_fault_isolator(Mission_Data_Matrix, Wing_25_Sim_Matrix, Wing_50_Sim_Matrix, Rudder_50_Sim_Matrix);
    
% end timer
runtime = toc;

    % FINAL REPORTING
    fprintf('\n============================================================\n');
    fprintf('FINAL DAMAGE ASSESSMENT REPORT\n');
    fprintf('============================================================\n');
    fprintf('Primary Diagnosis: %s\n', BestMatch);
    fprintf('Confidence Metric (Error): %.2f%%\n', RankingTable.Average_Percent_Error(1));
    disp(RankingTable);
    fprintf('============================================================\n');
    fprintf('Runtime: %.2f seconds\n', runtime);

% % ==========================================================
% EXTRACT DATA
% % ==========================================================
% 
% missionHeaders = strtrim(string(Mission_Data_Matrix(1,:)));
% missionData    = cell2mat(Mission_Data_Matrix(2:end,:));
% 
% simHeaders = strtrim(string(Nominal_Sim_Matrix(1,:)));
% simData    = cell2mat(Nominal_Sim_Matrix(2:end,:));
% 
% timeIdxMission = findColumn(missionHeaders, ["eng_elaps_t","eng_elaps_t_0000"]);
% timeIdxSim     = findColumn(simHeaders, ["eng_elaps_t","eng_elaps_t_0000"]);
% 
% timeMission = missionData(:,timeIdxMission);
% timeSim     = simData(:,timeIdxSim);
% 
% %% Mission signals
% missionDepthIdx = findColumn(missionHeaders,"eng_depth");
% missionPitchIdx = findColumn(missionHeaders,"eng_pitchAng");
% missionRollIdx  = findColumn(missionHeaders,"eng_rollAng");
% missionHeadIdx  = findColumn(missionHeaders,"eng_head");
% missionVertIdx  = findColumn(missionHeaders,"vert_speed_gsm");
% 
% %% DT signals
% DTDepthIdx = findColumn(simHeaders,["DT_depth","eng_depth"]);
% DTPitchIdx = findColumn(simHeaders,["DT_pitchAng","eng_pitchAng"]);
% DTRollIdx  = findColumn(simHeaders,["DT_rollAng","eng_rollAng"]);
% DTHeadIdx  = findColumn(simHeaders,["DT_heading","eng_head"]);
% DTVertIdx  = findColumn(simHeaders,["DT_vertVel","vert_speed_gsm"]);
% 
% %% Extract vectors
% missionDepth = missionData(:,missionDepthIdx);
% missionPitch = missionData(:,missionPitchIdx);
% missionRoll  = missionData(:,missionRollIdx);
% missionHead  = missionData(:,missionHeadIdx);
% missionVert  = missionData(:,missionVertIdx);
% 
% DTDepth = simData(:,DTDepthIdx);
% DTPitch = simData(:,DTPitchIdx);
% DTRoll  = simData(:,DTRollIdx);
% DTHead  = simData(:,DTHeadIdx);
% DTVert  = simData(:,DTVertIdx);
% 
% %% Force column vectors
% timeMission = timeMission(:);
% timeSim     = timeSim(:);
% 
% missionDepth = missionDepth(:);
% missionPitch = missionPitch(:);
% missionRoll  = missionRoll(:);
% missionHead  = missionHead(:);
% missionVert  = missionVert(:);
% 
% DTDepth = DTDepth(:);
% DTPitch = DTPitch(:);
% DTRoll  = DTRoll(:);
% DTHead  = DTHead(:);
% DTVert  = DTVert(:);
% 
% %% Match lengths
% n = min([length(timeMission),length(missionDepth),length(missionPitch), ...
%          length(missionRoll),length(missionHead),length(missionVert)]);
% 
% m = min([length(timeSim),length(DTDepth),length(DTPitch), ...
%          length(DTRoll),length(DTHead),length(DTVert)]);
% 
% timeMission = timeMission(1:n);
% timeSim     = timeSim(1:m);
% 
% missionDepth = missionDepth(1:n);
% missionPitch = missionPitch(1:n);
% missionRoll  = missionRoll(1:n);
% missionHead  = missionHead(1:n);
% missionVert  = missionVert(1:n);
% 
% DTDepth = DTDepth(1:m);
% DTPitch = DTPitch(1:m);
% DTRoll  = DTRoll(1:m);
% DTHead  = DTHead(1:m);
% DTVert  = DTVert(1:m);
% 
% %% ==========================================================
% % DT ERROR ANALYSIS
% %% ==========================================================
% 
% vars = {missionDepth, missionPitch, missionRoll, missionHead, missionVert};
% dtvars = {DTDepth, DTPitch, DTRoll, DTHead, DTVert};
% 
% names = ["Depth","Pitch","Roll","Heading","Vertical Velocity"];
% units = ["cm","deg","deg","deg","cm/s"];
% 
% pct = [0.066,0.066,0.066,0.066,0.033];
% 
% nV = length(vars);
% 
% for i = 1:nV
% 
%     mvar = vars{i};
%     dvar = dtvars{i};
% 
%     valid = isfinite(mvar) & isfinite(dvar);
% 
%     mvar = mvar(valid);
%     dvar = dvar(valid);
% 
%     t = timeMission(valid);
% 
%     e_actual = abs(mvar - dvar);
% 
%     range_m = max(mvar) - min(mvar);
%     E_allow = range_m * pct(i);
% 
%     range_e = max(e_actual) - min(e_actual);
%     std_e   = std(e_actual);
% 
%     pass = (range_e < E_allow) && (std_e < E_allow);
% 
%     fprintf('\n%s RESULT:\n', names(i));
%     fprintf('Allowed Error: %.4f %s\n', E_allow, units(i));
%     fprintf('Error Range  : %.4f\n', range_e);
%     fprintf('Error STD    : %.4f\n', std_e);
%     fprintf('PASS = %d\n', pass);
% 
%     %% PLOT WITH ERROR BOUNDS
%     figure
%     hold on
% 
%     upper = mvar + E_allow;
%     lower = mvar - E_allow;
% 
%     plot(t, mvar, 'b', 'LineWidth',1.5)
%     plot(t, dvar, '--r', 'LineWidth',1.5)
%     plot(t, upper, ':k')
%     plot(t, lower, ':k')
% 
%     title(names(i) + " DT Validation")
%     xlabel("Time [s]")
%     ylabel(names(i) + " [" + units(i) + "]")
% 
%     legend("Mission","DT","+Error","-Error")
% 
%     grid on
% 
%     if names(i) == "Depth"
%         set(gca,'YDir','reverse')
%     end
% 
% end
% 
% %% ==========================================================
% % HELPER
% %% ==========================================================
% 
% function idx = findColumn(headers, names)
% 
% for i = 1:length(names)
%     idx = find(strcmpi(headers,names(i)),1);
%     if ~isempty(idx)
%         return;
%     end
% end
% 
% error("Missing column.");
% 
% end
