function BestMatch = FaultIsolator(DT_Output,MI_Output, Historical_Data, Field_Data)
%% Section 1: Extracting DT and MI and Trimming

%Extract raw data from DT/MI
DT_raw = cell2mat(DT_Output(2:end, 2:end));
MI_raw = cell2mat(MI_Output(2:end, 2:end));

%Extracting Time (copied from mission compare. might need to be adjusted)
time_DT = cell2mat(DT_Output(2:end, 1));
time_MI = cell2mat(MI_Output(2:end, 1));

%creating 60% trim 
trim = (t > 0.2*(t(end)-t(1))) & (t < 0.8*(t(end)-t(1)));

%applying 60% trim to DT/MI
DT_trim = DT_time(trim, :); 
MI_trim = MI_time(trim, :);
%% Blue (Digital Twin Vs Mission Data)

% since we're trying to find minimum error need to use 'inf' error?
bestSimErr = inf;

% span from 0% to 100% of surfaces remaining
span_range = 0:1:100
%% Yellow (Historical Vs Field)
