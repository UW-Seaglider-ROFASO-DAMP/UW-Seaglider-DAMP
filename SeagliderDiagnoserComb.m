function Results = SeagliderDiagnoserComb(DT_Output, MI_Output)
% This function compares actual flight data (MI) against nominal models (DT)

%% 1. PRE-PROCESSING
varNames = string(DT_Output(1, :));         % Convert header row to string array
DT_data = cell2mat(DT_Output(2:end, :));    % Convert Digital Twin cells to numbers
MI_data = cell2mat(MI_Output(2:end, :));    % Convert Mission data cells to numbers

time = DT_data(:, 1);                       % Extract the time column
T = time(end) - time(1);                    % Calculate total flight duration
t0 = time(1);                               % Record the start time
% Create a mask to ignore the first 20% and last 20% of the flight 
trimMask = (time > t0 + 0.2*T) & (time < t0 + 0.8*T);

% Keep only the numeric data for the "steady state" part of the flight
DT_trim = DT_data(trimMask, 2:end);         % Trimmed nominal data (no time col)
MI_trim = MI_data(trimMask, 2:end);         % Trimmed actual mission data (no time col)

%% 2. FAULT DETECTION (The "Switch")
% Calculate percentage error for every single data point
errorNominal = abs((MI_trim - DT_trim) ./ DT_trim) * 100;
% If any data was 0, division results in NaN (Not a Number); change those to 0
errorNominal(isnan(errorNominal) | isinf(errorNominal)) = 0;

% Identify any points where error is greater than 2%
excursionMask = errorNominal > 2;
% Sum up all the errors that were outside the 2% "safe zone"
totalNominalDiff = sum(errorNominal(excursionMask), 'all');

% Decide if a fault exists: if the sum of errors is > 40, flip the switch ON
Results.IsDetected = totalNominalDiff > 40;
Results.NominalDiff = totalNominalDiff; % Save this for the report

if ~Results.IsDetected
    % If error is low, exit early and tell the user everything is fine
    fprintf('✓ System Nominal: Total Error (%.2f) below threshold.\n', totalNominalDiff);
    Results.Status = "Nominal";
    return;
end

% If we reach here, a fault was detected
fprintf('⚠ Fault Detected (Error: %.2f). Running Isolator...\n', totalNominalDiff);

%% 3. FAULT ISOLATION (The Search Logic)
bestError = inf; % Start with an "infinitely large" error
bestCase = struct('Type', "Unknown", 'Severity', 0); % Placeholder for best guess

% We will test every damage level from 0% to 100% in 1% steps
damageSteps = 0:1:100; 

% --- WING FAULT SEARCH ---
for wing = damageSteps
    % Recreate the math used to generate wing damage
    targetPct = 1 + (100 - 1) * (wing / 100);
    scale = 1 / (1 + targetPct/100);
    
    % Simulate what a Glider would look like with THIS specific wing damage
    % We apply the scale to the "Healthy" Digital Twin data
    FS_trim = DT_trim * scale; 
    
    % Compare our "Fake Fault Simulation" to the "Actual Mission" data
    currentErr = calculateDiff(MI_trim, FS_trim);
    
    % If this is the closest match we've found so far, save it
    if currentErr < bestError
        bestError = currentErr; % Update the lowest error
        bestCase.Type = "Wing"; % Record that it looks like a Wing fault
        bestCase.Severity = wing; % Record the damage percentage
    end
end

% --- RUDDER FAULT SEARCH ---
for rudder = damageSteps
    % Recreate the math used to generate rudder damage
    wingError = 1 + (100 - 1) * (rudder / 100);
    targetPct = 2 * wingError;
    scale = 1 / (1 + targetPct/100);
    
    % Simulate what a Glider would look like with THIS specific rudder damage
    FS_trim = DT_trim * scale;
    
    % Compare this simulation to the Actual Mission data
    currentErr = calculateDiff(MI_trim, FS_trim);
    
    % If this rudder fault is a better match than anything seen before...
    if currentErr < bestError
        bestError = currentErr;   % Update the lowest error
        bestCase.Type = "Rudder"; % Record that it looks like a Rudder fault
        bestCase.Severity = rudder;
    end
end

%% 4. FINAL REPORTING
Results.Status = "Faulted";       % Mark status as broken
Results.BestMatch = bestCase;     % Store the best Type and Severity found
Results.MatchError = bestError;   % Store how "tight" the fit was (closer to 0 is better)

% Print the final findings to the command window
fprintf('=== ISOLATION COMPLETE ===\n');
fprintf('Most Likely Fault: %s Damage\n', bestCase.Type);
fprintf('Estimated Severity: %d%%\n', bestCase.Severity);
fprintf('Residual Match Error: %.4f\n', bestError);

end

% HELPER FUNCTION: Mathematical Difference
function totalDiff = calculateDiff(Actual, Simulated)
    % This uses RMSE (Root Mean Square Error)
    % 1. Subtract datasets, 2. Square the results (makes all errors positive)
    diffs = (Actual - Simulated).^2;
    % 3. Find the average of those squares, 4. Take the square root
    % This gives us a single number representing how "far apart" two datasets are
    totalDiff = sqrt(mean(diffs, 'all')); 
end