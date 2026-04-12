%% 1. SETTINGS: Choose what fault you want to simulate
clear;clc;   %clearing board for use

%Option A: Plug in desired damage state (commented out until necessary)

SimulateWingDamage   = .1;  % Input: Percent of wing damage to fake (0-100)
SimulateRudderDamage = 0; % Input: Percent of rudder damage to fake (0-100)

%Option B: Random damage state (commented out until necessary)

%if rand() > 0.5
     %SimulateWingDamage = randi([5, 95]);   % Random wing damage between 5% and 95%
    % SimulateRudderDamage = 0;
 %else
    % SimulateRudderDamage = randi([5, 95]); % Random rudder damage between 5% and 95%
    % SimulateWingDamage = 0;
%end
%% 2. LOAD NOMINAL DATA (The Digital Twin)
load('MI_reference.mat');  % Open the file containing known-good flight data
DT_Output = MI_Output;     % Store the good data as the "Digital Twin" 

%% 3. GENERATE FAKE MISSION DATA (The "Broken" Seaglider)
% We convert the cell-array (text + numbers) into a pure matrix of numbers
MI_numeric = cell2mat(DT_Output(2:end, :)); % Skip the first row (headers), grab numbers
varNames   = DT_Output(1, :);               % Save the top row (headers) for later

if SimulateWingDamage > 0 % If user wants to test a wing fault...
    % Formula to map 0-100% damage to a specific error percentage
    targetPct = 1 + (100 - 1) * (SimulateWingDamage / 100);
    % Calculate the multiplier to shrink the data (simulates loss of lift/speed)
    scale = 1 / (1 + targetPct/100);
    fprintf('Simulating Wing Fault: %d%%\n', SimulateWingDamage); % Status message
elseif SimulateRudderDamage > 0 % If user wants to test a rudder fault...
    % Rudder damage uses a different multiplier formula (2x the impact)
    wingError = 1 + (100 - 1) * (SimulateRudderDamage / 100);
    targetPct = 2 * wingError;
    scale = 1 / (1 + targetPct/100);
    fprintf('Simulating Rudder Fault: %d%%\n', SimulateRudderDamage); % Status message
else
    scale = 1.0; % No damage selected, scale is 100% (unchanged)
    fprintf('Simulating Nominal (No Fault) condition.\n');
end

% Create a copy of the numbers to modify
MI_numeric_bad = MI_numeric;
% Apply the damage scale to all columns EXCEPT time (column 1)
MI_numeric_bad(:, 2:end) = MI_numeric(:, 2:end) * scale; 

% Re-combine the headers and the newly "damaged" numbers into one cell array
MI_Output_FieldData = [varNames; num2cell(MI_numeric_bad)];

%% 4. RUN THE COMBINED DIAGNOSER
% Call the main function: Compare "Good" (DT) vs "Faked Bad" (MI_FieldData)
Results = SeagliderDiagnoserComb(DT_Output, MI_Output_FieldData);

%% 5. VERIFY RESULTS
if Results.IsDetected % If the diagnoser found a fault...
    fprintf('\n--- TEST RESULT ---\n');
    
    % Check what we actually simulated so we can print it
    if SimulateWingDamage > 0
        actualType = "Wing";   % We faked a wing fault
        actualSev  = SimulateWingDamage;
    elseif SimulateRudderDamage > 0
        actualType = "Rudder"; % We faked a rudder fault
        actualSev  = SimulateRudderDamage;
    else
        actualType = "None";   % No fault faked
        actualSev  = 0;
    end
    
    % Print what we did vs what the computer guessed
    fprintf('Actual Damage Applied: %s %d%%\n', actualType, actualSev);
    fprintf('Diagnoser identified:  %s %d%%\n', Results.BestMatch.Type, Results.BestMatch.Severity);
    
    % Check if the guess matches the reality
    if actualType == Results.BestMatch.Type && actualSev == Results.BestMatch.Severity
        fprintf('SUCCESS: Diagnoser matched the simulated fault exactly.\n');
    else
        fprintf('NOTE: Diagnoser found a close match, but not exact.\n');
    end
else
    % This runs if the diagnoser thought the faked data was "good enough"
    disp('Diagnoser reported: System is Nominal');
end