function Nom_Sim_Matrix = Digital_Twin(Mission_Data_Matrix, coefs, params)
% DIGITAL_TWIN
%
% Runs dynamicsV5 row-by-row using ode45.
% Designed to match the dynamics_test_whole_dive workflow, but with
% row-by-row controller updates from Mission_Data_Matrix.
%
% IMPORTANT CHANGE:
%   The ODE simulation now starts only after the glider is submerged past a
%   selected depth threshold. Rows before that point are copied from the
%   mission data unchanged. This avoids trying to simulate unstable surface
%   startup behavior with the submerged-glider dynamics model.
%
% Mission data units:
%   eng_depth       = cm
%   vert_speed_gsm  = cm/s
%   density         = kg/m^3
%   pressure        = pressure from NC file
%
% Internal dynamics units:
%   depth           = m, positive downward
%   vertical speed  = m/s, positive downward
%   density         = kg/m^3
%
% State vector:
%   X = [phidot; thetadot; headingdot; phi; theta; heading; u; v; w; x; y; z]
%
%   X(1:3) = Euler angle rates, deg/s
%   X(4)   = roll angle, deg
%   X(5)   = pitch angle, deg
%   X(6)   = heading angle, deg
%   X(7)   = forward/glide-path velocity, m/s
%   X(8)   = lateral/glide-path velocity, m/s
%   X(9)   = vertical velocity, m/s, positive downward
%   X(12)  = depth, m, positive downward

%% Separate headers and numeric data
headers = strtrim(string(Mission_Data_Matrix(1,:)));
data = cell2mat(Mission_Data_Matrix(2:end,:));

numRows = size(data,1);

%% Find required columns
timeIdx  = findColumn(headers, ["eng_elaps_t", "eng_elaps_t_0000"]);
depthIdx = findColumn(headers, ["eng_depth"]);
headIdx  = findColumn(headers, ["eng_head"]);

pitchIdx = findColumn(headers, ["eng_pitchAng", "pitchAng", "pitchANG"]);
rollIdx  = findColumn(headers, ["eng_rollAng", "rollAng", "rollANG"]);

pitchCtlIdx = findColumn(headers, ["eng_pitchCtl", "pitchCtl"]);
rollCtlIdx  = findColumn(headers, ["eng_rollCtl", "rollCtl"]);
vbdIdx       = findColumn(headers, ["eng_vbdCC", "vbdCC"]);

vertIdx     = findColumn(headers, ["vert_speed_gsm"]);
densityIdx  = findColumn(headers, ["density", "log_RHO", "$RHO"]);
pressureIdx = findColumn(headers, ["pressure"]);
tempIdx     = findColumn(headers, ["temperature", "temp"]);

headingDesiredIdx = findColumn(headers, ...
    ["$MHEAD_RNG_PITCHd_Wd", "MHEAD_RNG_PITCHd_Wd", "log_MHEAD_RNG_PITCHd_Wd"]);

%% Error checks
if isempty(timeIdx), error("Missing time column."); end
if isempty(depthIdx), error("Missing depth column."); end
if isempty(headIdx), error("Missing heading column."); end
if isempty(pitchIdx), error("Missing pitch angle column."); end
if isempty(rollIdx), error("Missing roll angle column."); end
if isempty(pitchCtlIdx), error("Missing pitch control column."); end
if isempty(rollCtlIdx), error("Missing roll control column."); end
if isempty(vbdIdx), error("Missing VBD column."); end
if isempty(vertIdx), error("Missing vertical speed column."); end
if isempty(densityIdx), error("Missing density column."); end
if isempty(pressureIdx), error("Missing pressure column."); end
if isempty(headingDesiredIdx), error("Missing desired heading column."); end

%% Create output matrix
% Start by copying the mission matrix. Rows before the simulation start row
% intentionally remain unchanged.
Nom_Sim_Matrix = Mission_Data_Matrix;

%% Choose simulation start row
% The dynamics model behaves best once the glider is submerged and actually
% gliding. Starting at the surface caused the previous blow-up.
%
% Default: start once mission depth reaches 1000 cm = 10 m.
% To change this without editing code, add this field in parametersV5.m:
%   params.simStartDepth_cm = 500;  % example: start after 5 m
if isfield(params, 'simStartDepth_cm')
    simStartDepth_cm = params.simStartDepth_cm;
else
    simStartDepth_cm = 1000;
end

startRow = find(data(:,depthIdx) >= simStartDepth_cm, 1, 'first');

if isempty(startRow)
    warning('No depth greater than %.1f cm found. Starting simulation at row 1.', simStartDepth_cm);
    startRow = 1;
end

fprintf('\n====================================\n');
fprintf('Digital Twin Simulation Start\n');
fprintf('====================================\n');
fprintf('Simulation starts at numeric data row: %d of %d\n', startRow, numRows);
fprintf('Mission_Data_Matrix row: %d\n', startRow + 1);
fprintf('Start depth: %.2f cm\n', data(startRow, depthIdx));
fprintf('Rows before this are copied unchanged from mission data.\n');
fprintf('====================================\n\n');

%% Initial state vector at selected start row
X = zeros(12,1);

% Estimate initial angular rates near startRow.
% Prefer backward difference so the start state is based on the selected row.
if startRow > 1
    ratePrevRow = startRow - 1;
    rateNextRow = startRow;
elseif numRows >= 2
    ratePrevRow = startRow;
    rateNextRow = startRow + 1;
else
    ratePrevRow = startRow;
    rateNextRow = startRow;
end

dtRate = data(rateNextRow,timeIdx) - data(ratePrevRow,timeIdx);

if isfinite(dtRate) && dtRate > 0
    X(1) = (data(rateNextRow,rollIdx)  - data(ratePrevRow,rollIdx))  / dtRate; % phidot, deg/s
    X(2) = (data(rateNextRow,pitchIdx) - data(ratePrevRow,pitchIdx)) / dtRate; % thetadot, deg/s

    head1 = data(ratePrevRow,headIdx);
    head2 = data(rateNextRow,headIdx);
    dHead = wrapTo180_custom(head2 - head1);
    X(3) = dHead / dtRate; % headingdot, deg/s
end

% Initial angles from selected mission row.
X(4) = data(startRow,rollIdx);
X(5) = data(startRow,pitchIdx);
X(6) = mod(data(startRow,headIdx),360);

% Initial translational velocities.
X(7) = 0.1;                       % assumed forward velocity, m/s
X(8) = 0;                         % assumed lateral velocity, m/s
X(9) = data(startRow,vertIdx)/100; % cm/s to m/s, positive downward convention

% Initial positions.
X(10) = 0;
X(11) = 0;
X(12) = data(startRow,depthIdx)/100; % cm to m

% Apply initial safety cleanup.
X = cleanState(X);

%% Prebuild coefficient interpolants for faster dynamics calls
coefs = buildCoefficientInterpolants(coefs);

%% ode45 options
% Looser tolerances are intentional here. The digital twin is being driven
% by 5-6 s mission samples, so very tight tolerances slow the solver without
% improving the row-level result much.
odeOpts = odeset( ...
    'RelTol', 1e-3, ...
    'AbsTol', 1e-5, ...
    'MaxStep', 0.5);

%% Loop through mission rows
% k is the current numeric mission row. The ODE integrates row k -> k+1.
for k = startRow:numRows-1

    dt = data(k+1,timeIdx) - data(k,timeIdx);

    fprintf('Running simulation row %d of %d\n', k, numRows-1);
    fprintf('Row %d/%d | dt = %.3f | roll = %.2f | pitch = %.2f | head = %.2f | w = %.4f | depth = %.2f\n', ...
        k, numRows-1, dt, X(4), X(5), X(6), X(9), X(12));

    if ~isfinite(dt) || dt <= 0
        warning("Bad dt at mission row %d. Holding previous state.", k);
        Xnew = X;
        Nom_Sim_Matrix = saveStateToMatrix(Nom_Sim_Matrix, k, Xnew, ...
            pitchIdx, rollIdx, headIdx, vertIdx, depthIdx);
        continue;
    end

    %% Update parameters from mission data
    params_k = params;

    params_k.heading_desired = data(k, headingDesiredIdx);
    params_k.pressure = data(k, pressureIdx);
    params_k.rho = data(k, densityIdx);

    if ~isempty(tempIdx)
        params_k.temp = data(k, tempIdx);
    end

    %% Build control vector from row k
    % Pitch control appears to be cm; convert to m.
    x_bat = data(k, pitchCtlIdx) / 100;

    % Roll control is used directly as battery roll angle.
    phi_bat = data(k, rollCtlIdx);

    % VBD already in cc.
    vbdCC = data(k, vbdIdx);

    U = [x_bat; phi_bat; vbdCC];

    %% Run dynamics for this row-to-row time step
    X0 = X;

    try
        [~, Xout] = ode45(@(t,Xode) dynamicsV6_06(Xode, U, coefs, params_k), ...
                          [0 dt], X0, odeOpts);

        Xnew = Xout(end,:).';

        if any(~isfinite(Xnew))
            warning("Bad dynamics output at mission row %d. Holding previous state.", k);
            Xnew = X;
        end

        fprintf('   ode45 steps used: %d\n', size(Xout,1));

    catch ME
        warning("dynamicsV5 failed at mission row %d. Holding previous state. Message: %s", ...
                k, ME.message);
        Xnew = X;
    end

    %% Clean state values to prevent solver blow-up
    Xnew = cleanState(Xnew);

    %% Save simulated values to next mission row
    Nom_Sim_Matrix = saveStateToMatrix(Nom_Sim_Matrix, k, Xnew, ...
        pitchIdx, rollIdx, headIdx, vertIdx, depthIdx);

    %% Carry final ODE state into next step
    X = Xnew;

end

fprintf('\n');

%% Display summary
fprintf('\n====================================\n');
fprintf('Nominal Simulation Matrix Created\n');
fprintf('====================================\n');
fprintf('Rows in matrix: %d\n', numRows);
fprintf('Rows simulated with ODE: %d\n', max(numRows - startRow, 0));
fprintf('Rows copied before simulation: %d\n', startRow - 1);
fprintf('Updated columns after start row:\n');
fprintf(' - %s\n', headers(pitchIdx));
fprintf(' - %s\n', headers(rollIdx));
fprintf(' - %s\n', headers(headIdx));
fprintf(' - %s in cm/s\n', headers(vertIdx));
fprintf(' - %s in cm\n', headers(depthIdx));
fprintf('====================================\n');

end

%% =========================================================
% Helper: save state to output matrix
%% =========================================================
function Nom_Sim_Matrix = saveStateToMatrix(Nom_Sim_Matrix, k, Xnew, ...
    pitchIdx, rollIdx, headIdx, vertIdx, depthIdx)

% k is the current numeric data row.
% Mission_Data_Matrix has one header row, so numeric row k+1 in the data
% is matrix row k+2.
rowOut = k + 2;

Nom_Sim_Matrix{rowOut, pitchIdx} = Xnew(5);
Nom_Sim_Matrix{rowOut, rollIdx}  = Xnew(4);
Nom_Sim_Matrix{rowOut, headIdx}  = Xnew(6);

% Save internal w from m/s back to cm/s.
Nom_Sim_Matrix{rowOut, vertIdx} = Xnew(9) * 100;

% Convert depth from m to cm.
Nom_Sim_Matrix{rowOut, depthIdx} = Xnew(12) * 100;

end

%% =========================================================
% Helper: clean/saturate state vector
%% =========================================================
function X = cleanState(X)

% Remove non-finite values early.
if any(~isfinite(X))
    X(~isfinite(X)) = 0;
end

% Heading wrap.
X(6) = mod(X(6),360);

% Clamp angular rates, deg/s.
X(1:3) = max(min(X(1:3), 20), -20);

% Clamp Euler angles.
% Roll is limited to +/-90 for numerical stability.
% Pitch is limited to +/-85 to avoid tan(theta)/sec(theta) singularity
% inside the dynamics equations.
X(4) = max(min(X(4), 90), -90);
X(5) = max(min(X(5), 85), -85);

% Clamp velocities, m/s.
% Seaglider vertical speeds should be far below 1 m/s, but 1 m/s gives
% enough room during debugging while preventing runaway states.
X(7:9) = max(min(X(7:9), 1.0), -1.0);

% Prevent negative depth.
if X(12) <= 0
    X(12) = 0;

    % If at surface and moving upward, stop upward motion.
    if X(9) < 0
        X(9) = 0;
    end
end

% Hard upper bound on simulated depth for numerical protection.
% Increase this if your mission is deeper than 1000 m.
X(12) = min(X(12), 1000);

end

%% =========================================================
% Helper: build coefficient interpolants
%% =========================================================
function coefs = buildCoefficientInterpolants(coefs)

% Sort grids defensively.
[alphasSorted, ia] = sort(coefs.alphas(:).');
[betasSorted, ib]  = sort(coefs.betas(:).');

coefs.alphas = alphasSorted;
coefs.betas  = betasSorted;

coefNames = ["CLs", "CDs", "CYs", "Croll", "Cpitch", "Cyaw"];

for i = 1:numel(coefNames)
    name = coefNames(i);
    table = coefs.(name);
    table = table(ia, ib);
    coefs.(name) = table;
end

% Rows correspond to alpha; columns correspond to beta.
coefs.F_CL     = griddedInterpolant({coefs.alphas, coefs.betas}, coefs.CLs,     'linear', 'nearest');
coefs.F_CD     = griddedInterpolant({coefs.alphas, coefs.betas}, coefs.CDs,     'linear', 'nearest');
coefs.F_CY     = griddedInterpolant({coefs.alphas, coefs.betas}, coefs.CYs,     'linear', 'nearest');
coefs.F_Croll  = griddedInterpolant({coefs.alphas, coefs.betas}, coefs.Croll,  'linear', 'nearest');
coefs.F_Cpitch = griddedInterpolant({coefs.alphas, coefs.betas}, coefs.Cpitch, 'linear', 'nearest');
coefs.F_Cyaw   = griddedInterpolant({coefs.alphas, coefs.betas}, coefs.Cyaw,   'linear', 'nearest');

end

%% =========================================================
% Helper: find column
%% =========================================================
function idx = findColumn(headers, possibleNames)

idx = [];

for i = 1:length(possibleNames)
    idx = find(strcmpi(headers, possibleNames(i)), 1);

    if ~isempty(idx)
        return;
    end
end

end

%% =========================================================
% Helper: wrap angle to [-180, 180]
%% =========================================================
function angleOut = wrapTo180_custom(angleIn)

angleOut = mod(angleIn + 180, 360) - 180;

end
