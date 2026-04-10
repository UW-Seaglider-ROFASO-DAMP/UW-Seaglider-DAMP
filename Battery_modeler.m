
% Author(s): Jordan Cummings, Henry Hong, Josh Rolfe
% Project: Seaglider Capstone 2026
% Purpose: Battery Modeler
% Inputs: Initial pitch/roll and desired pitch/roll.
% Output: Voltage (V), Current (I), and time (s). 
% Function: To estimate the amount of voltage, current, and time it takes for the
%           battery system to move from initial state to desired state. 
% Version: 1.01, updated on 04/10/2026

%% Find the change in battery position based on inital to desired pitch, roll, yaw



% Output
% need an if/then statement for intial dive usign more current and
% following manuevers using less current. 

pitch_i = %find ;

pitch_f = %find ;

roll_i = %find ; 

roll_f = %find ;

%% Begining the function to take in inputs.
function battery_energy = Battery_modeler(pitch_i, pitch_f,roll_i, roll_f)

%% Average Voltage 

% equation based of data analysis for average voltage used during battery
% movement

Voltage =  ;

%% Average Amperage 

% equation based on data analysis for average amperage used during battery
% movement

Amperage =  ;

%% Time

% equation for average time per movement (i.e. 1s = 3cm)

Total_movement_pitch = pitch_f - pitch_i;

Total_movement_roll = roll_f - roll_i; 

time_pitch = Total_movement_pitch * %find average movement

time_roll = Total_movement_roll * %find average


%% Matrix for Power Modeler

battery_energy = [Voltage, Amperage, time]; 

end