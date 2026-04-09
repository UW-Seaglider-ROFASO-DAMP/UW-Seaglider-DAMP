% Author: Jordan Cummings with help from ChatGPT
% Project: Seaglider Capstone 2026
% Purpose: Power Modeler
% Inputs: Starting battery amount (joules), Voltage (V), Current (I), and time
%         (sec) from Battery modeler and VBD modeler.
%         Inputs from battery and VBD should be in a 3x1 matrix formatted
%         [V; I; t].
% Output: Battery/energy remaining in joules
% Function: To estimate power useage during acutuator movements for battery
%           postion and VBD volume and subtract that from current power. 
% Version: 1.01, updated on 4/8/26 by JC


function Power_modeler = Power_modeler(battery_energy, battery_modeler, vbd_modeler)

if numel(batt_vec) ~= 3 || numel(vbd_vec) ~= 3
    error('Input vectors must be 3x1 format: [V; I; t]');
end

%% -------- EXTRACT VALUES --------
V_battery = battery_modeler(1);
I_battery = battery_modeler(2);
t_battery = battery_modeler(3);

V_vbd = vbd_modeler(1);
I_vbd = vbd_modeler(2);
t_vbd = vbd_modeler(3);

%% -------- CALCULATIONS --------
E_batt = V_battery * I_battery * t_battery;
E_vbd  = V_vbd  * I_vbd  * t_vbd;

E_total = E_batt + E_vbd;

Power_modeler = battery_energy - E_total;

%% -------- OPTIONAL WARNING --------
if Power_modeler < 0
    warning('Battery depleted! Remaining energy is negative.');
end

end