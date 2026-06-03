function params = parametersV5()
% PARAMETERSV3 Creates parameter structure for dynamicsV3.m
%
% Usage:
%   params = parametersV3();
%
% These values are starter/default values. Replace TBD values with measured
% seaglider values when available.

% Target / mission parameters
params.heading_desired = 0;      % deg, desired heading for dive

% Aerodynamic / hydrodynamic geometry
params.S    = 0.486;              % m^2, wing/reference surface area [TBD]
params.cbar = 0.173;              % m, mean aerodynamic chord [TBD]
params.b    = 1.00;              % m, wing span [TBD]

% Battery properties
params.mbat  = 11.636;             % kg, battery pack mass [TBD]

% Whole glider properties
params.Vstatic = 0.0562;        % m^3, displaced volume without VBD [TBD]
params.Ms      = 58.852 - params.mbat;           % kg, stationary mass [TBD]

% Added mass matrix, kg
params.Mf = [3.310 0 0
            0 72.005 0
            0 0 72.005];          % estimated added mass

% Added mass inertia matrix, kg*m^2
params.Jf = [0 0 0
             0 10.062 0
             0 0 10.062];          % estimated added inertia

% Stationary mass inertia matrix, kg*m^2
params.Js = [0.1009 -0.0604 0.1086
            -0.0604 8.1926 -0.0015
             0.1086 -0.0015 8.1437]; % from trim sheet, may need CAD


%% Ocean / environmental properties
params.ambtemp  = 10;            % deg C, ambient surface temperature [TBD]
params.temp     = 10;            % deg C, local water temperature [TBD]
params.pressure = 0;             % pressure/depth input used in dynamicsV3 [TBD]
params.rho      = 1028.1;        % kg/m^3, seawater density [currently not used directly]
params.salt     = 35;            % PSU, salinity [currently not used directly]

end
