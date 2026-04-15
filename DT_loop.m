% function DT_wp = DT_loop(D_TGT, T_DIVE)
   
    % w = (2 * D_TGT * 100) / (T_DIVE * 60); % vertical component of velocity [cm/s] 
    % V = 25; % Average speed of the seaglider [cm/s] 
    % gamma = asin(w/V); % glide angle
    % u = w * (1/tan(gamma));  % horizontal component of velocity [cm/s] 
    % D_HORZ = D_TGT * (1/tan(gamma)); % horizontal distance form the beginning of the dive phase to when the SG reaches D_TGT. It is assumed constant 
 %% =======================
% DIVE WAYPOINT GENERATION ONLY
% =======================

clear; clc; close all;

%% -----------------------
% GIVEN PARAMETERS
% -----------------------

D_TGT = 100;       % target depth (meters)
V = 0.25;              % m/s  (instead of 25 cm/s)
T_DIVE = 200 * 60;     % seconds (if originally 200 min)
dt = 500;                % seconds

heading_deg = 30;  % random heading (just for testing)
psi = deg2rad(heading_deg);

%% -----------------------
% COMPUTE VELOCITY COMPONENTS
% -----------------------

% vertical velocity (downward)
w = (2 * D_TGT) / T_DIVE;  
% factor of 2 because depth reached at T_DIVE/2

% glide angle
gamma = asin(w / V);

% horizontal velocity magnitude
u = V * cos(gamma);

%% -----------------------
% BUILD VELOCITY VECTOR
% -----------------------

vx = u * cos(psi);   % North
vy = u * sin(psi);   % East
vz = w;              % Down

v = [vx, vy, vz];    % full 3D velocity vector

%% -----------------------
% TIME FOR DIVE ONLY
% -----------------------

t = 0:dt:T_DIVE/2;   % only until reaching D_TGT
N = length(t);

%% -----------------------
% GENERATE WAYPOINTS
% -----------------------

pos = zeros(N,3);    % store positions
pos(1,:) = [0,0,0];  % start at surface

for i = 2:N
    
    % move forward using velocity
    pos(i,:) = pos(i-1,:) + v * dt;
    
end

%% -----------------------
% CHECK FINAL DEPTH
% -----------------------

disp(['Final depth: ', num2str(pos(end,3)), ' m'])
disp(['Target depth: ', num2str(D_TGT), ' m'])

%% -----------------------
% 3D PLOT
% -----------------------

figure;

plot3(pos(:,1), pos(:,2), pos(:,3), 'b-o','LineWidth',1.5);
grid on;

xlabel('North (m)');
ylabel('East (m)');
zlabel('Depth (m)');

title('Dive Waypoints (Should Follow Glide Angle)');

set(gca, 'ZDir','reverse'); % depth increases downward
axis equal;


%% -----------------------
% MARK START AND END
% -----------------------

hold on;

plot3(pos(1,1), pos(1,2), pos(1,3), 'go','MarkerSize',10,'LineWidth',2);
plot3(pos(end,1), pos(end,2), pos(end,3), 'ro','MarkerSize',10,'LineWidth',2);

legend('Dive Path','Start','End');