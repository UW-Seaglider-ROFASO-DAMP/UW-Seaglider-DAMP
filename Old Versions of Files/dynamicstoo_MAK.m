% Seaglider dynamics function for digital twin via momentum
% To be put through your favorite ODE solver
% Author: Kyle Wittcoff
% Date: April 2026
% Send all complaints to: president@whitehouse.gov
% May Poseidon help us...

function X_dyn = dynamicstoo_MAK(X,U,coefs,params)
%% Unpacking Inputs

% Unpacking state vector X
phidot = X(1) * pi/180; % roll rate (deg/s to rad/s)
thetadot = X(2) * pi/180; % pitch rate (deg/s to rad/s)
headingdot = X(3) * pi/180; % yaw rate (deg/s to rad/s)

phi = X(4) * pi/180; % roll angle (deg to rad)
theta = X(5) * pi/180; % pitch angle (deg to rad)
heading = X(6) * pi/180; % heading angle (deg to rad)

u = X(7); % horizontal velocity
v = X(8); % lateral velocity
w = X(9); % vertical velocity

x = X(10); % x distance in Glide Path frame
y = X(11); % y distance in Glide Path frame
z = X(12); % z distance (depth) in Glide Path frame

% Unpacking control input
x_bat = U(1); % battery pos down x axis (in body frame)
phi_bat = U(2); % battery roll angle
% VBD_pos = U(3); % VBD position to be used later 



% Unpacking parameters
J0 = params.J0; % body moment of inerta around neutral battery cg
S = params.S; % wing surface area
cbar = params.cbar; % wing MAC
heading_desired = deg2rad(params.heading_desired); % desired heading for dive
rpbat = params.rpbat; % distance from x axis to cg of battery (in YZ plane)
mbat = params.mbat; % battery pack mass
Vol_static = params.Vstatic; % displaced volume w/o VBD
Vol_VBD = params.VolVBD; % placeholder VBD volume

% Kappa = [number] % compressibility factor -- MAK
% Tau = [number]  % Volumetric expansion, im not sure where to get this and if it constant -- MAK
% Vol_hull = Vol_static + Vol_VBD  % Volume of the hull -- MAK
% Volume = Vol_hull * exp(-(Kappa * P - tau * T)) -- MAK

Mf = params.Mf; % 3x3 added mass matrix, should be changed later?
Jf = params.Jf; % 3x3 added mass inertia matrix, should be changed later?
Js = params.Js; % 3x3 stationary mass inertia matrix
ms = params.Ms; % stationary mass

% % To (possibly) be added later:
% water_v = params.water_v; % water current velocity from model/forecast?
% water_h = params.water_heading; % water current direction model/forecast?
% salt = params.salt; % salinity contesnt of water from sensor?
% T = params.temp; % temperature
% P = params.pressure; % pressure from ct sail?
% m_total = ms + mbat  -- MAK

% Unpacking coefficients
alphas = coefs.alphas; % 1 x M vector of alphas used in wind tunnel test
betas = coefs.betas; % 1 x N vector of betas used in wind tunnel test
CLs = coefs.CLs; % M x N array of lift coef values from wind tunnel test
Cds = coefs.CDs; % M x N array of drag coef values from wind tunnel test
Cys = coefs.CYs; % M x N array of side coef values from wind tunnel test
Crolls= coefs.Croll;% M x N array of roll coef values from wind tunnel test
Cpitchs=coefs.Cpitch;%MxN array of pitch coef values from wind tunnel test
Cyaws= coefs.Cyaw;% M x N array of yaw coef values from wind tunnel test

% Yaw angle
psi = heading - heading_desired;

% Battery position rp in body frame
rp = [x_bat; rpbat * sind(phi_bat) ; rpbat * cosd(phi_bat) ];
rpx = skew(rp); % 3x3 cross product matrix for rp

% Seawater density linear approximation from: 
% https://mason.gmu.edu/~bklinger/seawater.pdf
rho = z * (1032.8 - 1028.1) + 1028.1;
% Note: we CAN change this to include full rho=f(P,T,S) formula
%       using sensor data.

% Total displaced volume & mass
Vol_disp = Vol_VBD + Vol_static;
m_disp = Vol_disp * rho;

% Gravity
g = 9.81;

%% Translational velocity, alpha, beta

% DCM for body frame to glide path frame
DCM_bg = angle2dcm(psi, theta, phi); % DCM body to glide
DCM_gb = DCM_bg.'; % DCM glide to body (it's just an inverse :) )

% Speed
V = sqrt(u^2+v^2+w^2); % norm function is for cowards

% Glide path and body frame velocity  
V_g = [u;v;w]; % glide path frame velocity
V_b = DCM_gb * V_g; % body frame velocity
ub = V_b(1);
vb = V_b(2);
wb = V_b(3);
% Note: We CAN implement the water currents in this part. Some frame 
%       silliness may be required.

% Flight path angles
alpha = atan2(wb,ub) * 180/pi; % angle of attack in degrees
beta = asin(vb / V) * 180/pi; % sideslip angle in degrees

%% Mass and Inertia

% Effective translational and rotational inertia matrices
M = ms*eye(3) + Mf; % masses
J = Js + Jf; % inertias

% Bouyancy approximation from masses
m0 = ms + mbat - m_disp;

%% Forces & Coefs

% Dynamic Pressure
Q = 0.5 * rho * V^2;

% Coefficients
CL = interp2(betas,alphas,CLs,beta,alpha); %interpolating Cl array
CD = interp2(betas,alphas,Cds,beta,alpha); %interpolating Cd array
CY = interp2(betas,alphas,Cys,beta,alpha); %interpolating Cy array
Croll = interp2(betas,alphas,Crolls,beta,alpha); % I think you get it by now
Cpitch = interp2(betas,alphas,Cpitchs,beta,alpha);
Cyaw = interp2(betas,alphas,Cyaws,beta,alpha);
% Note: Force coefs are probably in the wind frame, converting that next

% Hydrodynamic forces in wind frame
L = Q * CL * S; % lift
D = Q * CD * S; % drag
Y = Q * CY * S; % side
F_w = [-D; Y; -L];

% Buoyancy Force -- MAK
% B = g * (rho * Volume - m_total) 


% Wind -> body -> glide frames
DCM_bw = dcmbody2wind(alpha*pi/180,beta*pi/180); % <3 aerospace toolbox
DCM_wb = DCM_bw.'; % wind to body from inverse
Fb_aero = DCM_wb * F_w; % aero force wind to body

% Gravity in body frame
g_b = DCM_gb *[0;0;g];

%% Moments (torques)

% Hydrodynamic moments (torques)
Mp = Q * Croll * cbar * S; % roll moment
Mq = Q * Cpitch * cbar * S; % pitch moment
Mr = Q * Cyaw * cbar * S; % yaw moment
Torques = [Mp; Mq; Mr]; 

% Torques from Battery and VBD movement -- MAK
%
% pos_batt = battery position relative to the centriod   
% CG_batt =
% rho_oil = [value] % density of the oil in the bladder this is a constant
% F_batt = force battery exerts
% CG_VBD = (-1.69)
% F_VBD = g * Vol_VBD * (rho - rho_oil)
%
% 
%
% r_batt = radius of glider
% ang_acc = angular acceleration -- can be calculated with domega/dtime
% will add atsp 
%
% T_VBD = F_VBD * CG_VBD
% T_pitch = F_batt * ( pos_batt - CG_batt)
% T_roll = 0.5 * m_total * (r_batt)^2 * ang_acc
%
%

% Euler-rate vector in rad/s
eta_dot = [phidot; thetadot; headingdot]; %sensors give us euler angles
                                          %we'll convert for RB dynamics

% Kinematics matrix: eta_dot = H * omega, where omega = [p;q;r]
H = [1  sin(phi)*tan(theta)   cos(phi)*tan(theta);
     0  cos(phi)             -sin(phi);
     0  sin(phi)/cos(theta)   cos(phi)/cos(theta)];

% Time derivative of H
Hdot = [0,  cos(phi)*phidot*tan(theta) + sin(phi)*sec(theta)^2*thetadot, ...
            -sin(phi)*phidot*tan(theta) + cos(phi)*sec(theta)^2*thetadot;
        0, -sin(phi)*phidot, ...
            -cos(phi)*phidot;
        0,  cos(phi)*phidot*sec(theta) + sin(phi)*sec(theta)*tan(theta)*thetadot, ...
            -sin(phi)*phidot*sec(theta) + cos(phi)*sec(theta)*tan(theta)*thetadot];

% Body angular velocity
Omega = H \ eta_dot;   

%% Momentum (finally)

% Battery translational momentum
Pp = mbat * (V_b + cross(Omega, rp));

% Starting with vdot = M^-1 Fbar, see algebra in notes
Feq = cross(M*V_b + Pp, Omega) + m0*g_b + Fb_aero;
veq = [M + mbat*eye(3), -mbat*rpx];

% Starting with Omegadot = J^-1 Tbar, see algebra in notes
Teq = cross(J*Omega + cross(rp, Pp), Omega) ...
      + cross(M*V_b, V_b) ...
      + mbat * cross(rp, g_b) ...
      + Torques;
Omegaeq = [mbat*rpx, J - mbat*rpx*rpx];

% Accelerations from equation in notes
accels = [veq ; Omegaeq] \ [Feq ; Teq];
Vdot_b = accels(1:3); % translational
Omegadot = accels(4:6); % angular

%% Dx output vector

% Body angular accel to Euler angular accel
eta_Ddot = Hdot * Omega + H * Omegadot ; % chain rule :)

% Body accel to glide accel
Vdot_g = DCM_bg * ( Vdot_b + cross(Omega,V_b) );
udot = Vdot_g(1);
vdot = Vdot_g(2);
wdot = Vdot_g(3);

% Converting rad/s back to deg/s
phiDdot = eta_Ddot(1) * 180/pi;
thetaDdot = eta_Ddot(2) * 180/pi;
headingDdot = eta_Ddot(3) * 180/pi;
phidot = phidot * 180/pi;
thetadot = thetadot * 180/pi;
headingdot = headingdot * 180/pi;

% Output
X_dyn = [phiDdot; thetaDdot; headingDdot; phidot; thetadot; headingdot; ...
         udot; vdot; wdot; u; v; w];

end


