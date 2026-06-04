function X_dyn = dynamicsV7(X,U,coefs,params)
%% Unpacking Inputs

% Avoiding divide by zero errors

% Unpacking state vector X
phidot = X(1) * pi/180; % roll rate (deg/s to rad/s)
thetadot = X(2) * pi/180; % pitch rate (deg/s to rad/s)
headingdot = X(3) * pi/180; % yaw rate (deg/s to rad/s)

phi = X(4) * pi/180; % roll angle (deg to rad)
theta = X(5) * pi/180; % pitch angle (deg to rad)
heading = X(6) * pi/180; % heading angle (deg to rad)

u_i = X(7); % horizontal velocity wrt current heading
v_i = X(8); % lateral velocity wrt current heading
w = X(9); % vertical velocity in glide path frame

x = X(10); % x distance in Glide Path frame
y = X(11); % y distance in Glide Path frame
z = X(12); % z distance (depth) in Glide Path frame

% Unpacking control input
pitch_ctl = U(1); % battery pos down x axis (in body frame, aft positive)
phi_bat = U(2); % battery roll angle
VBD_cc = U(3); % oil count in the VBD pump in AD

% Battery CG longitudinal location, measured from the CB (body +x = nose)
X_bat0 = 0.0509; % neutral position, from 194 trim sheet
x_bat = X_bat0 - pitch_ctl ; % battery location relative to CB, m

% Unpacking parameters

% Target information
heading_desired = deg2rad(params.heading_desired); % desired heading for dive

% Aerodynamic surfaces properties
S = params.S; % wing surface area
cbar = params.cbar; % wing MAC
b = params.b; % wing span

% Glider properties

% Whole glider
Vol_static = params.Vstatic; % displaced volume w/o VBD
Mf = params.Mf; % 3x3 added mass matrix, should be changed later?
Jf = params.Jf; % 3x3 added mass inertia matrix, should be changed later?
ms = params.Ms; % stationary mass
Js = params.Js; % 3x3 stationary mass inertia matrix (referenced to the CB)
VCB = 0.00362; % Vertical CB location relative to neutral axis, in m for SG 194


% Stationary-mass CG offset from the CB, body frame, from 194 trim sheet
r_s = [-0.0125; 0; 0.0041];

% Battery
VCG_bat = 0.01082; % Battery vertical cg location relative to neutral axis, in m for SG 194
mbat = params.mbat; % battery pack mass

% Ocean properties
rho = params.rho;  % density of ocean from CT 

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

% Battery position rp in body frame (relative to the CB).
% rp_z uses (VCG_bat*cos - VCB): at phi_bat=0 this gives the battery CG
% below the CB, matching the measured 0.73 cm offset for SG 194
rp = [x_bat;
      abs(VCG_bat) * sind(phi_bat);
      abs(VCG_bat) * cosd(phi_bat) - VCB];
skew = @(v) [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
rpx = skew(rp); % 3x3 cross product matrix for rp
rsx = skew(r_s); % 3x3 cross product matrix for r_s


%% Translational velocity, alpha, beta

% Horizontal velocity in glide path frame.
% (u_i,v_i) are referenced to the current heading; they differ from the
% glide-path-frame (u,v) by a pure rotation about the vertical axis through
% the heading error psi
u = u_i * cos(psi) - v_i * sin(psi);
v = u_i * sin(psi) + v_i * cos(psi);


% DCM for body frame to glide path frame
DCM_gb = angle2dcm(psi, theta, phi); % DCM glide to body
DCM_bg = DCM_gb.'; % DCM body to glide (it's just an inverse :) )

% Glide path and body frame velocity  
V_g = [u;v;w]; % glide path frame velocity
V_b = DCM_gb * V_g; % body frame velocity
ub = V_b(1);
vb = V_b(2);
wb = V_b(3);
% Note: We CAN implement the water currents in this part. Some frame 
%       silliness may be required.


% Flight path angles
V = sqrt(u^2 + v^2 + w^2);% Speed, norm function is for cowards
if V > 1e-6
    alpha = atan2(wb, ub) * 180/pi;   % deg, body-frame angle of attack
    beta = asin(vb / V) * 180/pi;   % deg, body-frame sideslip
else
    alpha = 0;  beta = 0;
end
%% Mass and Inertia

% Effective translational and rotational inertia matrices
M = ms*eye(3) + Mf; % masses
J = Js + Jf ; % inertias

%% Forces & Coefs

% Dynamic Pressure
q = 0.5 * rho * V^2;

% Clamping aoa and sideslip so coefs don't blow up
alpha_clamp = min(max(alpha, min(alphas)), max(alphas)); %give em the clamps


% Sideslip: the coefficient tables are tabulated for beta >= 0 only.
% For negative sideslip, use the lateral symmetry of the (symmetric)
% vehicle: look up at |beta|, then apply the correct parity --
%   even in beta (CL, CD, Cpitch): coefficient unchanged
%   odd  in beta (CY, Croll, Cyaw): coefficient negated
% This replaces the old scheme that clamped all negative beta to 0.
beta_abs  = min(max(abs(beta), min(betas)), max(betas));
beta_sign = 1;
if beta < 0
    beta_sign = -1;
end

% Coefs from lookup table
% Even-in-beta coefficients (use |beta| directly)
CL     = interp2(betas,alphas,CLs,    beta_abs,alpha_clamp,'linear');
CD     = interp2(betas,alphas,Cds,    beta_abs,alpha_clamp,'linear');
Cpitch = interp2(betas,alphas,Cpitchs,beta_abs,alpha_clamp,'linear');
% Odd-in-beta coefficients (negate for negative sideslip)
CY     = beta_sign * interp2(betas,alphas,Cys,   beta_abs,alpha_clamp,'linear');
Croll  = beta_sign * interp2(betas,alphas,Crolls,beta_abs,alpha_clamp,'linear');
Cyaw   = beta_sign * interp2(betas,alphas,Cyaws, beta_abs,alpha_clamp,'linear');


% Hydrodynamic forces in body frame 
L = q * CL * S; % lift
D = q * CD * S; % drag
Y = q * CY * S; % side
F_hydro = [-D; Y; -L];


% Buoyancy Force -- MAK


% Total displaced volume & mass
Vol_blad = (VBD_cc + 1426.7) * 1e-6;  % oil volume in bladder m^3
Vol_disp = Vol_blad + Vol_static;   % total volume displaced 
Volume = Vol_disp; %* exp(-(Kappa * P - tau * (T - T0))); % -- MAK
m_total = ms + mbat; % total mass of the seaglider (ms will change with damage cases)

% Gravity
g = 9.81;  % m/s^2

% Gravity in body frame
g_b = DCM_gb *[0;0;g]; 

% buoyancy equation 
B = g_b * (m_total - rho * Volume) ;


%% Moments (torques)

% Hydrodynamic moments (torques)
Mp = q * Croll  * b * S; % roll moment
Mq = q * Cpitch * cbar * S; % pitch moment
Mr = q * Cyaw   * b * S; % yaw moment
Torques = [Mp; Mq; Mr]; 


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
Omega = H \ eta_dot;   % Omega = [p; q; r] body rates, rad/s

%% Momentum (finally)

% Battery translational momentum
Pp = mbat * (V_b + cross(Omega, rp));

% Internal-mass coupling vector for the 6x6 mass matrix.
% L&G's general off-diagonal block is (mbat*rp_hat + ms*r_s_hat); the paper
% drops the ms*r_s term only because it specializes r_s = 0.
coupx = mbat*rpx + ms*rsx;

% Starting with vdot = M^-1 Fbar, see algebra in notes
% Feq = cross(M*V_b + Pp, Omega) + m0*g_b + F_hydro;
Feq = cross(M*V_b + Pp, Omega) + B + F_hydro;
veq = [M + mbat*eye(3), -coupx];

% Starting with Omegadot = J^-1 Tbar, see algebra in notes
% Gravity moment now includes BOTH internal masses: the battery
% (mbat at rp) and the stationary mass (ms at r_s)
Teq = cross(J*Omega + cross(rp, Pp), Omega) ...
      + cross(M*V_b + Pp, V_b) ...
      + mbat * cross(rp, g_b) ...
      + ms   * cross(r_s, g_b) ...
      + Torques;
Omegaeq = [coupx, J - mbat*rpx*rpx];

% Accelerations from equation in notes
accels = [veq ; Omegaeq] \ [Feq ; Teq];
Vdot_b = accels(1:3); % translational
Omegadot = accels(4:6); % angular

%% Dx output vector

% Body angular accel to Euler angular accel
eta_Ddot = Hdot * Omega + H * Omegadot ; % chain rule :)

% Body accel to glide accel
Vdot_g = DCM_bg * ( Vdot_b + cross(Omega,V_b) );
udot = Vdot_g(1); % glide-path-frame accelerations
vdot = Vdot_g(2);
wdot = Vdot_g(3);

% Converting glide-path accelerations (udot,vdot) to heading-referenced derivatives
% The state stores u_i, v_i (heading-referenced velocities), which relate
% to the glide-path velocities u, v by a pure rotation about psi:
% [udot; vdot] = Rdot*[u_i;v_i] + R*[u_idot;v_idot]
% ==>  [u_idot; v_idot] = R \ ([udot;vdot] - Rdot*[u_i;v_i])
R = [cos(psi), -sin(psi);
     sin(psi),  cos(psi)];

% psi = heading - heading_desired (heading_desired constant) 
% ==> psidot = headingdot
psidot = headingdot;   % rad/s
Rdot = psidot * [-sin(psi), -cos(psi);
                  cos(psi), -sin(psi)];

% Horizontal and lateral accelerations 
ui_vi_dot = R \ ([udot; vdot] - Rdot*[u_i; v_i]);
u_idot = ui_vi_dot(1); % heading-referenced horizontal acceleration
v_idot = ui_vi_dot(2); % heading-referenced lateral acceleration

% Converting rad/s back to deg/s
phiDdot = eta_Ddot(1) * 180/pi;
thetaDdot = eta_Ddot(2) * 180/pi;
headingDdot = eta_Ddot(3) * 180/pi;
phidot = phidot * 180/pi;
thetadot = thetadot * 180/pi;
headingdot = headingdot * 180/pi;

% Output
X_dyn = [phiDdot; thetaDdot; headingDdot; phidot; thetadot; headingdot; ...
         u_idot; v_idot; wdot; u_i; v_i; w];

end
