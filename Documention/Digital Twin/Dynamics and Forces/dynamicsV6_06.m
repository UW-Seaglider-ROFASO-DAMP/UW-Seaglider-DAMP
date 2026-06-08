function X_dyn = dynamicsV6_06(X,U,coefs,params)
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

% Battery longitudinal location, measured from the CB (body +x = nose).
% pitch_ctl is the commanded shift about the pitch-rail mechanical zero;
% X_BAT0 places that zero relative to the CB (from SG 194 trim sheet:
% battery CG 5.09 cm forward of the CB at trim-sheet-neutral).
% NOTE: X_BAT0 and params.r_s MUST be derived from the same neutral
% configuration -- their longitudinal moments cancel at trim.
if isfield(params,'X_bat0'), X_bat0 = params.X_bat0; else, X_bat0 = 0.0509; end
x_bat = X_bat0 - pitch_ctl ; % battery location relative to CB, m

% Unpacking parameters

% Target information
heading_desired = deg2rad(params.heading_desired); % desired heading for dive

% Aerodynamic surfaces properties
S = params.S; % wing surface area
b = params.b; % wing span

% Glider properties

% Whole glider
Vol_static = params.Vstatic; % displaced volume w/o VBD
Mf = params.Mf; % 3x3 added mass matrix, should be changed later?
Jf = params.Jf; % 3x3 added mass inertia matrix, should be changed later?
ms = params.Ms; % stationary mass
Js = params.Js; % 3x3 stationary mass inertia matrix (referenced to the CB)
VCB = 0.00362; % Vertical CB location relative to neutral axis, in m for SG 194
% VCB = 0;

% Stationary-mass CG offset from the CB, body frame [x;y;z], m.
% L&G specialize r_s = 0 (static mass centered on CB); SG 194 is not built
% that way -- the trim sheet implies r_s ~ [-0.0125; 0; 0.0041] m.
if isfield(params,'r_s'), r_s = params.r_s(:); else, r_s = [-0.0125; 0; 0.0041]; end

% Battery
VCG_bat = 0.01082; % Battery vertical cg location relative to neutral axis, in m for SG 194
% rpbat = sqrt( VCB^2 + VCG_bat^2 - 2 * VCB* VCG_bat * cosd(phi_bat) ); % distance from cb to cg of battery (in YZ plane)
mbat = params.mbat; % battery pack mass

% Kappa = 5.529e-06; % compressibility factor, number from Dr. Charlie Erikson Paper-- MAK
% tau = 7.05e-05; % Volumetric expansion, number from Dr. Charlie Erikson Paper -- MAK 
% PAPER: Assessing Seaglider Model-Based Position Accuracy on an Acoustic Tracking Range

% Ocean properties
rho = params.rho;  % density of ocean from CT 
% salt = params.salt; % salinity contesnt of water from sensor?
T0 = params.ambtemp; % ambient surface temp of water 
T = params.temp; % temperature
P = params.pressure; % pressure from ct sail?

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
% below the CB, matching the measured 0.73 cm offset for SG 194.
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
% the heading error psi. Roll (phi) is NOT involved here -- it is handled
% separately by the full DCM_gb when converting to the body frame.
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

% Clamp alpha to the table range.
alpha_clamp = min(max(alpha, min(alphas)), max(alphas));

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
% VBD_ctlcc = VBD_ctlAD * -0.2453;    % converting VBD_ctl from AD to cm^3, VBD_ctlAD is control input
Vol_blad = (VBD_cc + 1426.7) * 1e-6;     % oil volume in bladder (m^3 now)
Vol_disp = Vol_blad + Vol_static;   % total volume displaced 
Volume = Vol_disp; %* exp(-(Kappa * P - tau * (T - T0))); % -- MAK
m_total = ms + mbat;       % total mass of the seaglider (ms will change with damage cases)
% m_disp = Vol_disp * rho;             % Not being used

% Gravity
g = 9.81;  % m/s^2

% Gravity in body frame
g_b = DCM_gb *[0;0;g];

% buoyancy equation 
B = g_b * (m_total - rho * Volume) ;


%% Moments (torques)

% Hydrodynamic moments (torques)
% All moments use span (b) as the reference length, matching the
% wind-tunnel data reduction (which normalizes pitch by span, not cbar).
Mp = q * Croll  * b * S; % roll moment
Mq = q * Cpitch * b * S; % pitch moment
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

%% Rotational rate damping (hydrodynamic)
% % The static lookup-table moments (Mp,Mq,Mr above) depend only on alpha/beta
% % and carry NO dependence on angular rate, which leaves all three rotational
% % modes undamped. Add linear rate-damping moments built from the standard
% % non-dimensional body-rate terms:
% %       p_hat = p*b/(2V)   q_hat = q*cbar/(2V)   r_hat = r*b/(2V)
% % Damping derivatives (Cl_p, Cm_q, Cn_r) must be NEGATIVE so the moment
% % opposes the rotation.
% %
% % NOTE: these defaults are TUNABLE PLACEHOLDERS. They are not derived from
% % the wind-tunnel static data (static tests cannot measure rate damping --
% % that needs forced-oscillation tests, CFD, or system ID from flight data).
% % They can be overridden per-run via params.Cl_p / params.Cm_q / params.Cn_r.
% if isfield(params,'Cl_p'), Cl_p = params.Cl_p; else, Cl_p = -2.0; end  % roll-rate damping deriv
% if isfield(params,'Cm_q'), Cm_q = params.Cm_q; else, Cm_q = -2.0; end  % pitch-rate damping deriv
% if isfield(params,'Cn_r'), Cn_r = params.Cn_r; else, Cn_r = -8.0; end  % yaw-rate damping deriv
% 
% Veff  = max(V, 1e-3);                  % guard against divide-by-zero at low speed
% p_hat = Omega(1) * b / (2*Veff);       % non-dim roll rate
% q_hat = Omega(2) * b / (2*Veff);       % non-dim pitch rate (span ref, matches Mq)
% r_hat = Omega(3) * b / (2*Veff);       % non-dim yaw rate
% 
% % Damping moment increments (q here is dynamic pressure, set in Forces section)
% % All use span (b) as reference length, consistent with the static moments.
% Mp_damp = q * Cl_p * p_hat * b * S; % roll
% Mq_damp = q * Cm_q * q_hat * b * S; % pitch
% Mr_damp = q * Cn_r * r_hat * b * S; % yaw
% 
% Torques = Torques + [Mp_damp; Mq_damp; Mr_damp];

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
% (mbat at rp) and the stationary mass (ms at r_s). The static-mass
% term was previously missing -- it supplies the vertical pendulum
% stiffness the trim sheet shows SG 194 actually has.
% NOTE: Js (hence J) is assumed referenced to the CB. The battery is a
% separate point mass, so only it contributes the -mbat*rpx*rpx rotational
% term; the stationary mass's rotational inertia is already in Js. The
% ms*r_s contribution enters ONLY the off-diagonal coupling block (coupx).
% If Js were instead referenced to the stationary-mass CG, also add
% - ms*rsx*rsx to the rotational block below.
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

% --- Convert glide-path accelerations (udot,vdot) to state derivatives ---
% The state stores u_i, v_i (heading-referenced velocities), which relate
% to the glide-path velocities u, v by a pure rotation about psi:
%       u = u_i*cos(psi) - v_i*sin(psi)
%       v = u_i*sin(psi) + v_i*cos(psi)
% i.e.  [u; v] = R * [u_i; v_i]
% Differentiating:  [udot; vdot] = Rdot*[u_i;v_i] + R*[u_idot;v_idot]
% so  [u_idot; v_idot] = R \ ([udot;vdot] - Rdot*[u_i;v_i])
% R is a proper rotation: det(R)=1 always, so it is never singular.
R = [cos(psi), -sin(psi);
     sin(psi),  cos(psi)];

% psi = heading - heading_desired (heading_desired constant) => psidot = headingdot.
% NOTE: headingdot is still in rad/s at this point (deg conversion is below).
psidot = headingdot;   % rad/s
Rdot = psidot * [-sin(psi), -cos(psi);
                  cos(psi), -sin(psi)];

ui_vi_dot = R \ ([udot; vdot] - Rdot*[u_i; v_i]);
u_idot = ui_vi_dot(1); % heading-referenced horizontal acceleration
v_idot = ui_vi_dot(2); % heading-referenced lateral acceleration
% w (vertical) is already a glide-path-frame state, so wdot needs no change.

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

% % Diagnostic
% tau_bat = mbat * cross(rp, g_b);
% 
% fprintf('phi_bat = %.3f deg\n', phi_bat);
% fprintf('rp_y    = %.6f m\n', rp(2));
% fprintf('tau_x   = %.6f N*m\n', tau_bat(1));

end
