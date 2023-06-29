function dxdt = UASDynamics(primal)
% Requires input of fuel burn model at []

% Call preamble and load primal variables:
[ x, y, h, psi, dm, ...                          % states
            Va, gam, phi, gam_a, Vg, ...         % controls 
            t, ...                               % time
            x0, y0, h0, psi0, m0, t0,  ...       % initial conditions          
            xf, yf, hf, psif, mf, tf,...         % endpoints   
            g, eta, Kp1, Kp2, M0, ...            % constants
            L, H, V, M, T, DM] ...               % scaling
            = UASPreamble(primal);

%Unscale to Eng units
Va_eng = Va.*V;
m_eng = dm.*DM + M0;

% Import Wind Values
[wn, we, wd] = UASWeather(primal);

%Call Density Model
rho = UASDensity(primal);

% Calculate weight
W = m_eng.*g; 

%=========================================================================        
% Equations of Motion:
%=========================================================================
xdot = Va_eng.*cos(psi).*cos(gam_a) + wn;
ydot = Va_eng.*sin(psi).*cos(gam_a) + we;
hdot = Va_eng.*sin(gam_a) - wd;
psidot = g./Va_eng.*tan(phi);
dmdot = []; %Enter Fuel burn model here


%=========================================================================

% Create an array of EoMs and apply scaling:
dxdt = [ xdot.*T./L; 
         ydot.*T./L; 
         hdot.*T./H;
         psidot.*T;
         dmdot.*T./DM];