function rho = UASDensity(primal)

% Call preamble and load primal variables:
[ x, y, h, psi, dm, ...                          % states
            Va, gam, phi, gam_a, Vg, ...         % controls 
            t, ...                               % time
            x0, y0, h0, psi0, m0, t0,  ...       % initial conditions          
            xf, yf, hf, psif, mf, tf,...         % endpoints   
            g, eta, Kp1, Kp2, M0, ...            % constants
            L, H, V, M, T, DM] ...               % scaling
            = UASPreamble(primal);

%-----------------------------
% Metric Exponential Model
%-----------------------------
% Unscale Alt
h = h.*H;

% Exponential Model
rho = 1.2245*exp(-h/10400.);