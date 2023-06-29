function [wn, we, wd] = UASWeather(primal)

% Call preamble and load primal variables:
[ x, y, h, psi, dm, ...                          % states
            Va, gam, phi, gam_a, Vg, ...         % controls 
            t, ...                               % time
            x0, y0, h0, psi0, m0, t0,  ...       % initial conditions          
            xf, yf, hf, psif, mf, tf,...         % endpoints   
            g, eta, Kp1, Kp2, M0, ...            % constants
            L, H, V, M, T, DM] ...               % scaling
            = UASPreamble(primal);

%=========================================================================        
% Define Simulated Wind Feild 
%=========================================================================

% -----------------
% Put x in Eng units
x_eng = x.*L;
y_eng = y.*L;
% -----------------


% North Wind
wn = 0.0001*y;

% East Wind
we = 0.0001*x;

draftx = 10000;
drafty = 2000;
% Down Wind
wd =  2./drafty.^2.*(y_eng-drafty).^2 - 2; 