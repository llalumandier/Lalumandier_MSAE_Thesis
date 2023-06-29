function efun = UASEvents(primal)
% Creates an array of state boundary conditions

% Call preamble and load primal variables:
[ x, y, h, psi, dm, ...                          % states
            Va, gam, phi, gam_a, Vg, ...         % controls 
            t, ...                               % time
            x0, y0, h0, psi0, m0, t0,  ...       % initial conditions          
            xf, yf, hf, psif, mf, tf,...         % endpoints   
            g, eta, Kp1, Kp2, M0, ...            % constants
            L, H, V, M, T, DM] ...               % scaling
            = UASPreamble(primal);

% preallocate boundary conditions
efun = zeros(7, 1); 

%===========================================================
efun(1) = x0;
efun(2) = y0;
efun(3) = h0;
efun(4) = m0;
%-----------------------------------------------------------
efun(5) = xf;
efun(6) = yf;
efun(7) = hf;
%===========================================================
