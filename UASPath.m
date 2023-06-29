function hfun = UASPath(primal)
% Defines Path Constraints

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

% Call Weather Data
[wn, we, wd] = UASWeather(primal);

%Apply scaling to Wind
wn = wn/V; 
we = we/V; 
wd = wd/V; 

% Call atmos model and scale
rho = UASDensity(primal);

% Calculate weight
W = m_eng.*g; 

%===========================================================
% Load all path constraints
%===========================================================
hfun(1,:) = (Va.*sin(gam_a) - wd)./Vg - sin(gam);     % 2.11
hfun(2,:) = -Vg.^2 + Va.^2 + wn.^2 + we.^2 + wd.^2 + ...
 2.*Va.*(cos(psi).*cos(gam_a).*wn + sin(psi).*cos(gam_a).*we - ...
 sin(gam_a).*wd); 
hfun(3,:) = h;                                        % Altitude Bound
hfun(4,:) = m_eng;
hfun(5,:) = (Va_eng.^3.*rho.*Kp1 + W.^2./(Va_eng.*rho).*cos(gam).^2./cos(phi).^2.*Kp2 + ...
          Va_eng.*W.*sin(gam)./eta).*T.^3./(M.*L.^2);    %Power constraint