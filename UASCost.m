function [ECost, FCost] = UASCost(primal)
% Defines the cost (power required),
% as derived in Appendix A1 of thesis test

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
% UAS cost function
%=========================================================================

%Unscale to Eng units
Va_eng = Va.*V;
m_eng = dm.*DM + M0;

% Calculate weight
W = m_eng.*g;           

% Call atmos model and scale
rho = UASDensity(primal);

%Cost function
ECost   = 0; %no end point cost
FCost   = (Va_eng.^3.*rho.*Kp1 + W.^2./(Va_eng.*rho).*cos(gam).^2./cos(phi).^2.*Kp2 + ...
          Va_eng.*W.*sin(gam)./eta) + 1e-4.*(Va_eng.^2 + Vg.*V.^2 + gam.^2 + phi.^2 + gam_a.^2);

% Apply Scaling to the Cost
FCost = FCost.*T.^3./(M.*L.^2);