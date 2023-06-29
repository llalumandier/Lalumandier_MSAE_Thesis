function [Kp1, Kp2, eta, m0, V, g] = getUAS()
% Aircraft Specifications
% Insert values where [] is present 

%=========================================================================
% Aircraft Parameters (User Inputs)
%=========================================================================
m0 = [];         %AC mass at TO (kg)
Cd0 = [];      %fuselage drag (ND)
S = [];          %surface area (m^2)
eta = [];         %propulsion efficiency (ND)
e_osv = [];       %oswald efficiency factor (ND)
span = [];        %wing spam (m)

%=========================================================================
% Calculate Useful Values
%=========================================================================
% SL Constants
g = 9.815;          %gravity (m/s^2)
rho_SL = 1.225;     %Density (kg^2/m^3)

% Useful Parameters
W = m0*g;           %AC Weight at TO
AR = span^2/S;      %AR (ND)

% Kp Values
Kp = 1/(e_osv *pi*AR);
Kp1 = S*Cd0/(2*eta);
Kp2 = 2*Kp/(eta*S);

% Scale Velocity
V = sqrt(2*W/(rho_SL*S) * sqrt(Kp/(3*Cd0)));
