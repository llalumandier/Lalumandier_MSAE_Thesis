%% UAS Optimal Control in the presence of a time-varying windfeild.
% Author: Luke Lalumandier
% Archived Date: April 28, 2023
% Version 1.1.7
%=========================================================================
% Launch DIDO
%=========================================================================
startupDIDO
TestDIDO

%% =======================================================================
% Clear Workspace and Load Aircraft Data
%=========================================================================
clear all; close all; clc; %#ok<CLALL> supress MATLAB warning

% Define Constants and AC Data
[Kp1, Kp2, eta, M0, V, g] = getUAS(); 

%=========================================================================
% Define the Problem (All User Inputs)
%=========================================================================
% Boundary Conditions
x0 = 0;     xf = 10000;
y0 = 0;     yf = 0;
h0 = 1000;     hf = 2000;
m0 = 0;

% Set Search States
x_search = [-100,10100];
y_search = [-100,2500];
h_search = [10,3000];
psi_search = [-pi,pi];
m_search = [-0.01,0]; 

% Search Controls
Va_search = [0.6*V, 2.4*V];
gam_search = [-pi/6,pi/6];
phi_search = [-pi/43,pi/43];
gam_a_search = [-pi/6,pi/6];
Vg_search = [0.6*V, 2.4*V];

% Set DIDO Nodes
algorithm.nodes = 21;

% Define Constants
constants.MyConstants.g    = g;    
constants.MyConstants.eta  = eta;      
constants.MyConstants.Kp1  = Kp1;     
constants.MyConstants.Kp2  = Kp2;
constants.MyConstants.M0   = M0;    


%=========================================================================
% Scaling
%=========================================================================
L = sqrt( (x0 - xf)^2 + (y0 - yf)^2 + (h0 - hf)^2); 
% V is given by getUAS()
DM = 0.0028; %Estimate of fuel burned
M = M0; 
T = L/V;
H = (h0 + hf)/2;

constants.SCALES.L = L;
constants.SCALES.H = H;
constants.SCALES.V = V;
constants.SCALES.M = M;
constants.SCALES.T = T;
constants.SCALES.DM = DM;

%=========================================================================
% Prepare DIDO
%=========================================================================
% Set Max Time
tfMax = 2*L/V;

% Define the Search States
search.states     = [x_search./L;        
                     y_search./L;        
                     h_search./H;
                     psi_search;
                     m_search./DM];       
                 
search.controls   = [Va_search./V;
                     gam_search;
                     phi_search;
                     gam_a_search;
                     Vg_search./V];      
                                    
% Define the Boundary Conditions
bounds.events     = [x0./L, x0./L;          
                     y0./L, y0./L;          
                     h0./H, h0./H;
                     m0./M, m0./M;        
                     xf./L, xf./L;
                     yf./L, yf./L;
                     hf./H, hf./H];    

% Define Path Constraints (scale units)
bounds.path       = [0,0;
                     0,0;
                     50/H, 10000/H;
                     0.95*M0, 2*M0;
                     0,475.*T.^3./(M.*L.^2)]; % Hybrid Tiger Estimate
 
                                                                  
% Define Time
bounds.initial.time     = [0, 0];       % t0 = 0
bounds.final.time       = [0.01, tfMax/T];   % tf is free (expect tf < tfMax)

% Define Problem in Terms of DIDO Inputs and Check DIDO Format
UAS.cost         = 'UASCost';
UAS.dynamics     = 'UASDynamics';
UAS.events       = 'UASEvents';
UAS.path         = 'UASPath';
UAS.bounds       = bounds;   
UAS.search       = search;
UAS.constants    = constants;

check(UAS);

disp('Please Work')

%=========================================================================
% Run DIDO
%=========================================================================
tic %Begin run-time clock
[cost, primal, dual] = dido(UAS, algorithm);
toc %Output run-time

% Process outputs
[ x, y, h, psi, dm, ...                          % states
            Va, gam, phi, gam_a, Vg, ...         % controls 
            t, ...                               % time
            x0, y0, h0, psi0, m0, t0,  ...       % initial conditions          
            xf, yf, hf, psif, mf, tf,...         % endpoints   
            g, eta, Kp1, Kp2, M0, ...            % constants
            L, H, V, M, T, DM] ...               % scaling
            = UASPreamble(primal);

lam_x = dual.dynamics(1,:);
lam_y = dual.dynamics(2,:);
lam_h = dual.dynamics(3,:);
lam_psi = dual.dynamics(4,:);
lam_m = dual.dynamics(5,:);
mu1  = dual.path(1,:);
mu2  = dual.path(2,:);
mu3  = dual.path(3,:);
mu4  = dual.path(4,:);
mu5  = dual.path(5,:);
Ham = dual.Hamiltonian;           
rho = UASDensity(primal);
[wn, we, wd] = UASWeather(primal);

%=========================================================================
% Create Text Outputs
%=========================================================================

% ---------------------
% Unscale parameters for Pr
Va_eng = Va.*V;
m_eng = dm.*DM + M0;
% ---------------------

%Calculate Weight
W = m_eng.*g;

Pr = (Va_eng.^3.*rho.*Kp1 + W.^2./(Va_eng.*rho).*cos(gam).^2./cos(phi).^2.*Kp2 + ...
          Va_eng.*W.*sin(gam)./eta); 

P_int = sum(Pr).*t(end).*T./algorithm.nodes;
P_msg = ['The energy expended across the solved trajectory is ',num2str(P_int),' J.'];
disp(P_msg)

Fburn = (m0 - dm(end)).*DM;
Fburn_msg = ['The UAS burned ',num2str(Fburn),' kg of fuel to complete this trajectory.'];
disp(Fburn_msg)

Fburn_msg = ['The UAS flew for ',num2str(t(end)*T),' seconds.'];
disp(Fburn_msg)
