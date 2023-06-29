function [ x, y, h, psi, dm, ...                          % states
            Va, gam, phi, gam_a, Vg, ...         % controls 
            t, ...                               % time
            x0, y0, h0, psi0, m0, t0,  ...       % initial conditions          
            xf, yf, hf, psif, mf, tf,...         % endpoints   
            g, eta, Kp1, Kp2, M0, ...            % constants
            L, H, V, M, T, DM] ...               % scaling
            = UASPreamble(primal)

%   States
x = primal.states(1,:);                 
y = primal.states(2,:);
h = primal.states(3,:);
psi = primal.states(4,:);
dm = primal.states(5,:);

% Controls
Va = primal.controls(1,:);  
gam = primal.controls(2,:); 
phi = primal.controls(3,:);
gam_a = primal.controls(4,:);
Vg = primal.controls(5,:);

% Time
t = primal.time;                   

%Initial States
x0 = primal.initial.states(1);       
y0 = primal.initial.states(2);
h0 = primal.initial.states(3);
psi0 = primal.initial.states(4);
m0 = primal.initial.states(5);

% Initial Time
t0 = primal.initial.time;               

% Final States
xf = primal.final.states(1);         
yf = primal.final.states(2);
hf = primal.final.states(3);
psif = primal.final.states(4);
mf = primal.final.states(5);

% Final Time
tf = primal.final.time;                 

% Constants (data)
g  = primal.constants.MyConstants.g;                
eta = primal.constants.MyConstants.eta;
Kp1 = primal.constants.MyConstants.Kp1;
Kp2 = primal.constants.MyConstants.Kp2;
M0 = primal.constants.MyConstants.M0; 

%Scaling
L = primal.constants.SCALES.L;
H = primal.constants.SCALES.H;
V = primal.constants.SCALES.V;
M = primal.constants.SCALES.M;
T = primal.constants.SCALES.T;
DM = primal.constants.SCALES.DM;