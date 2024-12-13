function dx  = dynsim(t,x)
% Computes xdot for dynamics simulation engagement.
% Written by Syed Aseem Ul Islam (aseemisl@umich.edu) 17 Nov 2020.

dx = zeros(8,1);
% x(1)    :: VP
% x(2)    :: gammaP
% x(3)    :: hP
% x(4)    :: dP
% x(5)    :: VE
% x(6)    :: gammaE
% x(7)    :: hE
% x(8)    :: dE


g = 9.81; %m/s^2


% Evader normal acceleration
nzE =  ; %m/s^2


SP = ; %Reference area for drag calculations (pursuer)
SE = ; %Reference area for drag calculations (evader)
mP = ; %mass of pursuer in kg
mE = ; %mass of evader in kg


; %use ISA to compute speed of sound aP and air density rhoP at altitude hP
; %use ISA to compute speed of sound aE and air density  rhoE at altitude hE
; %use ISA to air density rho0 at sea level

MiP = ; %MP data points for pursuer Cd
CdiP = ; %Cd data points for pursuer Cd
CdP = pchip(MiP,CdiP,x(1)/aP); % CdP at atltitude hP and Mach MP
MiE = ; %ME data points for evader Cd
CdiE = ; %Cd data points for evader Cd
CdE = pchip(MiE,CdiE,x(5)/aE); % CdE at atltitude hE and Mach ME
TE = ; %Turbofan thrust approximation for evader in N


% Thrust profile for AIM120C5 pursuer
TP =; %N



Rdot = ;
betadot = ;

% Pursuer normal acceleration (Gravity Corrected Proportional Guidance)
nzP = ;

% Saturate pursuer normal acceleration at 40g
if abs(nzP)>40*g
    nzP = sign(nzP)*40*g;
end


dx(1) = ;
dx(2) = ;
dx(3) = ;
dx(4) = ;

dx(5) = ;
dx(6) = ;
dx(7) = ;
dx(8) = ;



end

