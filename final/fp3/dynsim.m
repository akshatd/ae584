function dx  = dynsim(t,x,nzE)
% Returns xdot for dynamics engagement simulation.
% Takes evader normal acceleration as input
% Written by Syed Aseem Ul Islam (aseemisl@umich.edu) 17 Nov 2020.
% Finished by: Akshat Dubey 13 Dec 2024

VP = x(1); % Pursuer velocity, m/s
gammaP = x(2); % Pursuer flight path angle, rad
hP = x(3); % Pursuer altitude, m
dP = x(4); % Pursuer downrange, m
VE = x(5); % Evader velocity, m/s
gammaE = x(6); % Evader flight path angle, rad
hE = x(7); % Evader altitude, m
dE = x(8); % Evader downrange, m

g = 9.81; % Gravity, m/s^2

% Based on Problem Setup, near eq 50
SP = 2.3; % Reference area for drag calculations (pursuer), m^2
SE = 28; % Reference area for drag calculations (evader), m^2
mP = 130; % Mass of pursuer, kg
mE = 10000; % Mass of evader, kg

[~, aP, ~, rhoP] = atmosisa(hP); % speed of sound aP and air density rhoP at altitude hP
[~, aE, ~, rhoE] = atmosisa(hE); % speed of sound aE and air density  rhoE at altitude hE
[~, ~, ~, rho0] = atmosisa(0); % air density rho0 at sea level

MiP = [0 0.6 0.8 1 1.2 2 3 4 5]; % MP data points for pursuer Cd, eq 49
CdiP = [0.016 0.016 0.0195 0.045 0.039 0.0285 0.024 0.0215 0.020]; % Cd data points for pursuer Cd, eq 50
CdP = pchip(MiP, CdiP, VP/aP); % CdP at atltitude hP and Mach MP

MiE = [0 0.9 1 1.2 1.6 2]; % ME data points for evader Cd, eq 53
CdiE = [0.0175 0.019 0.05 0.045 0.043 0.038]; % Cd data points for evader Cd, eq 54
CdE = pchip(MiE, CdiE, VE/aE); % CdE at atltitude hE and Mach ME

TP = 0; % Thrust profile for AIM120C5 pursuer, N, eq 47
if 0 <= t && t < 10
	TP = 11000;
elseif 10 <= t && t < 30
	TP = 1800;
elseif t >= 30
	TP = 0;
end

TE = rhoE/rho0 * 76310; % Turbofan thrust approximation for evader, N, eq 51

% Range rate, m/s
R_dot = ((hE-hP)*(VE*sin(gammaE) - VP*sin(gammaP)) + (dE-dP)*(VE*cos(gammaE) - VP*cos(gammaP)))/sqrt((hE-hP)^2 + (dE-dP)^2);
% LOS Bearing rate, rad/s
beta_dot = ((dE-dP)*(VE*sin(gammaE) - VP*sin(gammaP)) - (hE-hP)*(VE*cos(gammaE) - VP*cos(gammaP)))/((dE-dP)^2 + (hE-hP)^2);

% Pursuer normal acceleration (Gravity Corrected Proportional Guidance)
nzP = -3*abs(R_dot)*beta_dot - g*cos(gammaP);

% Saturate pursuer normal acceleration at 40g
if abs(nzP)>40*g
	nzP = sign(nzP)*40*g;
end

VP_dot = (TP-drag(rhoP, VP, SP, CdP))/mP - g*sin(gammaP);
gammaP_dot = -(nzP + g*cos(gammaP))/VP;
hP_dot = VP*sin(gammaP);
dP_dot = VP*cos(gammaP);
VE_dot = (TE-drag(rhoE, VE, SE, CdE))/mE - g*sin(gammaE);
gammaE_dot = -(nzE + g*cos(gammaE))/VE;
hE_dot = VE*sin(gammaE);
dE_dot = VE*cos(gammaE);

dx = [VP_dot; gammaP_dot; hP_dot; dP_dot; VE_dot; gammaE_dot; hE_dot; dE_dot];

end

function drag_force = drag(rho, V, S, Cd)
drag_force = 0.5*rho*V^2*S*Cd;
end
