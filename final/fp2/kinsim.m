function dx  = kinsim(t,x)
% Returns xdot for kinematics simulation engagement
% Written by Syed Aseem Ul Islam (aseemisl@umich.edu) 17 Nov 2020
% Finished by: Akshat Dubey 13 Dec 2024

hP = x(1); % Pursuer altitude, m
dP = x(2); % Pursuer downrange, m
gammaP = x(3); % Pursuer flight path angle, rad
hE = x(4); % Evader altitude, m
dE = x(5); % Evader downrange, m
gammaE = x(6); % Evader flight path angle, rad
R = x(7); % Range, m
beta = x(8); % LOS Bearing, rad

% Based on Problem 2.i
VP = 900; % Pursuer velocity, m/s
VE = 450; % Evader velocity, m/s

g = 9.81; % Gravity, m/s^2

nzE = 0; % Evader normal acceleration, m/s^2
if 0 <= t && t < 9
	nzE = -8*g;
elseif 9 <= t && t < 10
	nzE = 0;
elseif 10 <= t && t < 22
	nzE = -6*g;
elseif t >= 22
	nzE = 0;
end

hP_dot = VP*sin(gammaP);
dP_dot = VP*cos(gammaP);
% skip gammaP_dot for now
hE_dot = VE*sin(gammaE);
dE_dot = VE*cos(gammaE);
gammaE_dot = -1/VE*nzE;
R_dot = VE*cos(beta - gammaE) - VP*cos(beta - gammaP);
beta_dot = (VP*sin(beta - gammaP) - VE*sin(beta - gammaE))/R;

% Pursuer normal acceleration (Proportional Guidance Law)
nzP = -3*abs(R_dot)*beta_dot;

% Saturate pursuer normal acceleration at 40g
if abs(nzP)>40*g
	nzP = sign(nzP)*40*g;
end

gammaP_dot = -1/VP*nzP;

dx = [hP_dot; dP_dot; gammaP_dot; hE_dot; dE_dot; gammaE_dot; R_dot; beta_dot];

end
