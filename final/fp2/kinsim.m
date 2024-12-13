function dx  = kinsim(t,x)
% Returns xdot for kinematics simulation engagement
% Written by Syed Aseem Ul Islam (aseemisl@umich.edu) 17 Nov 2020

dx = zeros(8,1);
% x(1)    :: hP
% x(2)    :: dP
% x(3)    :: gammaP
% x(4)    :: hE
% x(5)    :: dE
% x(6)    :: gammaE
% x(7)    :: R
% x(8)    :: beta

VP = ; %m/s
VE = ; %m/s
g  = 9.81; %m/s^2

% Evader normal acceleration
nzE = ; %m/s^2


dx(1) = ;
dx(2) = ;
% dx(3) needs to be implemented after dx(7) and dx(8) since it relies on Rdot betadot
dx(4) = ;
dx(5) = ;
dx(6) = ;
dx(7) = ;
dx(8) = ;

% Pursuer normal acceleration (Proportional Guidance)
nzP = ;

% Saturate pursuer normal acceleration at 40g
if abs(nzP)>40*g
    nzP = sign(nzP)*40*g;
end
dx(3) = ;





end

