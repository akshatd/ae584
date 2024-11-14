close all
clear all
clc

%% Stop conditions for ODE45 simulation
t = 0:0.01:10;
finPos = 10;
finVel1 = 2;
finVel2 = 10;

% For more information about events, go to https://www.mathworks.com/help/matlab/math/ode-event-location.html

% The events yield three vector variables. If at any point in the
% simulation, any of the components of the "value" variable changes sign
% between the steps evaluated by ode45, and the corresponding component in
% the "isterminal" variable is 1, then the simulation is terminated. If the
% corresponding value of the "direction" variable is 0, then it accepts
% sign changes from positive to negative and negative to positive.

% Set a position stop condition
optPos = odeset('Events', @(t,x)StopEventPos(t,x,finPos), 'RelTol', 1e-6, 'AbsTol', 1e-6);

% Set both a position and a velocity conditions. The simulation will end when either of these is met. 
optPosVel1 = odeset('Events', @(t,x)StopEventPosVel(t,x,finPos,finVel1));
optPosVel2 = odeset('Events', @(t,x)StopEventPosVel(t,x,finPos,finVel2));

% Position condition is met
[tP, xP] = ode45(@(t,x)dyn_fun(t,x,1), t, [0;0], optPos);

% Velocity condition is met before position condition
[tPV1, xPV1] = ode45(@(t,x)dyn_fun(t,x,1), t, [0;0], optPosVel1);

% Position condition is met before velocity condition
[tPV2, xPV2] = ode45(@(t,x)dyn_fun(t,x,1), t, [0;0], optPosVel2);


%% Plots
% Position condition is met
figure(1)
plot(tP, xP, 'Linewidth', 2)
grid on
set(gca,'FontSize',12)
set(gca,'TickLabelInterpreter','latex')
xlabel('$t$ (s)','interpreter','latex')
legend({'Position (m)','Velocity (m/s)'},'interpreter','latex')

% Velocity condition is met before position condition
figure(2)
plot(tPV1, xPV1, 'Linewidth', 2)
grid on
set(gca,'FontSize',12)
set(gca,'TickLabelInterpreter','latex')
xlabel('$t$ (s)','interpreter','latex')
legend({'Position (m)','Velocity (m/s)'},'interpreter','latex')

% Position condition is met before velocity condition
figure(3)
plot(tPV2, xPV2, 'Linewidth', 2)
grid on
set(gca,'FontSize',12)
set(gca,'TickLabelInterpreter','latex')
xlabel('$t$ (s)','interpreter','latex')
legend({'Position (m)','Velocity (m/s)'},'interpreter','latex')

%% Double integrator dynamics.
function xd = dyn_fun(t, x, u)
    xd = zeros(2,1);
    xd(1) = x(2);
    xd(2) = u;
end

%% Event for finishing the simulation once the position is near a given value.
function [value, isterminal, direction] = StopEventPos(t, x, pos)
    value      = x(1) - pos;
    isterminal = 1;   % Stop the integration
    direction  = 0;
end

%% Event for finishing the simulation if the position is near a given value or the velocity is over a given value.
function [value, isterminal, direction] = StopEventPosVel(t, x, pos, vel)
    cond1 = x(1)- pos;
    cond2 = x(2)- vel;
    value      = [cond1; cond2];
    isterminal = [1 ; 1];   % Stop the integration
    direction  = [0; 0];
end