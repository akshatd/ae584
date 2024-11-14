close all
clear all
clc

%% Stop conditions for ODE45 simulation timestep-by-timestep
Ts = 0.01;
t = 0:Ts:10;
finPos = 10;

% For more information about events, go to https://www.mathworks.com/help/matlab/math/ode-event-location.html

% This is a continuation from the explanation in the file "ode45_Stop_Conditions.m"

% Suppose that the input of a system is updated every Ts seconds and that
% it depends on a noisy measurement (in this example, u(k) = 1 + randn(1)).

% Events can still be used in this case, by running ode45 by parts, that
% is, using a for-loop, use ode45 to simulate the system in between Ts
% seconds and update the input before running ode45.

% For a given loop index, if the output variable "ie" from ode45 is
% nonempty, this implies that an event happened during that simulation,
% implying that the loop should be ended.

% Set a position stop condition
optPos = odeset('Events', @(t,x)StopEventPos(t,x,finPos), 'RelTol', 1e-6, 'AbsTol', 1e-6);

xP = zeros(2,length(t));
xP(:,1) = [0; 0];
tP = t;
idxEnd = 1;

for ii = 2:length(t)
    u = 1 + randn(1);
    [tt, xx, ~, ~, ie] = ode45(@(t,x)dyn_fun(t,x,u), [ii-2 ii-1]*Ts, xP(:,ii-1), optPos);
    idxEnd = ii;
    xP(:,ii) = xx(end,:)';
    if (~isempty(ie))
       tP(ii) = tt(end);
       break;
    end
end

%% Plots
% Position condition is met
figure(1)
kk1 = 1:idxEnd;
plot(tP(kk1), xP(:,kk1), 'Linewidth', 2)
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