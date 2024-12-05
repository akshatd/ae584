clc; clear; close all;

% set up sim
v_T = 5; % m/s
v_M = 6; % m/s
theta_T = pi/2; % rad, pointing north

% initial conditions
x_T_0 = 100; % m
y_T_0 = 0; % m
x_M_0 = 0; % m/s
y_M_0 = 0; % m/s
R_0 = sqrt((x_T_0 - x_M_0)^2 + (y_T_0 - y_M_0)^2); % m
beta_0 = 0; % rad, pointing east to target
x_0 = [x_T_0; y_T_0; x_M_0; y_M_0; R_0; beta_0]; % x_T, y_T, x_M, y_M, R, beta
fin_x_M = 100; % m

% simulate
tspan = 0:0.01:100;
stop_pos = odeset("Events", @(t, x) stop_x_M(t, x, fin_x_M), "RelTol", 1e-3, "AbsTol", 1e-3);
[t, x] = ode45(@(t, x) dp_law(t, x, v_T, v_M, theta_T), tspan, x_0, stop_pos);

% plot
figure;
title("HW5 P1: Trajectories of E and P under Direct Pursuit Guidance Law");
plot(x(:, 1), x(:, 2), "LineWidth", 2, "DisplayName", "E (Target)");
hold on;
plot(x(:, 3), x(:, 4), "LineWidth", 2, "DisplayName", "P (Missile)");
xlabel("x (m)");
ylabel("y (m)");
grid on;
legend("Location", "best")

% Direct pursuit guidance law for missile-target system (theta = beta)
function xdot = dp_law(t, x, v_T, v_M, theta_T)
% x_T = x(1);
% y_T = x(2);
% x_M = x(3);
% y_M = x(4);
R = x(5);
beta = x(6);
xdot = [
	0;
	v_T;
	v_M*cos(beta);
	v_M*sin(beta);
	v_T*cos(beta-theta_T) - v_M;
	-v_T*sin(beta-theta_T)/R;
	];
end

% stop condition to stop when the missile reaches the final position
function [value, isterminal, direction] = stop_x_M(t, x, fin_pos)
value = x(3) - fin_pos; % x(3) is x_M
isterminal = 1; % stop the integration
direction = 0;
end