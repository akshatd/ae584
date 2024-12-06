clc; clear; close all;

% set up sim
v_T = 5; % m/s
theta_T = pi/2; % rad, pointing north

% initial conditions
v_M = 6; % m/s
x_T_0 = 100; % m
y_T_0 = 0; % m
x_M_0 = 0; % m/s
y_M_0 = 0; % m/s
R_0 = sqrt((x_T_0 - x_M_0)^2 + (y_T_0 - y_M_0)^2); % m
beta_0 = 0; % rad, pointing east to target
theta_0 = beta_0; % rad, pointing east to target
x_0 = [x_T_0; y_T_0; x_M_0; y_M_0; R_0; beta_0; theta_0]; % x_T, y_T, x_M, y_M, R, beta, theta
fin_x_M = 100; % m

% set up simulation
tspan = 0:0.01:100;
stop_pos = odeset("Events", @(t, x) stop_x_M(t, x, fin_x_M), "RelTol", 1e-6, "AbsTol", 1e-6);

% simulate
[t, x] = ode45(@(t, x) dp_law(t, x, v_T, v_M, theta_T), tspan, x_0, stop_pos);
t_c = ttc([x_T_0; y_T_0], v_T, v_M);
fprintf("Time to collision:\n- Theoretical: %.2f s\n- Actual: %2f s\n", t_c, t(end));

% plot missile and target
figure;
plot(x(:, 1), x(:, 2), "LineWidth", 2, "DisplayName", "E (Target)");
hold on;
plot(x(:, 3), x(:, 4), "LineWidth", 2, "DisplayName", "P (Missile)");
xlabel("x (m)");
ylabel("y (m)");
title("HW5 P1: Trajectories of E and P under DP");
grid on;
legend("Location", "best")

% plot R
figure;
plot(t, x(:, 5), "LineWidth", 2);
xlabel("t (s)");
ylabel("R (m)");
title("HW5 P1: R versus Time under DP");
grid on;

% multiple missile velocities
figure;
[t, x] = ode45(@(t, x) dp_law(t, x, v_T, 6, theta_T), tspan, x_0, stop_pos);
t_6 = t;
x_6 = x;
plot(x(:, 3), x(:, 4), "LineWidth", 2, "DisplayName", "$V_p=6$");
hold on;
[t, x] = ode45(@(t, x) dp_law(t, x, v_T, 8, theta_T), tspan, x_0, stop_pos);
t_8 = t;
x_8 = x;
plot(x(:, 3), x(:, 4), "LineWidth", 2, "DisplayName", "$V_p=8$");
[t, x] = ode45(@(t, x) dp_law(t, x, v_T, 11, theta_T), tspan, x_0, stop_pos);
t_11 = t;
x_11 = x;
plot(x(:, 3), x(:, 4), "LineWidth", 2, "DisplayName", "$V_p=11$");
% plot the target with maximum steps
plot(x_6(:, 1), x_6(:, 2), "LineWidth", 2, "DisplayName", "E (Target)");
xlabel("x (m)");
ylabel("y (m)");
title("HW5 P1: Trajectories of E and P under DP with different $V_p$", "Interpreter", "latex");
grid on;
legend("Location", "best", "Interpreter", "latex");

% plot beta_dot
beta_dot_6 = zeros(length(t_6), 1);
for i = 1:length(t_6)
	beta_dot_6(i) = beta_dot_dp(x_6(i, :), v_T, theta_T);
end
beta_dot_8 = zeros(length(t_8), 1);
for i = 1:length(t_8)
	beta_dot_8(i) = beta_dot_dp(x_8(i, :), v_T, theta_T);
end
beta_dot_11 = zeros(length(t_11), 1);
for i = 1:length(t_11)
	beta_dot_11(i) = beta_dot_dp(x_11(i, :), v_T, theta_T);
end
figure;
plot(t_6, abs(beta_dot_6), "LineWidth", 2, "DisplayName", "$V_p=6$");
hold on;
plot(t_8, abs(beta_dot_8), "LineWidth", 2, "DisplayName", "$V_p=8$");
plot(t_11, abs(beta_dot_11), "LineWidth", 2, "DisplayName", "$V_p=11$");
xlabel("t (s)");
ylabel("$|\dot{\beta}|$ (rad/s)", "Interpreter", "latex", "FontSize", 14);
title("HW5 P1: $|\dot{\beta}|$ versus Time under DP with different $V_p$", "Interpreter", "latex");
grid on;
legend("Location", "best", "Interpreter", "latex");

% get beta_ddot and plot it
beta_ddot_6 = zeros(length(t_6), 1);
for i = 1:length(t_6)
	beta_ddot_6(i) = beta_ddot_dp(x_6(i, :), v_T, 6, theta_T);
end
beta_ddot_8 = zeros(length(t_8), 1);
for i = 1:length(t_8)
	beta_ddot_8(i) = beta_ddot_dp(x_8(i, :), v_T, 8, theta_T);
end
beta_ddot_11 = zeros(length(t_11)-1, 1); % ignore last value
for i = 1:length(t_11)-1
	beta_ddot_11(i) = beta_ddot_dp(x_11(i, :), v_T, 11, theta_T);
end
figure;
plot(t_6, abs(beta_ddot_6), "LineWidth", 2, "DisplayName", "$V_p=6$");
hold on;
plot(t_8, abs(beta_ddot_8), "LineWidth", 2, "DisplayName", "$V_p=8$");
plot(t_11(1:end-1), abs(beta_ddot_11), "LineWidth", 2, "DisplayName", "$V_p=11$");
xlabel("t (s)");
ylabel("$|\ddot{\beta}|$ (rad/s$^2$)", "Interpreter", "latex", "FontSize", 14);
title("HW5 P1: $|\ddot{\beta}|$ versus Time under DP with different $V_p$", "Interpreter", "latex");
grid on;
legend("Location", "best", "Interpreter", "latex");

% Direct pursuit guidance law for missile-target system (theta = beta)
function x_dot = dp_law(t, x, v_T, v_M, theta_T)
% x_T = x(1);
% y_T = x(2);
% x_M = x(3);
% y_M = x(4);
R = x(5);
beta = x(6);
theta = beta; % guidance law for direct pursuit
beta_dot = -(v_T*sin(theta-theta_T) - v_M*sin(beta-theta)) / R;
theta_dot = beta_dot; % guidance law for direct pursuit
x_dot = [
	0;
	v_T;
	v_M*cos(theta);
	v_M*sin(theta);
	v_T*cos(theta-theta_T) - v_M*cos(beta-theta);
	beta_dot;
	theta_dot;
	];
end

% stop condition to stop when the missile reaches the final position
function [value, isterminal, direction] = stop_x_M(t, x, fin_pos)
value = x(3) - fin_pos; % x(3) is x_M
isterminal = 1; % stop the integration
direction = 0;
end

function t_c = ttc(T_0, v_T, v_M)
gamma = v_T / v_M;
t_c = gamma * norm(T_0) / ((1-gamma^2) * v_T);
end

% beta_dot for direct pursuit
function beta_dot = beta_dot_dp(x, v_T, theta_T)
R = x(5);
beta = x(6);
beta_dot = -(v_T*sin(beta - theta_T)) / R;
end

% beta ddot for direct pursuit
function beta_ddot = beta_ddot_dp(x, v_T, v_M, theta_T)
xdot = dp_law(0, x, v_T, v_M, theta_T);
R = x(5);
R_dot = xdot(5);
beta = x(6);
beta_dot = xdot(6);
% beta_ddot = (v_T^2 * cos(theta - theta_T) - v_M^2 * cos(beta - theta) + R_dot * v_M * sin(beta - theta) - R * v_M * beta_dot * cos(beta - theta)) / R;
beta_ddot = (-R*beta_dot*v_T*cos(beta-theta_T) + R_dot*v_T*sin(beta-theta_T)) / R^2;
end