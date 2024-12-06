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

% part 1: direct pursuit guidance law

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
[t_6_dp, x_6_dp] = ode45(@(t, x) dp_law(t, x, v_T, 6, theta_T), tspan, x_0, stop_pos);
[t_8_dp, x_8_dp] = ode45(@(t, x) dp_law(t, x, v_T, 8, theta_T), tspan, x_0, stop_pos);
[t_11_dp, x_11_dp] = ode45(@(t, x) dp_law(t, x, v_T, 11, theta_T), tspan, x_0, stop_pos);
% plot the target with maximum steps
figure;
plot(x_6_dp(:, 1), x_6_dp(:, 2), "LineWidth", 2, "DisplayName", "E (Target)");
hold on;
plot(x_6_dp(:, 3), x_6_dp(:, 4), "LineWidth", 2, "DisplayName", "$V_p=6$");
plot(x_8_dp(:, 3), x_8_dp(:, 4), "LineWidth", 2, "DisplayName", "$V_p=8$");
plot(x_11_dp(:, 3), x_11_dp(:, 4), "LineWidth", 2, "DisplayName", "$V_p=11$");
xlabel("x (m)");
ylabel("y (m)");
title("HW5 P1: Trajectories of E and P under DP with different $V_p$", "Interpreter", "latex");
grid on;
legend("Location", "best", "Interpreter", "latex");

% plot beta_dot
beta_dot_6 = zeros(length(t_6_dp), 1);
for i = 1:length(t_6_dp)
	beta_dot_6(i) = beta_dot_dp(x_6_dp(i, :), v_T, theta_T);
end
beta_dot_8 = zeros(length(t_8_dp), 1);
for i = 1:length(t_8_dp)
	beta_dot_8(i) = beta_dot_dp(x_8_dp(i, :), v_T, theta_T);
end
beta_dot_11 = zeros(length(t_11_dp), 1);
for i = 1:length(t_11_dp)
	beta_dot_11(i) = beta_dot_dp(x_11_dp(i, :), v_T, theta_T);
end
figure;
plot(t_6_dp, abs(beta_dot_6), "LineWidth", 2, "DisplayName", "$V_p=6$");
hold on;
plot(t_8_dp, abs(beta_dot_8), "LineWidth", 2, "DisplayName", "$V_p=8$");
plot(t_11_dp, abs(beta_dot_11), "LineWidth", 2, "DisplayName", "$V_p=11$");
xlabel("t (s)");
ylabel("$|\dot{\beta}|$ (rad/s)", "Interpreter", "latex", "FontSize", 14);
title("HW5 P1: $|\dot{\beta}|$ versus Time under DP with different $V_p$", "Interpreter", "latex");
grid on;
legend("Location", "best", "Interpreter", "latex");

% get beta_ddot and plot it
beta_ddot_6 = zeros(length(t_6_dp), 1);
for i = 1:length(t_6_dp)
	beta_ddot_6(i) = beta_ddot_dp(x_6_dp(i, :), v_T, 6, theta_T);
end
beta_ddot_8 = zeros(length(t_8_dp), 1);
for i = 1:length(t_8_dp)
	beta_ddot_8(i) = beta_ddot_dp(x_8_dp(i, :), v_T, 8, theta_T);
end
beta_ddot_11 = zeros(length(t_11_dp)-1, 1); % ignore last value
for i = 1:length(t_11_dp)-1
	beta_ddot_11(i) = beta_ddot_dp(x_11_dp(i, :), v_T, 11, theta_T);
end
figure;
plot(t_6_dp, abs(beta_ddot_6), "LineWidth", 2, "DisplayName", "$V_p=6$");
hold on;
plot(t_8_dp, abs(beta_ddot_8), "LineWidth", 2, "DisplayName", "$V_p=8$");
plot(t_11_dp(1:end-1), abs(beta_ddot_11), "LineWidth", 2, "DisplayName", "$V_p=11$");
xlabel("t (s)");
ylabel("$|\ddot{\beta}|$ (rad/s$^2$)", "Interpreter", "latex", "FontSize", 14);
title("HW5 P1: $|\ddot{\beta}|$ versus Time under DP with different $V_p$", "Interpreter", "latex");
grid on;
legend("Location", "best", "Interpreter", "latex");

% part 2: constant bearing pursuit guidance law
theta_0 = theta_cbp(v_T, 6, theta_T);
x_0 = [x_T_0; y_T_0; x_M_0; y_M_0; R_0; beta_0; theta_0]; % x_T, y_T, x_M, y_M, R, beta, theta
[t_6_cbp, x_6_cbp] = ode45(@(t, x) cbp_law(t, x, v_T, 6, theta_T), tspan, x_0, stop_pos);
theta_0 = theta_cbp(v_T, 8, theta_T);
x_0 = [x_T_0; y_T_0; x_M_0; y_M_0; R_0; beta_0; theta_0]; % x_T, y_T, x_M, y_M, R, beta, theta
[t_8_cbp, x_8_cbp] = ode45(@(t, x) cbp_law(t, x, v_T, 8, theta_T), tspan, x_0, stop_pos);
theta_0 = theta_cbp(v_T, 11, theta_T);
x_0 = [x_T_0; y_T_0; x_M_0; y_M_0; R_0; beta_0; theta_0]; % x_T, y_T, x_M, y_M, R, beta, theta
[t_11_cbp, x_11_cbp] = ode45(@(t, x) cbp_law(t, x, v_T, 11, theta_T), tspan, x_0, stop_pos);

% plot missile and target in subplots with comparison
fig = figure;
sgtitle("HW5 P2: Trajectories of E and P under DP and CBP");

subplot(3, 1, 1);
plot(x_6_dp(:, 1), x_6_dp(:, 2), "LineWidth", 2, "DisplayName", "E (Target)");
hold on;
plot(x_6_dp(:, 3), x_6_dp(:, 4), "LineWidth", 2, "DisplayName", "P (Missile) DP");
plot(x_6_cbp(:, 3), x_6_cbp(:, 4), "LineWidth", 2, "DisplayName", "P (Missile) CBP");
xlabel("x (m)");
ylabel("y (m)");
title("$V_p=6$", "Interpreter", "latex");
grid on;
legend("Location", "best", "Interpreter", "latex");

subplot(3, 1, 2);
plot(x_6_dp(:, 1), x_6_dp(:, 2), "LineWidth", 2, "DisplayName", "E (Target)");
hold on;
plot(x_8_dp(:, 3), x_8_dp(:, 4), "LineWidth", 2, "DisplayName", "P (Missile) DP");
plot(x_8_cbp(:, 3), x_8_cbp(:, 4), "LineWidth", 2, "DisplayName", "P (Missile) CBP");
xlabel("x (m)");
ylabel("y (m)");
title("$V_p=8$", "Interpreter", "latex");
grid on;
legend("Location", "best", "Interpreter", "latex");

subplot(3, 1, 3);
plot(x_6_dp(:, 1), x_6_dp(:, 2), "LineWidth", 2, "DisplayName", "E (Target)");
hold on;
plot(x_11_dp(:, 3), x_11_dp(:, 4), "LineWidth", 2, "DisplayName", "P (Missile) DP");
plot(x_11_cbp(:, 3), x_11_cbp(:, 4), "LineWidth", 2, "DisplayName", "P (Missile) CBP");
xlabel("x (m)");
ylabel("y (m)");
title("$V_p=11$", "Interpreter", "latex");
grid on;
legend("Location", "best", "Interpreter", "latex");
xlim([0 ,120]);

fig.Position(4) = 1000;

% plot R comparison
fig = figure;
sgtitle("HW5 P2: R versus Time under DP and CBP");

subplot(3, 1, 1);
plot(t_6_dp, x_6_dp(:, 5), "LineWidth", 2, "DisplayName", "DP");
hold on;
plot(t_6_cbp, x_6_cbp(:, 5), "LineWidth", 2, "DisplayName", "CBP");
xlabel("t (s)");
ylabel("R (m)");
title("$V_p=6$", "Interpreter", "latex");
grid on;
legend("Location", "best", "Interpreter", "latex");

subplot(3, 1, 2);
plot(t_8_dp, x_8_dp(:, 5), "LineWidth", 2, "DisplayName", "DP");
hold on;
plot(t_8_cbp, x_8_cbp(:, 5), "LineWidth", 2, "DisplayName", "CBP");
xlabel("t (s)");
ylabel("R (m)");
title("$V_p=8$", "Interpreter", "latex");
grid on;
legend("Location", "best", "Interpreter", "latex");
ylim([0, 100]);

subplot(3, 1, 3);
plot(t_11_dp, x_11_dp(:, 5), "LineWidth", 2, "DisplayName", "DP");
hold on;
plot(t_11_cbp, x_11_cbp(:, 5), "LineWidth", 2, "DisplayName", "CBP");
xlabel("t (s)");
ylabel("R (m)");
title("$V_p=11$", "Interpreter", "latex");
grid on;
legend("Location", "best", "Interpreter", "latex");

fig.Position(4) = 1000;

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

function theta = theta_cbp(v_T, v_M, theta_T)
theta = -asin(v_T/v_M * sin(-theta_T));
end

function x_dot = cbp_law(t, x, v_T, v_M, theta_T)
% x_T = x(1);
% y_T = x(2);
% x_M = x(3);
% y_M = x(4);
% R = x(5);
beta = x(6);
theta = x(7);
x_dot = [
	0;
	v_T;
	v_M*cos(theta);
	v_M*sin(theta);
	v_T*cos(beta-theta_T) - v_M*cos(beta-theta);
	0; % cbp law, beta_dot = 0
	0; % cbp law, theta_dot = 0 in this case cos v_T is constant
	];
end