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
beta_0 = atan2(y_T_0-y_M_0, x_T_0-x_M_0); % rad, pointing east to target
x_0 = [x_T_0; y_T_0; x_M_0; y_M_0; R_0; beta_0]; % x_T, y_T, x_M, y_M, R, beta, theta

%% part 1: direct pursuit guidance law

% set up simulation
ts = 0.01;
tspan = 0:ts:100;
stop_pos = odeset("Events", @(t, x) stop_x_R0(t, x), "RelTol", 1e-6, "AbsTol", 1e-6);

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
[t_dp_6, x_dp_6] = ode45(@(t, x) dp_law(t, x, v_T, 6, theta_T), tspan, x_0, stop_pos);
[t_dp_8, x_dp_8] = ode45(@(t, x) dp_law(t, x, v_T, 8, theta_T), tspan, x_0, stop_pos);
[t_dp_11, x_dp_11] = ode45(@(t, x) dp_law(t, x, v_T, 11, theta_T), tspan, x_0, stop_pos);
% plot the target with maximum steps
figure;
plot(x_dp_6(:, 1), x_dp_6(:, 2), "LineWidth", 2, "DisplayName", "E (Target)");
hold on;
plot(x_dp_6(:, 3), x_dp_6(:, 4), "LineWidth", 2, "DisplayName", "$V_p=6$");
plot(x_dp_8(:, 3), x_dp_8(:, 4), "LineWidth", 2, "DisplayName", "$V_p=8$");
plot(x_dp_11(:, 3), x_dp_11(:, 4), "LineWidth", 2, "DisplayName", "$V_p=11$");
xlabel("x (m)");
ylabel("y (m)");
title("HW5 P1: Trajectories of E and P under DP with different $V_p$", "Interpreter", "latex");
grid on;
legend("Location", "best", "Interpreter", "latex");

% plot beta_dot (-2 to ignore last value)
beta_dot_6 = zeros(length(t_dp_6)-1, 1);
for i = 1:length(t_dp_6)-1
	beta_dot_6(i) = beta_dot_dp(x_dp_6(i, :), v_T, theta_T);
end
beta_dot_8 = zeros(length(t_dp_8)-1, 1);
for i = 1:length(t_dp_8)-1
	beta_dot_8(i) = beta_dot_dp(x_dp_8(i, :), v_T, theta_T);
end
beta_dot_11 = zeros(length(t_dp_11)-1, 1);
for i = 1:length(t_dp_11)-1
	beta_dot_11(i) = beta_dot_dp(x_dp_11(i, :), v_T, theta_T);
end
figure;
plot(t_dp_6(1:end-1), abs(beta_dot_6), "LineWidth", 2, "DisplayName", "$V_p=6$");
hold on;
plot(t_dp_8(1:end-1), abs(beta_dot_8), "LineWidth", 2, "DisplayName", "$V_p=8$");
plot(t_dp_11(1:end-1), abs(beta_dot_11), "LineWidth", 2, "DisplayName", "$V_p=11$");
xlabel("t (s)");
ylabel("$|\dot{\beta}|$ (rad/s)", "Interpreter", "latex", "FontSize", 14);
title("HW5 P1: $|\dot{\beta}|$ versus Time under DP with different $V_p$", "Interpreter", "latex");
grid on;
legend("Location", "best", "Interpreter", "latex");

% get beta_ddot and plot it (-2 to ignore last values)
beta_ddot_6 = zeros(length(t_dp_6)-2, 1);
for i = 1:length(t_dp_6)-2
	beta_ddot_6(i) = beta_ddot_dp(x_dp_6(i, :), v_T, 6, theta_T);
end
beta_ddot_8 = zeros(length(t_dp_8)-2, 1);
for i = 1:length(t_dp_8)-2
	beta_ddot_8(i) = beta_ddot_dp(x_dp_8(i, :), v_T, 8, theta_T);
end
beta_ddot_11 = zeros(length(t_dp_11)-2, 1);
for i = 1:length(t_dp_11)-2
	beta_ddot_11(i) = beta_ddot_dp(x_dp_11(i, :), v_T, 11, theta_T);
end
figure;
plot(t_dp_6(1:end-2), abs(beta_ddot_6), "LineWidth", 2, "DisplayName", "$V_p=6$");
hold on;
plot(t_dp_8(1:end-2), abs(beta_ddot_8), "LineWidth", 2, "DisplayName", "$V_p=8$");
plot(t_dp_11(1:end-2), abs(beta_ddot_11), "LineWidth", 2, "DisplayName", "$V_p=11$");
xlabel("t (s)");
ylabel("$|\ddot{\beta}|$ (rad/s$^2$)", "Interpreter", "latex", "FontSize", 14);
title("HW5 P1: $|\ddot{\beta}|$ versus Time under DP with different $V_p$", "Interpreter", "latex");
grid on;
legend("Location", "best", "Interpreter", "latex");

%% part 2: constant bearing pursuit guidance law
[t_cbp_6, x_cbp_6] = ode45(@(t, x) cbp_law(t, x, v_T, 6, theta_T), tspan, x_0, stop_pos);
[t_cbp_8, x_cbp_8] = ode45(@(t, x) cbp_law(t, x, v_T, 8, theta_T), tspan, x_0, stop_pos);
[t_cbp_11, x_cbp_11] = ode45(@(t, x) cbp_law(t, x, v_T, 11, theta_T), tspan, x_0, stop_pos);

% plot missile and target in subplots with comparison
fig = figure;
sgtitle("HW5 P2: Trajectories of E and P under DP and CBP");

subplot(3, 1, 1);
plot(x_dp_6(:, 1), x_dp_6(:, 2), "LineWidth", 2, "DisplayName", "E (Target)"); % max E
hold on;
plot(x_dp_6(:, 3), x_dp_6(:, 4), "LineWidth", 2, "DisplayName", "P (Missile) DP");
plot(x_cbp_6(:, 3), x_cbp_6(:, 4), "LineWidth", 2, "DisplayName", "P (Missile) CBP");
xlabel("x (m)");
ylabel("y (m)");
title("$V_p=6$", "Interpreter", "latex");
grid on;
legend("Location", "best", "Interpreter", "latex");

subplot(3, 1, 2);
plot(x_dp_6(:, 1), x_dp_6(:, 2), "LineWidth", 2, "DisplayName", "E (Target)"); % max E
hold on;
plot(x_dp_8(:, 3), x_dp_8(:, 4), "LineWidth", 2, "DisplayName", "P (Missile) DP");
plot(x_cbp_8(:, 3), x_cbp_8(:, 4), "LineWidth", 2, "DisplayName", "P (Missile) CBP");
xlabel("x (m)");
ylabel("y (m)");
title("$V_p=8$", "Interpreter", "latex");
grid on;
legend("Location", "best", "Interpreter", "latex");

subplot(3, 1, 3);
plot(x_dp_6(:, 1), x_dp_6(:, 2), "LineWidth", 2, "DisplayName", "E (Target)"); % max E
hold on;
plot(x_dp_11(:, 3), x_dp_11(:, 4), "LineWidth", 2, "DisplayName", "P (Missile) DP");
plot(x_cbp_11(:, 3), x_cbp_11(:, 4), "LineWidth", 2, "DisplayName", "P (Missile) CBP");
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
plot(t_dp_6, x_dp_6(:, 5), "LineWidth", 2, "DisplayName", "DP");
hold on;
plot(t_cbp_6, x_cbp_6(:, 5), "LineWidth", 2, "DisplayName", "CBP");
xlabel("t (s)");
ylabel("R (m)");
title("$V_p=6$", "Interpreter", "latex");
grid on;
legend("Location", "best", "Interpreter", "latex");
ylim([0, 100]);

subplot(3, 1, 2);
plot(t_dp_8, x_dp_8(:, 5), "LineWidth", 2, "DisplayName", "DP");
hold on;
plot(t_cbp_8, x_cbp_8(:, 5), "LineWidth", 2, "DisplayName", "CBP");
xlabel("t (s)");
ylabel("R (m)");
title("$V_p=8$", "Interpreter", "latex");
grid on;
legend("Location", "best", "Interpreter", "latex");
ylim([0, 100]);

subplot(3, 1, 3);
plot(t_dp_11, x_dp_11(:, 5), "LineWidth", 2, "DisplayName", "DP");
hold on;
plot(t_cbp_11, x_cbp_11(:, 5), "LineWidth", 2, "DisplayName", "CBP");
xlabel("t (s)");
ylabel("R (m)");
title("$V_p=11$", "Interpreter", "latex");
grid on;
legend("Location", "best", "Interpreter", "latex");
ylim([0, 100]);

fig.Position(4) = 1000;

%% part 3: proportional pursuit law
theta_0 = beta_0;
lambdas = [0.25, 0.5, 0.75, 0.9, 1, 2, 5, 50];
data_pp = {};
for i = 1:length(lambdas)
	[t, x] = ode45(@(t, x) pp_law(t, x, v_T, v_M, theta_T, lambdas(i), theta_0), tspan, x_0, stop_pos);
	data_pp{i} = {t, x};
end

% plot trajectories for pp for lambda<1
figure;
[t, x] = data_pp{1}{:};
plot(x(:, 1), x(:, 2), "LineWidth", 2, "DisplayName", "E (Target)"); % max E
hold on;
for i=1:4
	[t, x] = data_pp{i}{:};
	plot(x(:, 3), x(:, 4), "LineWidth", 2, "DisplayName", sprintf("$\\lambda=%.2f$", lambdas(i)));
end
xlabel("x (m)");
ylabel("y (m)");
title("HW5 P3: Trajectories of E and P under PP with $\lambda<1$", "Interpreter", "latex");
grid on;
legend("Location", "best", "Interpreter", "latex");

% plot R for pp for lambda<1
figure;
for i=1:4
	[t, x] = data_pp{i}{:};
	plot(t, x(:, 5), "LineWidth", 2, "DisplayName", sprintf("$\\lambda=%.2f$", lambdas(i)));
	hold on;
end
xlabel("t (s)");
ylabel("R (m)");
title("HW5 P3: R versus Time under PP with $\lambda<1$", "Interpreter", "latex");
grid on;
legend("Location", "best", "Interpreter", "latex");

% plot trajectories for pp for lambda>=1
figure;
[t, x] = data_pp{5}{:};
plot(x(:, 1), x(:, 2), "LineWidth", 2, "DisplayName", "E (Target)"); % max E
hold on;
for i=5:length(lambdas)
	[t, x] = data_pp{i}{:};
	plot(x(:, 3), x(:, 4), "LineWidth", 2, "DisplayName", sprintf("P (Missile) PP $\\lambda=%.2f$", lambdas(i)));
end
plot(x_cbp_6(:, 3), x_cbp_6(:, 4), "--", "LineWidth", 2, "DisplayName", "P (Missile) CBP");
xlabel("x (m)");
ylabel("y (m)");
title("HW5 P3: Trajectories of E and P under PP with $\lambda\geq1$", "Interpreter", "latex");
grid on;
legend("Location", "best", "Interpreter", "latex");

% plot R for pp for lambda>=1
figure;
for i=5:length(lambdas)
	[t, x] = data_pp{i}{:};
	plot(t, x(:, 5), "LineWidth", 2, "DisplayName", sprintf("PP $\\lambda=%.2f$", lambdas(i)));
	hold on;
end
plot(t_cbp_6, x_cbp_6(:, 5), "--", "LineWidth", 2, "DisplayName", "CBP");
xlabel("t (s)");
ylabel("R (m)");
title("HW5 P3: R versus Time under PP with $\lambda\geq1$", "Interpreter", "latex");
grid on;
legend("Location", "best", "Interpreter", "latex");

%% part 4: DP and PP with noise
stop_pos_r05 = odeset("Events", @(t, x) stop_x_R05(t, x), "RelTol", 1e-6, "AbsTol", 1e-6);
% noisy dp
[t_6_dp_noise, x_6_dp_noise] = ode_noisy(@(t, x, noise) dp_law_noisy(t, x, v_T, v_M, theta_T, noise), tspan, x_0, stop_pos_r05, ts);
% noisy pp
theta_0 = beta_0;
lambdas = [1.5, 2, 2.5];
data_pp_noise = {};
for i = 1:length(lambdas)
	[t, x] = ode_noisy(@(t, x, noise) pp_law_noisy(t, x, v_T, v_M, theta_T, lambdas(i), theta_0, noise), tspan, x_0, stop_pos_r05, ts);
	data_pp_noise{i} = {t, x};
end

% plot trajectories for dp and pp with noise
figure;
[t, x] = data_pp_noise{end}{:};
plot(x(:, 1), x(:, 2), "LineWidth", 2, "DisplayName", "E (Target)"); % max E
hold on;
plot(x_6_dp_noise(:, 3), x_6_dp_noise(:, 4), "LineWidth", 2, "DisplayName", "P (Missile) DP");
for i=1:length(lambdas)
	[t, x] = data_pp_noise{i}{:};
	plot(x(:, 3), x(:, 4), "LineWidth", 2, "DisplayName", sprintf("P (Missile) PP $\\lambda=%.2f$", lambdas(i)));
end
xlabel("x (m)");
ylabel("y (m)");
title("HW5 P4: Trajectories of E and P under DP and PP with noise", "Interpreter", "latex");
grid on;
legend("Location", "best", "Interpreter", "latex");
ylim([0, 510]);
% plot R for dp and pp with noise
figure;
plot(t_6_dp_noise, x_6_dp_noise(:, 5), "LineWidth", 2, "DisplayName", "DP");
hold on;
for i=1:length(lambdas)
	[t, x] = data_pp_noise{i}{:};
	plot(t, x(:, 5), "LineWidth", 2, "DisplayName", sprintf("PP $\\lambda=%.2f$", lambdas(i)));
end
xlabel("t (s)");
ylabel("R (m)");
title("HW5 P4: R versus Time under DP and PP with noise", "Interpreter", "latex");
grid on;
legend("Location", "best", "Interpreter", "latex");

%% part 5: CBP and PP with maneuvering target
% get cbp
[t_cbp_6_man, x_cbp_6_man] = ode45(@(t, x) cbp_law_man(t, x, v_T, 6), tspan, x_0, stop_pos);
% get pp
theta_0 = beta_0;
lambdas = [0.25, 0.5, 0.75, 0.9, 1, 2, 5, 50];
data_pp_man = {};
for i = 1:length(lambdas)
	[t, x] = ode45(@(t, x) pp_law_man(t, x, v_T, v_M, lambdas(i), theta_0), tspan, x_0, stop_pos);
	data_pp_man{i} = {t, x};
end

% plot traj for lambda < 1
figure
[t, x] = data_pp_man{1}{:};
plot(x(:, 1), x(:, 2), "LineWidth", 2, "DisplayName", "E (Target)"); % max E
hold on;
for i=1:4
	[t, x] = data_pp_man{i}{:};
	plot(x(:, 3), x(:, 4), "LineWidth", 2, "DisplayName", sprintf("P (Missile) PP $\\lambda=%.2f$", lambdas(i)));
end
xlabel("x (m)");
ylabel("y (m)");
title("HW5 P5: Trajectories of E and P under PP with $\lambda<1$ and maneuvering E", "Interpreter", "latex");
grid on;
legend("Location", "best", "Interpreter", "latex");

% plot R for lambda < 1
figure;
for i=1:4
	[t, x] = data_pp_man{i}{:};
	plot(t, x(:, 5), "LineWidth", 2, "DisplayName", sprintf("PP $\\lambda=%.2f$", lambdas(i)));
	hold on;
end
xlabel("t (s)");
ylabel("R (m)");
title("HW5 P5: R versus Time under PP with $\lambda<1$ and maneuvering E", "Interpreter", "latex");
grid on;
legend("Location", "best", "Interpreter", "latex");

% plot traj for lambda >= 1
figure;
[t, x] = data_pp_man{5}{:};
plot(x(:, 1), x(:, 2), "LineWidth", 2, "DisplayName", "E (Target)"); % max E
hold on;
for i=5:length(lambdas)
	[t, x] = data_pp_man{i}{:};
	plot(x(:, 3), x(:, 4), "LineWidth", 2, "DisplayName", sprintf("P (Missile) PP $\\lambda=%.2f$", lambdas(i)));
end
plot(x_cbp_6_man(:, 3), x_cbp_6_man(:, 4), "--", "LineWidth", 2, "DisplayName", "P (Missile) CBP");
xlabel("x (m)");
ylabel("y (m)");
title("HW5 P5: Trajectories of E and P under PP with $\lambda\geq1$ and maneuvering E", "Interpreter", "latex");
grid on;
legend("Location", "best", "Interpreter", "latex");

% plot R for lambda >= 1
figure;
for i=5:length(lambdas)
	[t, x] = data_pp_man{i}{:};
	plot(t, x(:, 5), "LineWidth", 2, "DisplayName", sprintf("PP $\\lambda=%.2f$", lambdas(i)));
	hold on;
end
plot(t_cbp_6_man, x_cbp_6_man(:, 5), "--", "LineWidth", 2, "DisplayName", "CBP");
xlabel("t (s)");
ylabel("R (m)");
title("HW5 P5: R versus Time under PP with $\lambda\geq1$ and maneuvering E", "Interpreter", "latex");
grid on;
legend("Location", "best", "Interpreter", "latex");

%% part 6: DP and PP with noise and maneuvering target
% noisy dp
[t_6_dp_noise_man, x_6_dp_noise_man] = ode_noisy(@(t, x, noise) dp_law_noisy_man(t, x, v_T, v_M, noise), tspan, x_0, stop_pos_r05, ts);
% noisy pp
theta_0 = beta_0;
lambdas = [1.5, 2, 2.5];
data_pp_noise_man = {};
for i = 1:length(lambdas)
	[t, x] = ode_noisy(@(t, x, noise) pp_law_noisy_man(t, x, v_T, v_M, lambdas(i), theta_0, noise), tspan, x_0, stop_pos_r05, ts);
	data_pp_noise_man{i} = {t, x};
end

%plot trajectories for dp and pp with noise and maneuvering target
figure;
[t, x] = data_pp_noise_man{end}{:};
plot(x(:, 1), x(:, 2), "LineWidth", 2, "DisplayName", "E (Target)"); % max E
hold on;
plot(x_6_dp_noise_man(:, 3), x_6_dp_noise_man(:, 4), "LineWidth", 2, "DisplayName", "P (Missile) DP");
for i=1:length(lambdas)
	[t, x] = data_pp_noise_man{i}{:};
	plot(x(:, 3), x(:, 4), "LineWidth", 2, "DisplayName", sprintf("P (Missile) PP $\\lambda=%.2f$", lambdas(i)));
end
xlabel("x (m)");
ylabel("y (m)");
title("HW5 P6: Trajectories of E and P under DP and PP with noise and maneuvering E", "Interpreter", "latex");
grid on;
legend("Location", "best", "Interpreter", "latex");
% plot R for dp and pp with noise and maneuvering target
figure;
plot(t_6_dp_noise_man, x_6_dp_noise_man(:, 5), "LineWidth", 2, "DisplayName", "DP");
hold on;
for i=1:length(lambdas)
	[t, x] = data_pp_noise_man{i}{:};
	plot(t, x(:, 5), "LineWidth", 2, "DisplayName", sprintf("PP $\\lambda=%.2f$", lambdas(i)));
end
xlabel("t (s)");
ylabel("R (m)");
title("HW5 P6: R versus Time under DP and PP with noise and maneuvering E", "Interpreter", "latex");
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
beta_dot = -(v_T*sin(beta-theta_T) - v_M*sin(beta-theta)) / R;
x_dot = [
	0;
	v_T;
	v_M*cos(theta);
	v_M*sin(theta);
	v_T*cos(beta-theta_T) - v_M*cos(beta-theta);
	beta_dot;
	];
end

% stop condition to stop when the missile reaches the final position
function [value, isterminal, direction] = stop_x_R0(t, x)
value = x(5); % R
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

function theta = theta_cbp(v_T, v_M, theta_T, beta)
theta = beta-asin(v_T/v_M * sin(beta-theta_T));
end

% Constant bearing pursuit guidance law (beta_dot = 0)
function x_dot = cbp_law(t, x, v_T, v_M, theta_T)
% x_T = x(1);
% y_T = x(2);
% x_M = x(3);
% y_M = x(4);
% R = x(5);
beta = x(6);
theta = theta_cbp(v_T, v_M, theta_T, beta); % cbp law
x_dot = [
	0;
	v_T;
	v_M*cos(theta);
	v_M*sin(theta);
	v_T*cos(beta-theta_T) - v_M*cos(beta-theta);
	0; % cbp law, beta_dot = 0
	];
end

% proportional pursuit guidance law (theta = lambda*beta + theta_0)
function x_dot = pp_law(t, x, v_T, v_M, theta_T, lambda, theta_0)
% x_T = x(1);
% y_T = x(2);
% x_M = x(3);
% y_M = x(4);
R = x(5);
beta = x(6);
theta = lambda * beta + theta_0; % proportional pursuit law
beta_dot = -(v_T*sin(beta-theta_T) - v_M*sin(beta-theta)) / R;
x_dot = [
	0;
	v_T;
	v_M*cos(theta);
	v_M*sin(theta);
	v_T*cos(beta-theta_T) - v_M*cos(beta-theta);
	beta_dot;
	];
end

% ode function with noisy steps
function [t, x] = ode_noisy(odefun, tspan, x_0, options, ts)
x = zeros(length(tspan), length(x_0));
x(1,:) = x_0;
t = tspan;
idx_end = 1;

for i = 2:length(t)
	noise = 0.25*randn(1);
	[t_ode, x_ode, ~, ~, ie] = ode45(@(t,x)odefun(t,x,noise), [i-2 i-1]*ts, x(i-1, :), options);
	idx_end = i;
	x(i,:) = x_ode(end,:);
	if (~isempty(ie))
		t(i) = t_ode(end); % current timestep is whenever the event happened
		break;
	end
end
x = x(1:idx_end, :);
t = t(1:idx_end);
end

function [value, isterminal, direction] = stop_x_R05(t, x)
value = x(5) - 0.5; % R
isterminal = 1; % stop the integration
direction = 0;
end


% Direct pursuit guidance law (theta = beta) with noise
function x_dot = dp_law_noisy(t, x, v_T, v_M, theta_T, noise)
% x_T = x(1);
% y_T = x(2);
% x_M = x(3);
% y_M = x(4);
R = x(5);
beta = x(6);
theta = beta + noise; % guidance law for direct pursuit, measurement noise in theta
beta_dot = -(v_T*sin(beta-theta_T) - v_M*sin(beta-theta)) / R;
% state evolution is based on actual beta, not noisy beta
x_dot = [
	0;
	v_T;
	v_M*cos(theta);
	v_M*sin(theta);
	v_T*cos(beta-theta_T) - v_M*cos(beta-theta);
	beta_dot;
	];
end

% proportional pursuit guidance law (theta = lambda*beta + theta_0) with noise
function x_dot = pp_law_noisy(t, x, v_T, v_M, theta_T, lambda, theta_0, noise)
% x_T = x(1);
% y_T = x(2);
% x_M = x(3);
% y_M = x(4);
R = x(5);
beta = x(6);
theta = lambda * (beta + noise) + theta_0; % proportional pursuit law, measurement noise in theta
beta_dot = -(v_T*sin(beta-theta_T) - v_M*sin(beta-theta)) / R;
% state evolution is based on actual beta, not noisy beta
x_dot = [
	0;
	v_T;
	v_M*cos(theta);
	v_M*sin(theta);
	v_T*cos(beta-theta_T) - v_M*cos(beta-theta);
	beta_dot;
	];
end

% Constant bearing pursuit guidance law (beta_dot = 0) with maneuvering target
function x_dot = cbp_law_man(t, x, v_T, v_M)
% x_T = x(1);
% y_T = x(2);
% x_M = x(3);
% y_M = x(4);
% R = x(5);
beta = x(6);
theta_T = pi/2 + cos(t); % maneuvering target
theta = theta_cbp(v_T, v_M, theta_T, beta); % cbp law
x_dot = [
	v_T * cos(theta_T);
	v_T * sin(theta_T);
	v_M*cos(theta);
	v_M*sin(theta);
	v_T*cos(beta-theta_T) - v_M*cos(beta-theta);
	0; % cbp law, beta_dot = 0
	];
end

% proportional pursuit guidance law (theta = lambda*beta + theta_0) with maneuvering target
function x_dot = pp_law_man(t, x, v_T, v_M, lambda, theta_0)
% x_T = x(1);
% y_T = x(2);
% x_M = x(3);
% y_M = x(4);
R = x(5);
beta = x(6);
theta_T = pi/2 + cos(t); % maneuvering target
theta = lambda * beta + theta_0; % proportional pursuit law
beta_dot = -(v_T*sin(beta-theta_T) - v_M*sin(beta-theta)) / R;
x_dot = [
	v_T * cos(theta_T);
	v_T * sin(theta_T);
	v_M*cos(theta);
	v_M*sin(theta);
	v_T*cos(beta-theta_T) - v_M*cos(beta-theta);
	beta_dot;
	];
end

% Direct pursuit guidance law (theta = beta) with noise
function x_dot = dp_law_noisy_man(t, x, v_T, v_M, noise)
% x_T = x(1);
% y_T = x(2);
% x_M = x(3);
% y_M = x(4);
R = x(5);
beta = x(6);
theta_T = pi/2 + cos(t); % maneuvering target
theta = beta + noise; % guidance law for direct pursuit, measurement noise in theta
beta_dot = -(v_T*sin(beta-theta_T) - v_M*sin(beta-theta)) / R;
% state evolution is based on actual beta, not noisy beta
x_dot = [
	v_T * cos(theta_T);
	v_T * sin(theta_T);
	v_M*cos(theta);
	v_M*sin(theta);
	v_T*cos(beta-theta_T) - v_M*cos(beta-theta);
	beta_dot;
	];
end

% proportional pursuit guidance law (theta = lambda*beta + theta_0) with noise
function x_dot = pp_law_noisy_man(t, x, v_T, v_M, lambda, theta_0, noise)
% x_T = x(1);
% y_T = x(2);
% x_M = x(3);
% y_M = x(4);
R = x(5);
beta = x(6);
theta_T = pi/2 + cos(t); % maneuvering target
theta = lambda * (beta + noise) + theta_0; % proportional pursuit law, measurement noise in theta
beta_dot = -(v_T*sin(beta-theta_T) - v_M*sin(beta-theta)) / R;
% state evolution is based on actual beta, not noisy beta
x_dot = [
	v_T * cos(theta_T);
	v_T * sin(theta_T);
	v_M*cos(theta);
	v_M*sin(theta);
	v_T*cos(beta-theta_T) - v_M*cos(beta-theta);
	beta_dot;
	];
end
