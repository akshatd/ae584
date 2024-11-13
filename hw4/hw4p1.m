clc; clear; close all;

% setup given values without z
r_Pw = [0.7212; 2.4080];
r_L1w = [0; 0];
r_L2w = [5; 5];
r_L3w = [2.5; 0];

plot_init_state(r_Pw, r_L1w, r_L2w, r_L3w);
r1 = norm(r_L1w - r_Pw);
r2 = norm(r_L2w - r_Pw);
r3 = norm(r_L3w - r_Pw);

% setup jacobian for C
syms x y;
X = [x; y];
f = Gx(X, r_L1w, r_L2w, r_L3w);
C = jacobian(f, X);

% setup simulation
ks = 50;
p0s = [0.01, 0.1, 1, 10];

x0 = r_Pw;
xhat0 = [4;4];

data.xhat = zeros(2, ks+1, length(p0s));
data.Pnorm = zeros(ks+1, length(p0s));

for i = 1:length(p0s)
	P0 = p0s(i) * eye(2);
	x_k = x0;
	xhat_k = xhat0;
	P_k = P0;
	data.xhat(:, 1, i) = xhat_k;
	data.Pnorm(1, i) = norm(P_k, "fro");
	for k = 1:ks
		% predict
		xhat_kp1 = ekf_predict(xhat_k, P_k);
		
		% propagate real system
		x_kp1 = x_k;
		% measure wtr true pos
		y_kp1 = measure_noisy(x_kp1, r_L1w, r_L2w, r_L3w);
		
		% update
		C_kp1 = vpa(subs(C, X, xhat_kp1)); % get C wrt xhat
		[xhat_kp1, P_k] = ekf_update(xhat_k, P_k, y_kp1, C_kp1, r_L1w, r_L2w, r_L3w);
		
		% store data
		data.xhat(:, k+1, i) = xhat_kp1;
		data.Pnorm(k+1, i) = norm(P_k, "fro");
		
		% update vars for next iteration
		x_k = x_kp1;
		xhat_k = xhat_kp1;
	end
end

%% plot
plot_xhat(data, p0s, r_Pw);
plot_Pnorm(data, p0s);

function y = Gx(x, L1, L2, L3)
y = [
	norm(x - L1);
	norm(x - L2);
	norm(x - L3);
	];
end

function Yk = measure_noisy(x, L1, L2, L3)
D = diag([0.1, 0.1, 0.1]);
wk = mvnrnd([0; 0; 0], eye(3))';
Yk = Gx(x, L1, L2, L3) + D*wk;
end

function [xhat_p, P_p] = ekf_predict(xhat, P)
xhat_p = xhat;
P_p = P;
end

function [xhat_u, P_u] = ekf_update(xhat_p, P_p, Yk, Ck, L1, L2, L3)
R = 0.1*eye(3);
K_k = P_p * Ck' * inv(Ck * P_p * Ck' + R);
xhat_u = xhat_p + K_k * (Yk - Gx(xhat_p, L1, L2, L3));
P_u = P_p - K_k * Ck * P_p;
end

function plot_init_state(r_Pw, r_L1w, r_L2w, r_L3w)
figure;
hold on;
plot(r_Pw(1), r_Pw(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', 'P');
plot(r_L1w(1), r_L1w(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', 'L1');
plot(r_L2w(1), r_L2w(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', 'L2');
plot(r_L3w(1), r_L3w(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', 'L3');
xlabel('x');
ylabel('y');
title('Initial State');
legend boxoff; grid on;
axis equal;
end

function plot_xhat(data, p0s, r_Pw)
figure;
hold on;
for i = 1:length(p0s)
	plot(data.xhat(1, :, i), data.xhat(2, :, i), 'DisplayName', sprintf('p0 = %.2f', p0s(i)), 'LineWidth', 1);
end
plot(r_Pw(1), r_Pw(2), 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'r', 'DisplayName', 'P');

title('HW4 P1b: 2D trajectory of $\hat{X}_k$', 'Interpreter', 'latex', 'FontSize', 16);
xlabel("$\hat{x}$", 'Interpreter', 'latex', 'FontSize', 14);
ylabel("$\hat{y}$", 'Interpreter', 'latex', 'FontSize', 14);
legend; grid on;
axis equal;
end

function plot_Pnorm(data, p0s)
figure;
for i = 1:size(data.Pnorm, 2)
	semilogy(0:size(data.Pnorm, 1)-1, data.Pnorm(:, i), 'DisplayName', sprintf('p0 = %.2f', p0s(i)), 'LineWidth', 1);
    hold on;
end
hold off;

title('HW4 P1b: Frobenius norm of $P_{k|k}$', 'Interpreter', 'latex', 'FontSize', 16);
xlabel('k');
ylabel('$||P_{k|k}||_F$', 'Interpreter', 'latex', 'FontSize', 14);
legend; grid on;
end