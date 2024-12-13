% problem 1: satellite dynamics

clc; clear; close all;

% initial conditions
mu = 0.5;
xhat_0 = [2.5;0;1;0;0.5;-0.1];
P_0 = eye(6);
D1 = [
	zeros(3, 3);
	0.01*eye(3)
	];
D2 = 0.1*eye(3);
Q = D1 * eye(3) * D1';
R = D2 * eye(3) * D2';
x_ref = load("AE584_Final_P1_pos_Ts_0_01.mat");
y_ref = load("AE584_Final_P1_meas_Ts_0_01.mat");
syms x1 x2 x3 x4 x5 x6;
X = [x1; x2; x3; x4; x5; x6];
g = measure(X);
C = jacobian(g, X);

% setup simm
Ts = 0.01;
Tmeass = [1, 0.1];
ks = 10000;
xhat = zeros(6, ks, length(Tmeass));
xhat(:, 1, 1) = xhat_0;
xhat(:, 1, 2) = xhat_0;
for Tmeas=Tmeass
	xhat_k = xhat_0;
	P_k = P_0;
	for k=2:ks
		% predict
		[xhat_p, P_p] = ekf_predict(xhat_k, P_k, Q, Ts, mu);
		
		% check if theres measurement
		if mod(k, Tmeas/Ts) == 0
			% get measurement
			y_k = [y_ref.Range(k); y_ref.Azimuth(k); y_ref.Elevation(k)];
			% update
			C_kp1 = double(subs(C, X, xhat_p));
			[xhat_u, P_u] = ekf_update(xhat_p, P_p, y_k, C_kp1, R);
			xhat_k = xhat_u;
			P_k = P_u;
		else % just propagate
			xhat_k = xhat_p;
			P_k = P_p;
		end
		
		% store data
		xhat(:, k, Tmeass==Tmeas) = xhat_k;
	end
end

%% plot
figure;
plot3(xhat(1, :, 1), xhat(2, :, 1), xhat(3, :, 1), 'r', "LineWidth", 2, "DisplayName", "Tmeas=1");
hold on;
plot3(xhat(1, :, 2), xhat(2, :, 2), xhat(3, :, 2), 'b', "LineWidth", 2,"DisplayName", "Tmeas=0.1");
plot3(x_ref.Xref, x_ref.Yref, x_ref.Zref, '--g', "LineWidth", 2, "DisplayName", "True");
xlabel("X");
ylabel("Y");
zlabel("Z");
title("Final P1: Satellite Trajectory");
legend("Location", "best");
grid on;

figure;
subplot(3, 1, 1);
plot(xhat(1, :, 1), 'r', "LineWidth", 2, "DisplayName", "Tmeas=1");
hold on;
plot(xhat(1, :, 2), 'b', "LineWidth", 2, "DisplayName", "Tmeas=0.1");
plot(x_ref.Xref, '--g', "LineWidth", 2, "DisplayName", "True");
xlabel("Time");
ylabel("X");
legend("Location", "best");
grid on;

subplot(3, 1, 2);
plot(xhat(2, :, 1), 'r', "LineWidth", 2, "DisplayName", "Tmeas=1");
hold on;
plot(xhat(2, :, 2), 'b', "LineWidth", 2, "DisplayName", "Tmeas=0.1");
plot(x_ref.Yref, '--g', "LineWidth", 2, "DisplayName", "True");
xlabel("Time");
ylabel("Y");
legend("Location", "best");
grid on;

subplot(3, 1, 3);
plot(xhat(3, :, 1), 'r', "LineWidth", 2, "DisplayName", "Tmeas=1");
hold on;
plot(xhat(3, :, 2), 'b', "LineWidth", 2, "DisplayName", "Tmeas=0.1");
plot(x_ref.Zref, '--g', "LineWidth", 2, "DisplayName", "True");
xlabel("Time");
ylabel("Z");
legend("Location", "best");
grid on;

function y_k = measure(x_k)
X_k = x_k(1);
Y_k = x_k(2);
Z_k = x_k(3);
y_k = [
	sqrt(X_k^2 + Y_k^2 + Z_k^2);
	-atan2(X_k, Y_k);
	atan2(Z_k, sqrt(X_k^2 + Y_k^2));
	];
end


function y_k = measure_unwrapped(x_k)
X_k = x_k(1);
Y_k = x_k(2);
Z_k = x_k(3);
y_k = [
	sqrt(X_k^2 + Y_k^2 + Z_k^2);
	AzUnwrap(-atan2(X_k, Y_k), 0);
	atan2(Z_k, sqrt(X_k^2 + Y_k^2));
	];
end

function A_k = calc_Ak(x_k, mu, Ts)
X_k = x_k(1);
Y_k = x_k(2);
Z_k = x_k(3);
r = sqrt(X_k^2 + Y_k^2 + Z_k^2);
F_xyz = [3*mu*X_k^2/r^5 - mu/r^3, 3*mu*X_k*Y_k/r^5, 3*mu*X_k*Z_k/r^5;
	3*mu*X_k*Y_k/r^5, 3*mu*Y_k^2/r^5 - mu/r^3, 3*mu*Y_k*Z_k/r^5;
	3*mu*X_k*Z_k/r^5, 3*mu*Y_k*Z_k/r^5, 3*mu*Z_k^2/r^5 - mu/r^3];
F_Xk = [zeros(3, 3), eye(3);
	F_xyz, zeros(3, 3)];
A_k = expm(F_Xk * Ts);
end

function [xhat_p, P_p] = ekf_predict(xhat_k, P_k, Q, Ts, mu)
A_k = calc_Ak(xhat_k, mu, Ts);
xhat_p = A_k * xhat_k;
P_p = A_k * P_k * A_k' + Q;
end

function [xhat_u, P_u] = ekf_update(xhat_p, P_p, y_k, C_k, R)
K_k = P_p * C_k' * inv(C_k * P_p * C_k' + R);
xhat_u = xhat_p + K_k * (y_k - measure_unwrapped(xhat_p));
P_u = P_p - K_k * C_k * P_p;
end