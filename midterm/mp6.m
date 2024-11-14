clc; clear; close all;

Q = 1;
R = 2;

Tsteps = 100;

x0_hat = 0;
P0 = 1;

data.x = zeros(1, Tsteps+1);
data.x(1) = x0_hat;
data.y = zeros(1, Tsteps+1);
data.y(1) = x0_hat;
data.xhat = zeros(1, Tsteps+1);
data.xhat(1) = x0_hat;

xk = x0_hat;
xk_hat = x0_hat;
Pk = P0;

for i=1:Tsteps
	xkp1 = xk1(xk, Q);
	[xhat_p, P_p] = predict(xk_hat, Pk, Q);
	ykp1 = yk1(xkp1, R);
	[xkp1_hat, Pk] = update(xhat_p, P_p, ykp1, R);
	xk = xkp1;
	xk_hat = xkp1_hat;
	data.x(i) = xkp1;
	data.y(i) = ykp1;
	data.xhat(i) = xkp1_hat;
end

figure;
plot(1:Tsteps+1, data.x, 'r', "DisplayName", "x");
hold on;
plot(1:Tsteps+1, data.y, 'b', "DisplayName", "y");
plot(1:Tsteps+1, data.xhat, 'g', "DisplayName", "xhat");
legend;
xlabel('Time');
ylabel('Value');
grid on; grid minor;
title('P6: Kalman Filter');

function x = xk1(xk, cov)
x = xk + randn(1)*sqrt(cov);
end

function y = yk1(xk, cov)
y = xk + randn(1)*sqrt(cov);
end

function [xhat_p, P_p] = predict(xhat_u, P_u, Q)
xhat_p = xhat_u;
P_p = P_u + Q;
end

function [xhat_u, P_u] = update(xhat_p, P_p, yk, R)
K = P_p/(P_p + R);
xhat_u = xhat_p + K*(yk - xhat_p);
P_u = P_p - K*P_p;
end
