clc; clear; close all;

% set up adjoint eq
tf = 10;
ts = 0.001;
tspan = 0:ts:(tf-ts);
tspan_inv = flip(tspan);
p_f = [1;0]; % final p because were integrating in reverse
T = 1;
Lambdas = [3, 4, 5];

data = {};
for i=1:length(Lambdas)
	[t_ode, p_ode] = ode45(@(t, p) adj_eq(t, p, Lambdas(i), T, tf), tspan_inv, p_f);
	p2 = p_ode(:,2);
	
	% calculate G
	neg_TG = zeros(length(t_ode), 1);
	t_xaxis = zeros(length(t_ode), 1);
	for j = 1:length(t_ode)
		neg_TG(j) = -T * (Lambdas(i) * p2(j) ) / (T * (tf - t_ode(j)));
		t_xaxis(j) = (tf-t_ode(j)) / T;
	end
	data{i} = [t_xaxis, neg_TG];
end

% plot G
figure;
Lambda_markers = ["--", "-", "-."];
hold on;
for i=1:length(Lambdas)
	plot(data{i}(:,1), data{i}(:,2), Lambda_markers(i), "LineWidth", 1.5, "DisplayName", "$\Lambda=$" + string(Lambdas(i)));
end
xlabel("$(\bar{t_f}-t)/T$", "Interpreter", "latex", "FontSize", 14);
ylabel("$-TG(\bar{t_f},t)$", "Interpreter", "latex", "FontSize", 14);
title("HW5 P7: $-TG$ versus $(\bar{t_f}-t)/T$", "Interpreter", "latex", "FontSize", 16);
grid on;
legend("Location", "best", "Interpreter", "latex");


% adjoint equation
function p_dot = adj_eq(t, p, Lambda, T, tf)
p_dot = [
	0, Lambda/(tf-t);
	-1, 1/T;
	] * p;
end