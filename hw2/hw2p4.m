clc; clear; close all;

% Setup problem 4

L1 = [0, 0]; L2 = [5, 5]; L3 = [2.5, 0];
R1 = 2.5; R2 = 5; R3 = 3;

guess = [10, 0];
rand_samples = 100;
true_pos = [0.7212, 2.4080]; % from the question

%% 4.a Position fix with noise
% Setup optimization
xmin = zeros(rand_samples, 2);
a = 2;
options = optimoptions('fminunc', 'OptimalityTolerance', 1e-12);
for i= 1:rand_samples
	R1_noisy = add_noise(R1, a);
	R2_noisy = add_noise(R2, a);
	R3_noisy = add_noise(R3, a);
	[xmin(i, :), fval] = fminunc(@(x) circle_fix_eq_2d(x, L1, L2, L3, R1_noisy, R2_noisy, R3_noisy), guess, options);
end

% Plot circles and optimal positions
figure;
hold on;
th = 0:pi/50:2*pi;
x1 = R1*cos(th) + L1(1);
y1 = R1*sin(th) + L1(2);
x2 = R2*cos(th) + L2(1);
y2 = R2*sin(th) + L2(2);
x3 = R3*cos(th) + L3(1);
y3 = R3*sin(th) + L3(2);
plot(x1, y1, 'c', 'LineWidth', 1, 'DisplayName', 'Circle L1');
plot(x2, y2, 'm', 'LineWidth', 1, 'DisplayName', 'Circle L2');
plot(x3, y3, 'b', 'LineWidth', 1, 'DisplayName', 'Circle L3');
scatter(xmin(:, 1), xmin(:, 2), 'filled', 'DisplayName', 'Position Fixes', 'MarkerFaceColor', 'r');
scatter(true_pos(1), true_pos(2), 'filled', 'DisplayName', 'True Position Fix', 'MarkerFaceColor', 'g');
axis equal;
xlabel('x'); ylabel('y');
title('HW2 P4a: Circles and Position Fixes with noise');
legend('Location', 'best');
grid on; grid minor;

%% 4.b Max error over a range of a
a_range = 0.05:0.05:10;
options = optimoptions('fminunc', 'OptimalityTolerance', 1e-12, 'Display', 'none');
max_errors = zeros(length(a_range), 1);
for i = 1:length(a_range)
	xmin = zeros(rand_samples, 2);
	a = a_range(i);
	for j= 1:rand_samples
		R1_noisy = add_noise(R1, a);
		R2_noisy = add_noise(R2, a);
		R3_noisy = add_noise(R3, a);
		[xmin(j, :), fval] = fminunc(@(x) circle_fix_eq_2d(x, L1, L2, L3, R1_noisy, R2_noisy, R3_noisy), guess, options);
	end
	max_error = max(sqrt((xmin(:, 1) - true_pos(1)).^2 + (xmin(:, 2) - true_pos(2)).^2));
	max_errors(i) = max_error;
	fprintf('Max error for a=%.2f: %.4f\n', a, max_error);
end

% Plot max error vs a
figure;
semilogy(a_range, max_errors, 'LineWidth', 2);
xlabel('a'); ylabel('Max Error');
title('HW2 P4b: Max Error vs a');
grid on; grid minor;


function error = circle_fix_eq_2d(x, L1, L2, L3, R1, R2, R3)
L1_error = (x(1) - L1(1))^2 + (x(2) - L1(2))^2 - R1^2;
L2_error = (x(1) - L2(1))^2 + (x(2) - L2(2))^2 - R2^2;
L3_error = (x(1) - L3(1))^2 + (x(2) - L3(2))^2 - R3^2;
error = L1_error^2 + L2_error^2 + L3_error^2;
end

function noisy_R = add_noise(R, a)
noisy_R = R + a * (rand(1) - 0.5);
end

% function to capture iterations and their values
function stop = outfun(x, optimValues, ~, guessnum)
global iters iterates errvals;
iters(guessnum) = iters(guessnum) + 1;
iterates(guessnum, :, iters(guessnum)) = x;
errvals(guessnum, iters(guessnum)) = optimValues.fval;
stop = false;
end