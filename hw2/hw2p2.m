clc; clear; close all;

% setup problem 2
L1 = [0, 0]; L2 = [5, 5];
R1 = 2.5; R2 = 5;

% setup minimization
guesses = [
	10, 0;
	0, 10;
	];

global iters iterates errvals;
iters = zeros(size(guesses, 1), 1);
iterates = zeros(size(guesses, 1), size(guesses, 2), 100);
errvals = zeros(size(guesses, 1), 100);

for i = 1:size(guesses, 1)
	options = optimoptions('fminunc', 'Display', 'iter', 'OptimalityTolerance', 1e-6, 'OutputFcn', @(x, optimValues, state) outfun(x, optimValues, state, i));
	[x, fval] = fminunc(@(x) circle_fix_eq_2d(x, L1, L2, R1, R2), guesses(i, :), options);
	fprintf('Optimal position for guess %d: (%f, %f)\n', i, x(1), x(2));
end

% plot the circles and the optimal positions
fig = figure;
hold on;
th = 0:pi/50:2*pi;
x1 = R1*cos(th) + L1(1);
y1 = R1*sin(th) + L1(2);
x2 = R2*cos(th) + L2(1);
y2 = R2*sin(th) + L2(2);
plot(x1, y1, 'b', 'LineWidth', 2, 'DisplayName', 'Circle L1');
plot(x2, y2, 'r', 'LineWidth', 2, 'DisplayName', 'Circle L2');
for i = 1:size(guesses, 1)
	plot(iterates(i, 1, iters(i)), iterates(i, 2, iters(i)), '.', 'MarkerSize', 20, 'DisplayName', sprintf('Guess %d: (%.1f, %.1f)', i, guesses(i, 1), guesses(i, 2)));
end
axis equal;
xlabel('x'); ylabel('y');
title('HW2 P2a: Circles and Optimal Positions');
legend('Location', 'best');
grid on; grid minor;
saveas(fig, 'circles.svg');

% function to calculate the error of the position given points and ranges
function error = circle_fix_eq_2d(x, L1, L2, R1, R2)
L1_error = (x(1) - L1(1))^2 + (x(2) - L1(2))^2 - R1^2;
L2_error = (x(1) - L2(1))^2 + (x(2) - L2(2))^2 - R2^2;
error = norm([L1_error, L2_error]);
end

% function to capture iterations and their values
function stop = outfun(x, optimValues, ~, guessnum)
global iters iterates errvals;
iters(guessnum) = iters(guessnum) + 1;
iterates(guessnum, :, iters(guessnum)) = x;
errvals(guessnum, iters(guessnum)) = optimValues.fval;
stop = false;
end