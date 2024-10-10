clc; clear; close all;

%% setup problem 3.a
L1 = [0, 0]; L2 = [5, 5]; L3 = [2.5, 0];
R1 = 2.5; R2 = 5; R3 = 3;

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
	options = optimoptions('fminunc', 'OptimalityTolerance', 1e-12, 'OutputFcn', @(x, optimValues, state) outfun(x, optimValues, state, i));
	[x, fval] = fminunc(@(x) circle_fix_eq_2d(x, L1, L2, L3, R1, R2, R3), guesses(i, :), options);
	errvals(i, 1:iters(i)) = errvals(i, 1:iters(i)) - fval;
	fprintf('Optimal position for guess %d(%.1f, %.1f): (%.2f, %.2f)\n', i, guesses(i, 1), guesses(i, 2), x(1), x(2));
end

% %% 2.a plot the circles and the optimal positions
figure;
hold on;
th = 0:pi/50:2*pi;
x1 = R1*cos(th) + L1(1);
y1 = R1*sin(th) + L1(2);
x2 = R2*cos(th) + L2(1);
y2 = R2*sin(th) + L2(2);
x3 = R3*cos(th) + L3(1);
y3 = R3*sin(th) + L3(2);
plot(x1, y1, 'b', 'LineWidth', 1, 'DisplayName', 'Circle L1');
plot(x2, y2, 'r', 'LineWidth', 1, 'DisplayName', 'Circle L2');
plot(x3, y3, 'g', 'LineWidth', 1, 'DisplayName', 'Circle L3');
for i = 1:size(guesses, 1)
	plot(iterates(i, 1, iters(i)), iterates(i, 2, iters(i)), '.', 'MarkerSize', 20, 'DisplayName', sprintf('x_{init}=(%.1f, %.1f)', guesses(i, 1), guesses(i, 2)));
end
axis equal;
xlabel('x'); ylabel('y');
title('HW2 P3a: Circles and Position Fixes');
legend('Location', 'best');
grid on; grid minor;
% saveas(fig, 'circles_fixes.svg');

%% setup problem 3.b
L1 = [0, 0]; L2 = [5, 5]; L3 = [2.5, 0];
R1 = 2.5; R2 = 6; R3 = 2;

iters = zeros(size(guesses, 1), 1);
iterates = zeros(size(guesses, 1), size(guesses, 2), 100);
errvals = zeros(size(guesses, 1), 100);

for i = 1:size(guesses, 1)
	options = optimoptions('fminunc', 'OptimalityTolerance', 1e-12, 'OutputFcn', @(x, optimValues, state) outfun(x, optimValues, state, i));
	[x, fval] = fminunc(@(x) circle_fix_eq_2d(x, L1, L2, L3, R1, R2, R3), guesses(i, :), options);
	errvals(i, 1:iters(i)) = errvals(i, 1:iters(i)) - fval;
	fprintf('Optimal position for guess %d(%.1f, %.1f): (%.2f, %.2f)\n', i, guesses(i, 1), guesses(i, 2), x(1), x(2));
end

%% 3.b plot the circles and the optimal positions for inconsistent circles
figure;
hold on;
th = 0:pi/50:2*pi;
x1 = R1*cos(th) + L1(1);
y1 = R1*sin(th) + L1(2);
x2 = R2*cos(th) + L2(1);
y2 = R2*sin(th) + L2(2);
x3 = R3*cos(th) + L3(1);
y3 = R3*sin(th) + L3(2);
plot(x1, y1, 'b', 'LineWidth', 1, 'DisplayName', 'Circle L1');
plot(x2, y2, 'r', 'LineWidth', 1, 'DisplayName', 'Circle L2');
plot(x3, y3, 'g', 'LineWidth', 1, 'DisplayName', 'Circle L3');
for i = 1:size(guesses, 1)
	plot(iterates(i, 1, iters(i)), iterates(i, 2, iters(i)), '.', 'MarkerSize', 20, 'DisplayName', sprintf('x_{init}=(%.1f, %.1f)', guesses(i, 1), guesses(i, 2)));
end
axis equal;
xlabel('x'); ylabel('y');
title('HW2 P3b: Circles and Inconsistent Position Fixes');
legend('Location', 'best');
grid on; grid minor;

%% 3.c plot position fixes form a grid of initial guesses
guess_grid = -5:2.5:10;
guesses = zeros(length(guess_grid)^2, 2);
for i = 1:length(guess_grid)
	for j = 1:length(guess_grid)
		guesses((i-1)*length(guess_grid)+j, :) = [guess_grid(i), guess_grid(j)];
	end
end

iters = zeros(size(guesses, 1), 1);
iterates = zeros(size(guesses, 1), size(guesses, 2), 100);
errvals = zeros(size(guesses, 1), 100);
xmin = zeros(size(guesses, 1), 2);
for i = 1:size(guesses, 1)
	options = optimoptions('fminunc', 'OptimalityTolerance', 1e-12, 'OutputFcn', @(x, optimValues, state) outfun(x, optimValues, state, i));
	[x, fval] = fminunc(@(x) circle_fix_eq_2d(x, L1, L2, L3, R1, R2, R3), guesses(i, :), options);
	errvals(i, 1:iters(i)) = errvals(i, 1:iters(i)) - fval;
	xmin(i, :) = x;
	fprintf('Optimal position for guess %d(%.1f, %.1f): (%.2f, %.2f)\n', i, guesses(i, 1), guesses(i, 2), x(1), x(2));
end

% separate the guesses into groups based on thier final positions
xmin_uniq_x = uniquetol(xmin(:, 1), 0.1);
guess_groups = cell(length(xmin_uniq_x), 1);
xmin_groups = cell(length(xmin_uniq_x), 1);
for i = 1:length(xmin_uniq_x)
	guess_groups{i} = guesses(ismembertol(xmin(:, 1), xmin_uniq_x(i), 0.1), :);
	xmin_groups{i} = xmin(ismembertol(xmin(:, 1), xmin_uniq_x(i), 0.1), :);
end

group_colors = {'r', 'b'}; % make sure there are enough for the number of groups
%% plot the circles and the guess groups
figure;
hold on;

for i = 1:length(guess_groups)
	scatter(guess_groups{i}(:, 1), guess_groups{i}(:, 2), 35, 'filled', ...
		'DisplayName', sprintf('x_{min}=(%.2f, %.2f)', mean(xmin_groups{i}(:, 1)), mean(xmin_groups{i}(:, 2))), ...
		'MarkerFaceColor', group_colors{i});
end

axis equal;
xlabel('x'); ylabel('y');
title('HW2 P3c: Grid and Position Fixes');
legend('Location', 'best');
grid on; grid minor;


function error = circle_fix_eq_2d(x, L1, L2, L3, R1, R2, R3)
L1_error = (x(1) - L1(1))^2 + (x(2) - L1(2))^2 - R1^2;
L2_error = (x(1) - L2(1))^2 + (x(2) - L2(2))^2 - R2^2;
L3_error = (x(1) - L3(1))^2 + (x(2) - L3(2))^2 - R3^2;
error = L1_error^2 + L2_error^2 + L3_error^2;
end

% function to capture iterations and their values
function stop = outfun(x, optimValues, ~, guessnum)
global iters iterates errvals;
iters(guessnum) = iters(guessnum) + 1;
iterates(guessnum, :, iters(guessnum)) = x;
errvals(guessnum, iters(guessnum)) = optimValues.fval;
stop = false;
end