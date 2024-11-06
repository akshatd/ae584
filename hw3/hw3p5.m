clc; clear; close all;

L1 = [0;0;0]; L2 = [5;0;0]; % lighthouses
S_r = [0; 1; 0]; S_v = [0; 0; 1]; % vector pointing of star
psi_r = 135; psi_v = 90; % relative to lighthouse 2
theta = 90; % subtended angle between lighthouses
pos_actual = [2.5;2.5;0];
rv_samples = 200;
rv_mean = 0; rv_std = 2;

pos_fixes = zeros(rv_samples, 3);

for i = 1:rv_samples
	theta = theta + rv_std*randn + rv_mean;
	psi_r = psi_r + rv_std*randn + rv_mean;
	psi_v = psi_v + rv_std*randn + rv_mean;
	pos_fixes(i, :) = get_pos_fix(S_r, S_v, L1, L2, theta, psi_r, psi_v, -1:3:5);
end

%% plot
plot_2d_proj(pos_actual, pos_fixes, L1, L2);
plot_pos_fix_err(pos_actual, pos_fixes);

function cost = costfn(x, S1, S2, L1, L2, theta, psi1, psi2)
% setup
x = x'; % fix x's dims as used by fminunc
r_S1_Y = S1;
r_S2_Y = S2;
r_Y_L1 = x-L1;
r_L2_L1 = L2-L1;
r_L2_Y = L2-x;
theta = deg2rad(theta);
psi1 = deg2rad(psi1);
psi2 = deg2rad(psi2);

% eq4
l4 = dot(r_Y_L1, r_L2_L1);
r4 = - norm(r_Y_L1) * norm(r_L2_Y) * cos(theta) + norm(r_Y_L1)^2;
err_subt_angle = l4-r4;

% eq5
l5 = dot(r_L2_Y, r_S1_Y);
r5 = norm(r_L2_Y) * cos(psi1);
err_bearing1 = l5-r5;

% eq6
l6 = dot(r_L2_Y, r_S2_Y);
r6 = norm(r_L2_Y) * cos(psi2);
err_bearing2 = l6-r6;

cost = err_subt_angle^2 + err_bearing1^2 + err_bearing2^2;
end


function pos = get_pos_fix(S1, S2, L1, L2, theta, psi1, psi2, guess_range)
p_inits = zeros(length(guess_range)^3, 3);
for i = 1:length(guess_range)
	for j = 1:length(guess_range)
		for k = 1:length(guess_range)
			p_inits((i-1)*length(guess_range)^2 + (j-1)*length(guess_range) + k, :) = [guess_range(i), guess_range(j), guess_range(k)];
		end
	end
end

costs = zeros(size(p_inits, 1), 1);
pos_fixes = zeros(size(p_inits));
options = optimoptions('fminunc', 'OptimalityTolerance', 1e-12, 'Display', 'off');
for i = 1:size(p_inits, 1)
	[x, fval] = fminunc(@(x) costfn(x, S1, S2, L1, L2, theta, psi1, psi2), p_inits(i, :), options);
	costs(i) = fval;
	pos_fixes(i, :) = x;
end

% pick point with the lowest cost that isnt closest to a lighthouse
epsilon = 0.01;
pos_fix = [100, 100]; % large number
pos_fix_cost = 100; % large number
for i = 1:size(pos_fixes, 1)
	if costs(i) < pos_fix_cost
		dist_L1 = norm(pos_fixes(i, :) - L1');
		dist_L2 = norm(pos_fixes(i, :) - L2');
		if dist_L1 > epsilon && dist_L2 > epsilon
			pos_fix = pos_fixes(i, :);
			pos_fix_cost = costs(i);
		end
	end
end
pos = pos_fix;

end

function plot_2d_proj(pos_actual, pos_fixes, L1, L2)
x1 = L1(1); y1 = L1(2); z1 = L1(3);
x2 = L2(1); y2 = L2(2); z2 = L2(3);

figure;
sgtitle('HW3 P5: 2D Projections of obtained locations with actual location and lighthouses')

subplot(2,2,1)
scatter(pos_fixes(:,1), pos_fixes(:,3), 50, 'b', 'filled')
hold on
scatter(x1, z1, 100, 'g', 'filled')
scatter(x2, z2, 100, 'g', 'filled')
scatter(pos_actual(1), pos_actual(3), 100, 'r', 'filled')
hold off
grid on
xlim([-3 5.5])
ylim([-3 3.5])
ylabel('$z$ (m)','interpreter','latex','fontsize',15)
xlabel('$x$ (m)','interpreter','latex','fontsize',15)
legend('Obtained locations', 'Lighthouse 1', 'Lighthouse 2', 'Actual Location')

subplot(2,2,3)
scatter(pos_fixes(:,1), pos_fixes(:,2), 50, 'b', 'filled')
hold on
scatter(x1, y1, 100, 'g', 'filled')
scatter(x2, y2, 100, 'g', 'filled')
scatter(pos_actual(1), pos_actual(2), 100, 'r', 'filled')
hold off
grid on
xlim([-3 5.5])
ylim([-3 3.5])
ylabel('$y$ (m)','interpreter','latex','fontsize',15)
xlabel('$x$ (m)','interpreter','latex','fontsize',15)
legend('Obtained locations', 'Lighthouse 1', 'Lighthouse 2', 'Actual Location')

subplot(2,2,4)
scatter(pos_fixes(:,3), pos_fixes(:,2), 50, 'b', 'filled')
hold on
scatter(z1, y1, 100, 'g', 'filled')
scatter(z2, y2, 100, 'g', 'filled')
scatter(pos_actual(3), pos_actual(2), 100, 'r', 'filled')
hold off
grid on
xlim([-3 5.5])
ylim([-3 3.5])
ylabel('$y$ (m)','interpreter','latex','fontsize',15)
xlabel('$z$ (m)','interpreter','latex','fontsize',15)
legend('Obtained locations', 'Lighthouse 1', 'Lighthouse 2', 'Actual Location')
end

function plot_pos_fix_err(pos_actual, pos_fixes)
err = vecnorm(pos_fixes - pos_actual', 2, 2); % 2 norm along rows
err_mean = mean(err);
err_std = std(err);
figure;
hold on;
histogram(err, 20, 'DisplayName', "Position fix error distribution");
xline(err_mean, '--r', 'LineWidth', 2, 'DisplayName', "Mean: " + err_mean);
xline(err_mean + err_std, '--g', 'LineWidth', 2, 'DisplayName', "+1 Std");
xline(err_mean - err_std, '--g', 'LineWidth', 2, 'DisplayName', "-1 Std");
hold off;
xlabel('Error (m)');
ylabel('Frequency');
legend show;
title('HW3 P5: Distribution of position fix errors');
end