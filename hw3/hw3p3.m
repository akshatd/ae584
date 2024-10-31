clc; clear; close all;

% part a

L1 = [0;0;0]; L2 = [5;0;0]; % lighthouses
S = [0; 1; 0]; % vector pointing of star
pos_actual = [2.5;2.5;0];
pos_fixes = get_pos_fix(S, L1, L2, 90, 135, -5:2:5);
plot_2d_proj(pos_actual, pos_fixes, L1, L2);
plot_sphere(pos_actual, pos_fixes, L1, L2);

function cost = costfn(x, S, L1, L2, theta, psi)
% setup
x = x'; % fix x's dims as used by fminunc
r_S_Y = S;
r_Y_L1 = x-L1;
r_L2_L1 = L2-L1;
r_L2_Y = L2-x;
theta = deg2rad(theta);
psi = deg2rad(psi);

% eq4
l2 = dot(r_Y_L1, r_L2_L1);
r2 = - norm(r_Y_L1) * norm(r_L2_Y) * cos(theta) + norm(r_Y_L1)^2;
err_subt_angle = l2-r2;

% eq5
l3 = dot(r_L2_Y, r_S_Y);
r3 = norm(r_L2_Y) * cos(psi);
err_bearing = l3-r3;

cost = err_subt_angle^2 + err_bearing^2;
end


function pos_fixes = get_pos_fix(S, L1, L2, theta, psi, guess_range)
p_inits = zeros(length(guess_range)^3, 3);
for i = 1:length(guess_range)
	for j = 1:length(guess_range)
		for k = 1:length(guess_range)
			p_inits((i-1)*length(guess_range)^2 + (j-1)*length(guess_range) + k, :) = [guess_range(i), guess_range(j), guess_range(k)];
		end
		% p_inits((i-1)*length(guess_range)+j, :) = [guess_range(i), guess_range(j)];
	end
end

costs = zeros(size(p_inits, 1), 1);
pos_fixes = zeros(size(p_inits));
options = optimoptions('fminunc', 'OptimalityTolerance', 1e-12, 'Display', 'off');
for i = 1:size(p_inits, 1)
	[x, fval] = fminunc(@(x) costfn(x, S, L1, L2, theta, psi), p_inits(i, :), options);
	costs(i) = fval;
	pos_fixes(i, :) = x;
end
end

function plot_2d_proj(pos_actual, pos_fixes, L1, L2)
x1 = L1(1); y1 = L1(2); z1 = L1(3);
x2 = L2(1); y2 = L2(2); z2 = L2(3);

figure;
sgtitle('HW3 P3: 2D Projections of solutions with actual location and lighthouses')

subplot(2,2,1)
scatter(pos_fixes(:,1), pos_fixes(:,3), 50, 'b', 'filled')
hold on
scatter(x1, z1, 100, 'g', 'filled')
scatter(x2, z2, 100, 'g', 'filled')
scatter(pos_actual(1), pos_actual(3), 100, 'r', 'filled')
hold off
grid on
ylabel('$z$ (m)','interpreter','latex','fontsize',15)
xlabel('$x$ (m)','interpreter','latex','fontsize',15)
legend('Obtained solutions', 'Lighthouse 1', 'Lighthouse 2', 'Actual Location')

subplot(2,2,3)
scatter(pos_fixes(:,1), pos_fixes(:,2), 50, 'b', 'filled')
hold on
scatter(x1, y1, 100, 'g', 'filled')
scatter(x2, y2, 100, 'g', 'filled')
scatter(pos_actual(1), pos_actual(2), 100, 'r', 'filled')
hold off
grid on
ylabel('$y$ (m)','interpreter','latex','fontsize',15)
xlabel('$x$ (m)','interpreter','latex','fontsize',15)
legend('Obtained solutions', 'Lighthouse 1', 'Lighthouse 2', 'Actual Location')

subplot(2,2,4)
scatter(pos_fixes(:,3), pos_fixes(:,2), 50, 'b', 'filled')
hold on
scatter(z1, y1, 100, 'g', 'filled')
scatter(z2, y2, 100, 'g', 'filled')
scatter(pos_actual(3), pos_actual(2), 100, 'r', 'filled')
hold off
grid on
ylabel('$y$ (m)','interpreter','latex','fontsize',15)
xlabel('$z$ (m)','interpreter','latex','fontsize',15)
legend('Obtained solutions', 'Lighthouse 1', 'Lighthouse 2', 'Actual Location')
end

function plot_sphere(pos_actual, pos_fixes, L1, L2)
x1 = L1(1); y1 = L1(2); z1 = L1(3);
x2 = L2(1); y2 = L2(2); z2 = L2(3);

figure;
% 3D plot
scatter3(pos_fixes(:,1), pos_fixes(:,2), pos_fixes(:,3), 50, 'b', 'filled')
hold on
scatter3(x1, y1, z1, 100, 'g', 'filled')
scatter3(x2, y2, z2, 100, 'g', 'filled')
scatter3(pos_actual(1), pos_actual(2), pos_actual(3), 100, 'r', 'filled')

% Plot the sphere
[Xsp, Ysp, Zsp] = sphere;
surf(2.5.*Xsp + 2.5, 2.5.*Ysp, 2.5.*Zsp, 'FaceAlpha', 0.25)

hold off
zlabel('$z$ (m)','interpreter','latex','fontsize',15)
ylabel('$y$ (m)','interpreter','latex','fontsize',15)
xlabel('$x$ (m)','interpreter','latex','fontsize',15)
legend('Obtained solutions', 'Lighthouse 1', 'Lighthouse 2', 'Actual Location')
title('HW3 P3: 3D plot of the obtained solutions with actual location and lighhouses')
end
