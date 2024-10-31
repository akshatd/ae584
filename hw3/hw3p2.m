clc; clear; close all;


% subtended(0, 10)
% subtended(-10, 10)
% subtended(-10, -10)
% subtended(10, -10)
% subtended(10, 10)
% subtended(10, 370)
% subtended(370, 10)
% subtended(150, -165)
% subtended(-165, 150)
% subtended(150, 50)
% subtended(50, 150)
% subtended(170, -170)
% subtended(-170, 170)
% subtended(190, 200)
% subtended(190, 170)
% subtended(-10, -20)
% subtended(90, -30);

% part a
L1 = [0; 0]; L2 = [4; 2]; % lighthouses
theta1 = -165; theta2 = 150; % bearing to the lighthouses
theta12 = 45; % subtended angle between lighthouses
S = [0; 1]; % vector pointing of star
% psi = theta2; % bearing from star to L2

guess_range = -2:2:10;
pos_fix = get_pos_fix(S, L1, L2, theta12, theta2, guess_range);

figure;
hold on;
scatter(pos_fix(1), pos_fix(2), 35, 'r', 'filled', 'DisplayName', 'Position fix');
scatter(L1(1), L1(2), 35, 'g', 'filled', 'DisplayName', 'Lighthouse 1');
scatter(L2(1), L2(2), 35, 'g', 'filled', 'DisplayName', 'Lighthouse 2');
xlim([-1, 6.5]); ylim([-1, 6.5]);
xlabel('x'); ylabel('y');
grid on; grid minor;
legend('Location', 'best');
title('HW3 P2a: Locations of position fix and light houses');

% part b
L1 = [0;0]; L2 = [4;2]; L3 = [1;4]; % lighthouses
theta1 = -140; theta2 = 90; theta3 = -30; % bearing to the lighthouses
theta12 = 130; theta23 = 120; theta31 = 110; % subtended angle between lighthouses
S = [0; 1]; % vector pointing of star

guess_range = -2:2:10;
pos_fix_12 = get_pos_fix(S, L1, L2, theta12, theta2, guess_range);
pos_fix_23 = get_pos_fix(S, L2, L3, theta23, theta3, guess_range);
pos_fix_31 = get_pos_fix(S, L3, L1, theta31, theta1, guess_range);
pos_fix_final = (pos_fix_12 + pos_fix_23 + pos_fix_31)/3;

figure;
hold on;
scatter(pos_fix_final(1), pos_fix_final(2), 35, 'r', 'filled', 'DisplayName', 'Position fix center');
scatter(pos_fix_12(1), pos_fix_12(2), 35, 'b', 'filled', 'DisplayName', 'Position fix 1-2');
scatter(pos_fix_23(1), pos_fix_23(2), 35, 'b', 'filled', 'DisplayName', 'Position fix 2-3');
scatter(pos_fix_31(1), pos_fix_31(2), 35, 'b', 'filled', 'DisplayName', 'Position fix 3-1');
scatter(L1(1), L1(2), 35, 'g', 'filled', 'DisplayName', 'Lighthouse 1');
scatter(L2(1), L2(2), 35, 'g', 'filled', 'DisplayName', 'Lighthouse 2');
scatter(L3(1), L3(2), 35, 'g', 'filled', 'DisplayName', 'Lighthouse 3');
xlim([-1, 4.5]); ylim([-1, 4.5]);
xlabel('x'); ylabel('y');
grid on; grid minor;
legend('Location', 'best');
title('HW3 P2b: Locations of position fixes and light houses');



% function theta_subtended = subtended(theta1, theta2)
% % if theta1 < 0
% % 	theta1 = -theta1 + 360;
% % end
% % if theta2 < 0
% % 	theta2 = -theta2 + 360;
% % end
% % theta_subtended = mod(abs(theta1 - theta2), 180);
% if theta1 <= 0 && theta2 > 0
% elseif theta2 <= 0 && theta1 > 0
%     temp = abs(theta1) + abs(theta2);
% else
%     temp = theta1 - theta2;
% end
% 
% theta_subtended = mod(temp, 180);
% end

function cost = costfn(x, S, L1, L2, theta, psi)
% setup
x = x'; % fix x's dims as used by fminunc
r_S_Y = S;
r_Y_L1 = x-L1;
r_L2_L1 = L2-L1;
r_L2_Y = L2-x;
theta = deg2rad(theta);
psi = deg2rad(psi);

% eq2
l2 = dot(r_Y_L1, r_L2_L1);
r2 = - norm(r_Y_L1) * norm(r_L2_Y) * cos(theta) + norm(r_Y_L1)^2;
err_subt_angle = l2-r2;

% eq3
l3 = dot(r_L2_Y, r_S_Y);
r3 = norm(r_L2_Y) * cos(psi);
err_bearing = l3-r3;

cost = err_subt_angle^2 + err_bearing^2;
end

function pos = get_pos_fix(S, L1, L2, theta, psi, guess_range)
p_inits = zeros(length(guess_range)^2, 2);
for i = 1:length(guess_range)
	for j = 1:length(guess_range)
		p_inits((i-1)*length(guess_range)+j, :) = [guess_range(i), guess_range(j)];
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