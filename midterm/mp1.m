clc; clear; close all;

% given position vectors between w, P, L1, L2, L3 (w = L1)
r_L1_w = [0;0;0]; r_L2_w = [4;2;0]; r_L3_w = [1;4;0]; r_P_w = [2.5;2;0];
S = [0;1;0]; % star vector

% derived position vectors
r1 = r_L1_w - r_P_w; % r_L1_P
r2 = r_L2_w - r_P_w; % r_L2_P
r3 = r_L3_w - r_P_w; % r_L3_P

%% part a derived angles
theta1 = get_angle_between(r1, S);
theta2 = get_angle_between(r2, S);
theta3 = get_angle_between(r3, S);

fprintf("P1.a: true bearing angles\n- theta1: %.3f\n- theta2: %.3f\n- theta3: %.3f\n", theta1, theta2, theta3);

%% part b position estimates
beta1 = 0.05; beta2 = -0.03; beta3 = 0.04; % bearing biases
sigma = 0.1:0.1:1; % bearing noise std devs
num_samples = 100000;
prob_inside = zeros(length(sigma),1);
avg_dist = zeros(length(sigma),1);
for jj = 1:length(sigma) % For each sigma value
	% Do some setup stuff (if needed)
	dist_to_centroid = zeros(num_samples,1);
	is_inside = zeros(num_samples,1);
	data.P12 = zeros(2,num_samples);
	data.P13 = zeros(2,num_samples);
	data.P23 = zeros(2,num_samples);
	for ii = 1:num_samples % For each sample
		theta1_noisy = get_noisy_bearing(theta1, beta1, sigma(jj));
		theta2_noisy = get_noisy_bearing(theta2, beta2, sigma(jj));
		theta3_noisy = get_noisy_bearing(theta3, beta3, sigma(jj));
		% Calculate 3 estimated positions based on noisy bearing measurments
		% positions of lighthouses are wrt to w
		P12 = get_pos_fix_2d(theta1_noisy, theta2_noisy, r_L1_w, r_L2_w);
		P13 = get_pos_fix_2d(theta1_noisy, theta3_noisy, r_L1_w, r_L3_w);
		P23 = get_pos_fix_2d(theta2_noisy, theta3_noisy, r_L2_w, r_L3_w);
		% Determine whether actual position is inside the triangle
		is_inside(ii) = is_inside_triangle(r_P_w, P12, P13, P23);
		% Determine distance from triangle centroid to actual position
		dist_to_centroid(ii) = norm(mean([P12, P13, P23], 2) - r_P_w);
		% Save these results somewhere for use outside current loop
		data.P12(:,ii) = P12(1:2);
		data.P13(:,ii) = P13(1:2);
		data.P23(:,ii) = P23(1:2);
	end
	% figure;
	% plot(data.P12(1,:), data.P12(2,:), 'y.', 'DisplayName', 'P12');
	% hold on;
	% plot(data.P13(1,:), data.P13(2,:), 'm.', 'DisplayName', 'P13');
	% plot(data.P23(1,:), data.P23(2,:), 'c.', 'DisplayName', 'P23');
	% plot(r_P_w(1), r_P_w(2), 'rx', 'DisplayName', 'P', 'MarkerSize', 10);
	% plot(r_L1_w(1), r_L1_w(2), 'go', 'DisplayName', 'L1', 'MarkerSize', 10);
	% plot(r_L2_w(1), r_L2_w(2), 'go', 'DisplayName', 'L2', 'MarkerSize', 10);
	% plot(r_L3_w(1), r_L3_w(2), 'go', 'DisplayName', 'L3', 'MarkerSize', 10);
	% xlabel('x'); ylabel('y');
	% grid on; grid minor;
	% legend('Location', 'best');
	% title("$\sigma = " + sigma(jj) + "$", 'Interpreter', 'latex');
	% Use data obtained in inner loop to calculate average distance from
	% triangle centroid to actual position and the percentage of times the
	% actual position was inside the triangle formed by the estimated positions.
	prob_inside(jj) = sum(is_inside)/num_samples * 100;
	avg_dist(jj) = mean(dist_to_centroid);
end

%% part c plot p inside versus sigma
figure;
plot(sigma, prob_inside, "LineWidth", 1.5);
xlabel('$\sigma$ (rad)', 'Interpreter', 'latex');
ylabel('Percentage Inside Triangle (%)');
grid on; grid minor;
title('P1.c: Percentage Inside Triangle vs. $\sigma$', 'Interpreter', 'latex');

%% part d plot avg dist versus sigma
figure;
plot(sigma, avg_dist, "LineWidth", 1.5);
xlabel('$\sigma$ (rad)', 'Interpreter', 'latex');
ylabel('Average Distance to Centroid (m)');
grid on; grid minor;
title('P1.d: Average Distance to Centroid vs. $\sigma$', 'Interpreter', 'latex');

%% tests
% fprintf("angle between\n");
% get_angle_between([1;0;0], [0;1;0]) % pi/2
% get_angle_between([1;0;0], [1;0;0]) % 0
% get_angle_between([1;1;0], [1;0;0]) % pi/4
% get_angle_between([2;2;0], [0;1;0]) % pi/4

% fprintf("triange area\n");
% get_triangle_area([0;0;0], [1;0;0], [0;1;0]) % 0.5
% get_triangle_area([0;0;0], [2;0;0], [0;1;0]) % 1
% get_triangle_area([0;1;0], [2;0;0], [0;0;0]) % 1
% get_triangle_area([1;1;0], [2;2;0], [0;2;0]) % 1

% fprintf("is inside triangle\n");
% is_inside_triangle([0.1;0.1;0], r_L1_w, r_L2_w, r_L3_w) % 1
% is_inside_triangle([0.5;0.5;0], r_L1_w, r_L2_w, r_L3_w) % 1
% is_inside_triangle([0.6;0.6;0], r_L1_w, r_L2_w, r_L3_w) % 0

%% functions
function angle = get_angle_between(v1, v2)
raw_angle = atan2(norm(cross(v1, v2)), dot(v1, v2));
cross_prod = cross(v1, v2);
angle = sign(cross_prod(3))*raw_angle;
end

function t = T(theta)
t = tan(pi/2 - theta);
end

function pos = get_pos_fix_2d(theta1, theta2, L1, L2)
T1 = T(theta1); T2 = T(theta2);
A = [T1, -1; T2, -1];
B = [T1*L1(1) - L1(2); T2*L2(1) - L2(2)];
pos = A\B;
pos = [pos;0];
end

function noisy = get_noisy_bearing(theta, bias ,std_dev)
noisy = theta + bias + std_dev*randn;
end

function area = get_triangle_area(A, B, C)
% Calculate area of triangle ABC using heron's formula
a = norm(B - C);
b = norm(A - C);
c = norm(A - B);
s = (a + b + c)/2;
area = sqrt(s*(s - a)*(s - b)*(s - c));
end

function isin = is_inside_triangle(P, A, B, C)
% Determine if point P is inside triangle ABC
area = get_triangle_area(A, B, C);
A1 = get_triangle_area(P, A, B);
A2 = get_triangle_area(P, B, C);
A3 = get_triangle_area(P, A, C);
rhs = (A1 + A2 + A3);
isin = abs(area - rhs) <= 1e-6;
end
