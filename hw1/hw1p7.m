clc; clear;close all;

% setup sim
Tfid = 0.01;
Tsim = 10;
color_orange = [0.8500 0.3250 0.0980];

% plot angular velocity over time
ang_vels = calc_angvel(0:Tfid:Tsim);
figure;
hold on;
plot(0:Tfid:Tsim, ang_vels(1,:), 'r', 'LineWidth', 2, 'DisplayName', '$\omega_i$');
plot(0:Tfid:Tsim, ang_vels(2,:), '--g', 'LineWidth', 2, 'DisplayName', '$\omega_j$');
plot(0:Tfid:Tsim, ang_vels(3,:), 'b', 'LineWidth', 2, 'DisplayName', '$\omega_k$');
ylabel('Angular Velocity (rad/s)');
title('HW1 P7(i): Angular Velocity vs Time');
xlabel('Time (s)');
legend("Location", "best", "Interpreter", "latex");
grid on; grid minor;
% saveas(fig, 'angular_velocity.svg');

% run simulation
x0 = [0;0;0]; % phi, theta, psi
[t, x] = ode45(@(t, x) dyn(t, x), 0:Tfid:Tsim, x0);

figure;
hold on;
plot(t, x(:,1), 'r', 'LineWidth', 2, 'DisplayName', '$\phi$');
plot(t, x(:,2), 'g', 'LineWidth', 2, 'DisplayName', '$\theta$');
plot(t, x(:,3), 'b', 'LineWidth', 2, 'DisplayName', '$\psi$');
ylabel('Angle (rad)');
title('HW1 P7(i): Euler Angles vs Time');
xlabel('Time (s)');
legend("Location", "best", "Interpreter", "latex");
grid on; grid minor;
% saveas(fig, 'euler_angles.svg');

% plot orientation matrix
omat = zeros(3,3,length(t));
for i = 1:length(t)
	omat(:,:,i) = calc_omat(x(i,:));
end

fig = figure;
sgtitle('HW1 P7(i): Orientation Matrix vs Time');

for i = 1:3
	for j = 1:3
		subplot(3,3,3*(i-1)+j);
		plot(t, squeeze(omat(i,j,:)), 'LineWidth', 2);
		ylabel(['\Theta_{', num2str(i), num2str(j), '}']);
		xlabel('Time (s)');
		grid on; grid minor;
	end
end
fig.Position(3:4) = [900, 800];
% saveas(fig, 'orientation_matrix.svg');

% get orientation matrix using poisson's formula
omat_poisson_0 = eye(3); % start with identity matrix
[t, omat_poisson] = ode45(@(t, x) calc_omat_poisson(t, x), 0:Tfid:Tsim, omat_poisson_0);

fig = figure;
sgtitle('HW1 P7(ii): Orientation Matrix comparison vs Time');

for i = 1:3
	for j = 1:3
		subplot(3,3,3*(i-1)+j);
		hold on;
		plot(t, squeeze(omat_poisson(:, 3*(j-1)+i)), 'LineWidth', 2, 'DisplayName', '$\Theta$ (Poisson)');
		plot(t, squeeze(omat(i,j,:)), '--', 'LineWidth', 2, 'Color', color_orange, 'DisplayName', '$\Theta$');
		ylabel(['\Theta_{', num2str(i), num2str(j), '}']);
		xlabel('Time (s)');
		grid on; grid minor;
	end
end
legend("Location", "best", "Interpreter", "latex");

fig.Position(3:4) = [900, 800];
% saveas(fig, 'orientation_matrix_poisson.svg');

% calculate euler angles from orientation matrix
euler_angles = zeros(3, length(t));
euler_angles_poisson = zeros(3, length(t));

for i = 1:length(t)
	[phi, theta, psi] = calc_euler_from_omat(reshape(squeeze(omat(:,:,i)), 3, 3));
	euler_angles(:,i) = [phi; theta; psi];
	% transpose because reshape interprets column wise but it is stored row wise
	[phi, theta, psi] = calc_euler_from_omat(reshape(omat_poisson(i,:), 3, 3));
	euler_angles_poisson(:,i) = [phi; theta; psi];
end

names = ["$\phi$", "$\theta$", "$\psi$"];
fig = figure;
sgtitle('HW1 P7(iii): Euler Angles comparison vs Time');

for i = 1:3
	subplot(3,1,i);
	hold on;
	plot(t, x(:,i), 'b', 'LineWidth', 2, 'DisplayName', names(i));
	plot(t, euler_angles(i,:), '--','Color', color_orange, 'LineWidth', 2, 'DisplayName', names(i) + " with $\Theta$");
	plot(t, euler_angles_poisson(i,:), '-.y', 'LineWidth', 2, 'DisplayName', names(i) + " with $\Theta$ (Poisson)");
	ylabel('Angle (rad)');
	xlabel('Time (s)');
	legend("Location", "best", "Interpreter", "latex");
	grid on; grid minor;
end

fig.Position(3:4) = [900, 800];
% saveas(fig, 'euler_angles_comparison.svg');

function ang_vel = calc_angvel(t)
ang_vel = [
	cos(2*t);
	cos(2*t);
	0.025*t;
	];
end

function xdot = dyn(t, x)
phi = x(1);
theta = x(2);
% psi = x(3);
S_inv = [
	1, sin(phi)*tan(theta), cos(phi)*tan(theta);
	0, cos(phi), -sin(phi);
	0, sin(phi)/cos(theta), cos(phi)/cos(theta);
	];
ang_vel = calc_angvel(t);
xdot = S_inv * ang_vel;
end

function omat = calc_omat(x)
a = x(1);
b = x(2);
c = x(3);
omat = [
	cos(b)*cos(c), cos(b)*sin(c), -sin(b);
	cos(c)*sin(a)*sin(b) - cos(a)*sin(c), cos(a)*cos(c) + sin(a)*sin(b)*sin(c), cos(b)*sin(a);
	sin(a)*sin(c) + cos(a)*cos(c)*sin(b), cos(a)*sin(b)*sin(c) - cos(c)*sin(a), cos(a)*cos(b);
	];
end

function omat_poisson = calc_omat_poisson(t, omat)
ang_vel_cross = crMat(calc_angvel(t));
omat = reshape(omat, 3, 3);
omat_poisson = -ang_vel_cross * omat;
omat_poisson = omat_poisson(:);
end

%% cross matrix function
function crossMat = crMat(X)
crossMat = [
	0, -X(3), X(2);
	X(3), 0, -X(1);
	-X(2), X(1), 0;
	];
end

function [phi, theta, psi] = calc_euler_from_omat(omat)
O23 = omat(2,3);
O33 = omat(3,3);
O13 = omat(1,3);
O12 = omat(1,2);
O11 = omat(1,1);

phi = atan2(O23, O33);
theta = -asin(O13);
psi = atan2(O12, O11);
end