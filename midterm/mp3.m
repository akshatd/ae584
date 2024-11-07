clc; clear; close all;

% setup sim
Tfid = 0.01;
Tsim = 10;

% plot angular velocity over time for B_I
angvels = angvel_B_I(0:Tfid:Tsim);
plot_angvel(0:Tfid:Tsim, angvels, 'Angular Velocity $B|I_{|B}$ vs Time');

% propagate orientation matrix with angular velocity for B_I
omat_BI_0 = eye(3); % start with identity matrix
omat_BI_0 = omat_BI_0(:); % vectorize the matrix
[t, omat_BI] = ode45(@(t,x)omat_dot(t,x,@angvel_B_I), 0:Tfid:Tsim, omat_BI_0);
plot_omat(t, omat_BI, '$O_{B|I}$ vs Time');

% propagate euler angles with angular velocity for E_I
theta0 = [0; pi/6; 0]; % psi, theta, phi
[t, eul_ang_EI] = ode45(@(t,x)prop_theta(t,x), 0:Tfid:Tsim, theta0);
plot_euler_angles(t, eul_ang_EI, 'Euler Angles $E|I_{|E}$ vs Time');

% convert euler angles to orientation matrix E_I
omat_EI = zeros(3,3,length(t));
omat_EI_flat = zeros(length(t), 9);
for i=1:length(t)
	omat = omat313(eul_ang_EI(i,:));
	omat_EI(:,:,i) = omat;
	omat_EI_flat(i, :) = reshape(omat', 1, 9);
end
plot_omat(t, omat_EI_flat, '$O_{E|I}$ vs Time');

% get orientation matrix for B|E
omat_BE = zeros(3,3,length(t));
omat_BE_flat = zeros(length(t), 9);
for i=1:length(t)
	omat_BI_mat = reshape(omat_BI(i,:), 3, 3)';
	omat = omat_EI(:,:,i) \ omat_BI_mat;
	omat_BE(:,:,i) = omat;
	omat_BE_flat(i, :) = reshape(omat', 1, 9);
end
plot_omat(t, omat_BE_flat, 'P3: $O_{B|E}$ vs Time');

% get euler angles for B|E
eul_ang_BE = zeros(length(t), 3);
for i=1:length(t)
	[psi, theta, phi] = euler_from_omat321(omat_BE(:,:,i));
	eul_ang_BE(i,:) = [psi, theta, phi];
end
plot_euler_angles(t, eul_ang_BE, 'P3: Euler Angles vs Time');


% angular velocity for B|I
function angvel = angvel_B_I(t)
angvel = [
	cos(2*t);
	cos(2*t);
	0.025*t;
	];
end

% angular velocity for E|I
function angvel = angvel_E_I(t)
angvel = [
	sin(0.05*t);
	0.3 * cos(0.01*t);
	0.5 * sin(0.01*t);
	];
end

%% cross matrix function
function crossMat = crMat(X)
crossMat = [
	0      -X(3)  X(2);
	X(3)    0    -X(1);
	-X(2)  X(1)   0;
	];
end

%% Orientation matrix ODE
function xdot = omat_dot(t,x,omega_d)
xT = reshape(x,3,3);
xdT = -crMat(omega_d(t))*xT;
xdot = xdT(:);
end

function theta_dot = prop_theta(t, x)
psi = x(1);
theta = x(2);
% phi = x(2);
S_inv = [
	-sin(psi)*cot(theta), -cos(psi)*cot(theta), 1;
	cos(psi), -sin(psi), 0;
	sin(psi)*csc(theta), cos(psi)*csc(theta), 0;
	];
ang_vel = angvel_E_I(t);
theta_dot = S_inv * ang_vel;
end

function omat = omat1(x)
omat = [
	1, 0, 0;
	0, cos(x), sin(x);
	0, -sin(x), cos(x);
	];
end

function omat = omat3(x)
omat = [
	cos(x), sin(x), 0;
	-sin(x), cos(x), 0;
	0, 0, 1;
	];
end

function omat = omat313(x)
psi = x(1);
theta = x(2);
phi = x(3);
omat = omat3(psi) * omat1(theta) * omat3(phi);
end

function [psi, theta, phi] = euler_from_omat321(omat)
O23 = omat(2,3);
O33 = omat(3,3);
O13 = omat(1,3);
O12 = omat(1,2);
O11 = omat(1,1);

phi = atan2(O23, O33);
theta = -asin(O13);
psi = atan2(O12, O11);
end

function plot_angvel(t, angvels, title_str)
figure;
hold on;
plot(t, angvels(1,:), 'r', 'LineWidth', 2, 'DisplayName', '$\omega_i$');
plot(t, angvels(2,:), '--g', 'LineWidth', 2, 'DisplayName', '$\omega_j$');
plot(t, angvels(3,:), 'b', 'LineWidth', 2, 'DisplayName', '$\omega_k$');
ylabel('Angular Velocity (rad/s)');
title(title_str, 'Interpreter', 'latex');
xlabel('Time (s)');
legend("Location", "best", "Interpreter", "latex");
grid on; grid minor;
end


function plot_euler_angles(t, theta, title_str)
fig = figure;
fig.Position(3:4) = [500, 1000];
sgtitle(title_str, 'Interpreter', 'latex');

subplot(3,1,1);
plot(t, theta(:,1), '-', 'LineWidth', 2);
ylabel('$\psi$', 'Interpreter', 'latex', 'FontSize', 16);
grid on; grid minor;

subplot(3,1,2);
plot(t, theta(:,2), '-', 'LineWidth', 2);
ylabel('$\theta$', 'Interpreter', 'latex', 'FontSize', 16);
grid on; grid minor;

subplot(3,1,3);
plot(t, theta(:,3), '-', 'LineWidth', 2);
ylabel('$\phi$', 'Interpreter', 'latex', 'FontSize', 16);
xlabel('Time (s)');
grid on; grid minor;

end


function plot_omat(t, omat, title_str)
omat_11 = omat(:,1);
omat_12 = omat(:,4);
omat_13 = omat(:,7);
omat_21 = omat(:,2);
omat_22 = omat(:,5);
omat_23 = omat(:,8);
omat_31 = omat(:,3);
omat_32 = omat(:,6);
omat_33 = omat(:,9);

fig = figure;
fig.Position(3:4) = [900, 800];
sgtitle(title_str, 'Interpreter', 'latex');

subplot(3,3,1)
plot(t,omat_11,'linewidth',2)
hold off
set(gca,'FontSize',12)
set(gca,'TickLabelInterpreter','latex');
ylabel('Row 1','Interpreter','latex')
title('Column 1','Interpreter','latex')
grid on

subplot(3,3,2)
plot(t,omat_12,'linewidth',2)
hold off
set(gca,'FontSize',12)
set(gca,'TickLabelInterpreter','latex');
title('Column 2','Interpreter','latex')
grid on

subplot(3,3,3)
plot(t,omat_13,'linewidth',2)
hold off
set(gca,'FontSize',12)
set(gca,'TickLabelInterpreter','latex');
title('Column 3','Interpreter','latex')
grid on

subplot(3,3,4)
plot(t,omat_21,'linewidth',2)
hold off
set(gca,'FontSize',12)
set(gca,'TickLabelInterpreter','latex');
ylabel('Row 2','Interpreter','latex')
grid on

subplot(3,3,5)
plot(t,omat_22,'linewidth',2)
hold off
set(gca,'FontSize',12)
set(gca,'TickLabelInterpreter','latex');
grid on

subplot(3,3,6)
plot(t,omat_23,'linewidth',2)
hold off
set(gca,'FontSize',12)
set(gca,'TickLabelInterpreter','latex');
grid on

subplot(3,3,7)
plot(t,omat_31,'linewidth',2)
hold off
set(gca,'FontSize',12)
set(gca,'TickLabelInterpreter','latex');
xlabel('$t$ (s)','Interpreter','latex')
ylabel('Row 3','Interpreter','latex')
grid on

subplot(3,3,8)
plot(t,omat_32,'linewidth',2)
hold off
set(gca,'FontSize',12)
set(gca,'TickLabelInterpreter','latex');
xlabel('$t$ (s)','Interpreter','latex')
grid on

subplot(3,3,9)
plot(t,omat_33,'linewidth',2)
hold off
set(gca,'FontSize',12)
set(gca,'TickLabelInterpreter','latex');
xlabel('$t$ (s)','Interpreter','latex')
grid on

end
