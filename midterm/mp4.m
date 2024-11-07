clc; clear; close all;

% consts
g = 9.80665; % m/s^2
alpha  = pi/4; % rad

% setup sim
Tfid = 0.01;
Tsim = 20;

% propagate orientation matrix with angular velocity for B_A
r_cw_0 = [1;0;0]; % m
rdot_cw_0 = [0;cos(alpha);sin(alpha)]; % m/s
omat_BA_0 = [
	1, 0,           0;
	0, cos(alpha),  sin(alpha);
	0, -sin(alpha), cos(alpha);
	];
display(omat_BA_0);
omat_BA_0_flat = reshape(omat_BA_0', 9, 1); % transpose so its rowwise
display(omat_BA_0_flat);
x0 = [ % extended state
	r_cw_0;
	rdot_cw_0;
	omat_BA_0_flat;
	];
[t, omat_BA] = ode45(@(t,x)omat_dot(t,x,@omega_BA), 0:Tfid:Tsim, omat_BA_0);
plot_omat(t, omat_BA, '$O_{B|A}$ vs Time');

% test = dynamics(0.1, x0, g, alpha);


function omega = omega_BA(~)
omega = [0;0;1];
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

function acc = get_acc(g, alpha, t)
acc = [
	-1-g*sin(alpha)*sin(t);
	-g*sin(alpha)*cos(t);
	-g*cos(alpha);
	];
end

% function xdot = dynamics(t, x, g, alpha)
% r_cw = x(1:3);
% rdot_cw = x(4:6);
% omat_BA = reshape(x(7:15), 3, 3)';

% % get omat_BA_dot
% omat_BA_dot = omat_dot(t, omat_BA, @omega_BA);

% acc_cw_B =

% end

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
