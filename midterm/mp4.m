clc; clear; close all;

% consts
g = 9.80665; % m/s^2
alpha  = pi/4; % rad

% setup sim
Tfid = 0.01;
Tsim = 20;

% propagate orientation matrix with angular velocity for B_A
r_0 = [1;0;0]; % m
rdot_0 = [0;cos(alpha);sin(alpha)]; % m/s
omat_BA_0 = [
	1, 0,           0;
	0, cos(alpha),  sin(alpha);
	0, -sin(alpha), cos(alpha);
	];
% display(omat_BA_0);
omat_BA_0 = omat_BA_0(:); % plot and omat_dot expects it columnwise as in example_posson_equation.m
% display(omat_BA_0);
x0 = [ % extended state
	r_0;
	rdot_0;
	omat_BA_0;
	];

% [t, x] = ode45(@(t,x) dynamics1(t, x), 0:Tfid:Tsim, x0);
[t, x] = ode45(@(t,x) dynamics(t, x, g, alpha), 0:Tfid:Tsim, x0);

r = x(:,1:3);
plot_traj(r);
omat_BA = x(:,7:15);
plot_omat(t, omat_BA, '$O_{B|A}$ vs Time');
% test = dynamics(0.1, x0, g, alpha);


function omega = get_omega_BA(~)
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

function xdot = dynamics(t, x, g, alpha)
% extract states from x
r_A = x(1:3);
rdot_A = x(4:6);
omat_BA = x(7:15); % flat here cuz omat_dot expects it flat, columnwise
omat_BA_dot = omat_dot(t, omat_BA, @get_omega_BA); % calculate first for returning

% get rdot_B from rdot_A and omat_BA (transport theorem)
omega_BA = get_omega_BA(t);
omat_BA = reshape(omat_BA, 3, 3); % reshape does columnwise by default
rdot_B = rdot_A - crMat(omega_BA)*r_A;

% get everything else for double transport
rddot_B = get_acc(g, alpha, t) - omat_BA*[0;0;-g];
omega_BA = get_omega_BA(t);
omega_BA_cross = crMat(omega_BA);
rddot_A = rddot_B + 2*omega_BA_cross*rdot_B + omega_BA_cross*(omega_BA_cross*r_A);

xdot = [
	rdot_A;
	rddot_A;
	omat_BA_dot;
	];
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


function plot_traj(r)
figure;
plot3(r(:,1), r(:,2), r(:,3), 'linewidth', 2);
xlabel('x');
ylabel('y');
zlabel('z');
title('Trajectory of center of mass of quadcopter');
grid on;
end