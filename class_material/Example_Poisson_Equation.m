%% Problem 7
close all
clear all
clc

%% Find Orientation Matrix Using Poisson Equation
omega_d = @(t) [cos(t); sin(t); 0];           % angular velocity
tspan = 0:0.01:10;                            % time step
Omat_0 = eye(3);                              % initial condition
Omat_0 = Omat_0(:);                           % vectorize the matrix

[tout,Omat_out] = ode45(@(t,x)Omat_d(t,x,omega_d), tspan, Omat_0);

Omat_out_11 = Omat_out(:,1);
Omat_out_12 = Omat_out(:,4);
Omat_out_13 = Omat_out(:,7);
Omat_out_21 = Omat_out(:,2);
Omat_out_22 = Omat_out(:,5);
Omat_out_23 = Omat_out(:,8);
Omat_out_31 = Omat_out(:,3);
Omat_out_32 = Omat_out(:,6);
Omat_out_33 = Omat_out(:,9);

%% plot
figure(4)
subplot(3,3,1)
plot(tout,Omat_out_11,'linewidth',2)
hold off
set(gca,'FontSize',12)
set(gca,'TickLabelInterpreter','latex');
ylabel('Row 1','interpreter','latex')
title('Column 1','interpreter','latex')
grid on

subplot(3,3,2)
plot(tout,Omat_out_12,'linewidth',2)
hold off
set(gca,'FontSize',12)
set(gca,'TickLabelInterpreter','latex');
title('Column 2','interpreter','latex')
grid on

subplot(3,3,3)
plot(tout,Omat_out_13,'linewidth',2)
hold off
set(gca,'FontSize',12)
set(gca,'TickLabelInterpreter','latex');
title('Column 3','interpreter','latex')
grid on

subplot(3,3,4)
plot(tout,Omat_out_21,'linewidth',2)
hold off
set(gca,'FontSize',12)
set(gca,'TickLabelInterpreter','latex');
ylabel('Row 2','interpreter','latex')
grid on

subplot(3,3,5)
plot(tout,Omat_out_22,'linewidth',2)
hold off
set(gca,'FontSize',12)
set(gca,'TickLabelInterpreter','latex');
grid on

subplot(3,3,6)
plot(tout,Omat_out_23,'linewidth',2)
hold off
set(gca,'FontSize',12)
set(gca,'TickLabelInterpreter','latex');
grid on

subplot(3,3,7)
plot(tout,Omat_out_31,'linewidth',2)
hold off
set(gca,'FontSize',12)
set(gca,'TickLabelInterpreter','latex');
xlabel('$t$ (s)','interpreter','latex')
ylabel('Row 3','interpreter','latex')
grid on

subplot(3,3,8)
plot(tout,Omat_out_32,'linewidth',2)
hold off
set(gca,'FontSize',12)
set(gca,'TickLabelInterpreter','latex');
xlabel('$t$ (s)','interpreter','latex')
grid on

subplot(3,3,9)
plot(tout,Omat_out_33,'linewidth',2)
hold off
set(gca,'FontSize',12)
set(gca,'TickLabelInterpreter','latex');
xlabel('$t$ (s)','interpreter','latex')
grid on


%% ODE
function xd = Omat_d(t,x,omega_d)
    xT = reshape(x,3,3);
    xdT = -crMat(omega_d(t))*xT;
    xd = xdT(:);
end

%% cross matrix function
function crossMat = crMat(X)
   crossMat = [0      -X(3)  X(2);
               X(3)    0    -X(1);
               -X(2)  X(1)   0];
end