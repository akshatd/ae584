clc
clear all
close all

%3D circumference

ang = 0:0.001*pi:2*pi;

x = 2.*cos(ang);
y = 2.*sin(ang);
z = sin(ang)+cos(ang);

%Center of the circumference

center = [0;0;0];

%3D plot

figure(1)
scatter3(x,y,z,50,'b','fill')
hold on
scatter3(center(1), center(2), center(3),50,'r','fill')
hold off
zlabel('$z$ (m)','interpreter','latex','fontsize',15)
ylabel('$y$ (m)','interpreter','latex','fontsize',15)
xlabel('$x$ (m)','interpreter','latex','fontsize',15)

%2D projections

figure(2)
subplot(2,2,1)
scatter(x,z,50,'b','fill')
hold on
scatter(center(1), center(3),50,'r','fill')
hold off
grid on
ylabel('$z$ (m)','interpreter','latex','fontsize',15)
%xlabel('$x$ (m)','interpreter','latex','fontsize',15)
subplot(2,2,3)
scatter(x,y,50,'b','fill')
hold on
scatter(center(1), center(2),50,'r','fill')
hold off
grid on
ylabel('$y$ (m)','interpreter','latex','fontsize',15)
xlabel('$x$ (m)','interpreter','latex','fontsize',15)
subplot(2,2,4)
scatter(z,y,50,'b','fill')
hold on
scatter(center(3), center(2),50,'r','fill')
hold off
%ylabel('$y$ (m)','interpreter','latex','fontsize',15)
xlabel('$z$ (m)','interpreter','latex','fontsize',15)
grid on