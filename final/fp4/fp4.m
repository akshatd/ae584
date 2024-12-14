% 584 Final Project Fall 2020
% Problem 4
% Finished by: Akshat Dubey 13 Dec 2024

clc; clear; close all;

time = 50; % seconds
Ts = 0.1;

g = 9.81; % Gravity, m/s^2
umax = 40*g; %Saturation limit of nzP for optimization

% x(1)    :: VP
% x(2)    :: gammaP
% x(3)    :: hP
% x(4)    :: dP
% x(5)    :: VE
% x(6)    :: gammaE
% x(7)    :: hE
% x(8)    :: dE

x0_dyn = [ 450 ; 0 ; 10000 ; 0 ; 450 ; 0 ; 10000 ; 6500 ]; %initial condition for engagement simulation
x0_dyn = [ 450 ; 0 ; 10000 ; 0 ; 350 ; 0.1 ; 10000 ; 9000 ]; %initial condition for engagement simulation
x0_dyn = [ 450 ; 0 ; 10000 ; 0 ; 350 ; -0.1 ; 10000 ; 8000 ]; %initial condition for engagement simulation
x0_dyn = [ 450 ; 0 ; 8000 ; 0 ; 350 ; -0.1 ; 10000 ; 6000 ]; %initial condition for engagement simulation
x0_dyn = [ 450 ; 0 ; 4000 ; 0 ; 350 ; 0.2 ; 6000 ; 2000 ]; %initial condition for engagement simulation
x0_dyn = [ 450 ; 0 ; 10000 ; 0 ; 350 ; 0.2 ; 6000 ; 2000 ]; %initial condition for engagement simulation
x0_dyn = [ 450 ; 0 ; 10000 ; 0 ; 400 ; 0 ; 7000 ; 5000 ]; %initial condition for engagement simulation
x0_dyn = [ 450 ; 0 ; 10000 ; 0 ; 400 ; 0.0 ; 12000 ; 6000 ]; %initial condition for engagement simulation
x0_dyn = [ 450 ; 0 ; 9000 ; 0 ; 400 ; -0.1 ; 12000 ; 6000 ]; %initial condition for engagement simulation
x0_dyn = [ 450 ; 0 ; 9000 ; 0 ; 320 ; -0.4 ; 12000 ; 7000 ]; %initial condition for engagement simulation

nzE = -g; % Evader normal acceleration, m/s^2

% %%%%%%%%%%%%% i %%%%%%%%%%%%%%%%%
x1 = zeros(8,time/Ts);
nzP = zeros(1,time/Ts);
Rdot = zeros(1,time/Ts);

for ii = 1:time/Ts
	if ii == 1
		[~,xx] = ode45(@(t,x)dynsim(t,x,nzE,0), [ii ii+1]*Ts, x0_dyn);
	else
		[~,xx] = ode45(@(t,x)dynsim(t,x,nzE,nzP(ii-1)), [ii ii+1]*Ts, x1(:,ii-1));
	end
	x1(:,ii) = xx(end,:);
	y = output(x1(:,ii));
	
	%%% Proportinal Guidance %%%%%%%%%%
	nzP(:,ii) = -3*abs(y(2))*y(3) - g*cos(x1(2,ii)) ;
	%%% Proportinal Guidance %%%%%%%%%%
	
	%%% Custom Guidance %%%%%%%%%%
	% nzP(:,ii) = ;
	%%% Custom Guidance %%%%%%%%%%
	
	%%% Intersample Fuzing %%%%%%%%
	[detonate, missDistance ] = fuze(xx); %check for fuzing conditions in intersample xx
	if detonate
		fprintf('Miss Distance: %f\n',missDistance);
		break
	end
	%%% Intersample Fuzing %%%%%%%%
end

%%
% plot optimal guidance law
data_opt = load("OptimalGuidanceRunData.mat");
figure(1);
p1 = plot(data_opt.x2(4,1:data_opt.iio)/1000, data_opt.x2(3,1:data_opt.iio)/1000, 'b');
hold on;
plot(data_opt.x2(4,1)/1000, data_opt.x2(3,1)/1000, 'bx');
plot(data_opt.x2(4,data_opt.iio)/1000, data_opt.x2(3,data_opt.iio)/1000, 'bo');
p2 = plot(data_opt.x2(8,1:data_opt.iio)/1000, data_opt.x2(7,1:data_opt.iio)/1000, 'r');
plot(data_opt.x2(8,1)/1000, data_opt.x2(7,1)/1000, 'rx');
plot(data_opt.x2(8,data_opt.iio)/1000, data_opt.x2(7,data_opt.iio)/1000, 'ro');
hold off;
ylimits = ylim;
xlimits = xlim;
axis equal;
xlim(xlimits);
ylim([0 ylimits(2)]);
legend([p1,p2],{'Pursuer','Evader'},'interpreter','latex','Location','best');
ylabel('$h$ (km)','interpreter','latex');
xlabel('$d$ (km)','interpreter','latex');
grid on;
title('Final P4: Optimal Guidance Law');


fig = figure(2);
p1 = plot(x1(4,1:ii)/1000, x1(3,1:ii)/1000, 'b');
hold on;
plot(x1(4,1)/1000, x1(3,1)/1000, 'bx');
plot(x1(4,ii)/1000, x1(3,ii)/1000, 'bo');
p2 = plot(x1(8,1:ii)/1000, x1(7,1:ii)/1000, 'r');
plot(x1(8,1)/1000, x1(7,1)/1000, 'rx');
plot(x1(8,ii)/1000, x1(7,ii)/1000, 'ro');
hold off;
ylimits = ylim;
xlimits = xlim;
axis equal;
xlim(xlimits);
ylim([0 ylimits(2)]);
legend([p1,p2],{'Pursuer','Evader'},'interpreter','latex','Location','best');
ylabel('$h$ (km)','interpreter','latex');
xlabel('$d$ (km)','interpreter','latex');
grid on;
title('Final P4: Pursuer and Evader Trajectories');
saveas(fig, 'fp4_fig1.svg');
