

% 584 Final Project Fall 2024
% Problem 2: Planar Endoatmospheric Guidance
% Written by Syed Aseem Ul Islam (aseemisl@umich.edu) 17 Nov 2020.
% Finished by: Akshat Dubey 13 Dec 2024

clc; clear; close all;

time = 500; %seconds
Ts = 0.1;

%%%%%%%%%%%%% i %%%%%%%%%%%%%%%%%
% x(1)    :: hP
% x(2)    :: dP
% x(3)    :: gammaP
% x(4)    :: hE
% x(5)    :: dE
% x(6)    :: gammaE
% x(7)    :: R
% x(8)    :: beta

hP_0 = 10000; % Pursuer altitude, m
dP_0 = 0; % Pursuer downrange, m
gammaP_0 = 0; % Pursuer flight path angle, rad
hE_0 = 10000; % Evader altitude, m
dE_0 = 30000; % Evader downrange, m
gammaE_0 = pi; % Evader flight path angle, rad
R_0 = norm([hE_0 - hP_0, dE_0 - dP_0]); % Range, m
beta_0 = atan2(hE_0 - hP_0, dE_0 - dP_0); % LOS Bearing, rad

x0_kin = [hP_0, dP_0, gammaP_0, hE_0, dE_0, gammaE_0, R_0, beta_0];

x = zeros(8,time/Ts);
for ii = 1:time/Ts
	if ii == 1
		[~,xx] = ode45(@(t,x)kinsim(t,x), [ii ii+1]*Ts , x0_kin );
	else
		[~,xx] = ode45(@(t,x)kinsim(t,x), [ii ii+1]*Ts , x(:,ii-1) );
	end
	x(:,ii) = xx(end,:);
	
	%%% Intersample Fuzing %%%%%%%%
	[detonate , missDistance ] = fuzeKin( xx );
	if detonate
		fprintf('Miss Distance: %f\n',missDistance);
		break
	end
	%%% Intersample Fuzing %%%%%%%%
end

figure(1)
p1 = plot( x(2,1:ii)/1000 , x(1,1:ii)/1000  , 'b' );
hold on
plot( x(2,1)/1000  , x(1,1)/1000  , 'bx' )
plot( x(2,ii)/1000  , x(1,ii)/1000  , 'bo' )
p2 = plot( x(5,1:ii)/1000  , x(4,1:ii)/1000  , 'r' );
plot( x(5,1)/1000  , x(4,1)/1000  , 'rx' )
plot( x(5,ii)/1000  , x(4,ii)/1000  , 'ro' )
hold off
ylimits = ylim;
xlimits = xlim;
axis equal
ylim([0 ylimits(2)])
xlim([0 xlimits(2)])
legend([p1,p2],{'Pursuer','Evader'},'interpreter','latex')
ylabel('$h$ (km)','interpreter','latex')
xlabel('$d$ (km)','interpreter','latex')

%%
% %%%%%%%%%%%%% ii %%%%%%%%%%%%%%%%%
% x(1)    :: VP
% x(2)    :: gammaP
% x(3)    :: hP
% x(4)    :: dP
% x(5)    :: VE
% x(6)    :: gammaE
% x(7)    :: hE
% x(8)    :: dE

VP_0 = 450; % Pursuer velocity, m/s
VE_0 = 450; % Evader velocity, m/s

x0_dyn = [VP_0, gammaP_0, hP_0, dP_0, VE_0, gammaE_0, hE_0, dE_0];

x2 = zeros(8,time/Ts);
for ii = 1:time/Ts
	if ii == 1
		[~,xx]      = ode45(@(t,x)dynsim(t,x), [ii ii+1]*Ts , x0_dyn );
	else
		[~,xx]      = ode45(@(t,x)dynsim(t,x), [ii ii+1]*Ts , x2(:,ii-1) );
	end
	x2(:,ii) = xx(end,:);
	
	%%% Intersample Fuzing %%%%%%%%
	[detonate , missDistance ] = fuze( xx );
	if detonate
		fprintf('Miss Distance: %f\n',missDistance);
		break
	end
	%%% Intersample Fuzing %%%%%%%%
end

figure(2)
p1 = plot( x2(4,1:ii)/1000  , x2(3,1:ii)/1000  , 'b' );
hold on
plot( x2(4,1)/1000  , x2(3,1)/1000  , 'bx' )
plot( x2(4,ii)/1000  , x2(3,ii)/1000  , 'bo' )
p2 = plot( x2(8,1:ii)/1000  , x2(7,1:ii)/1000  , 'r' );
plot( x2(8,1)/1000  , x2(7,1)/1000  , 'rx' )
plot( x2(8,ii)/1000  , x2(7,ii)/1000  , 'ro' )
hold off
ylimits = ylim;
xlimits = xlim;
axis equal
ylim([0 ylimits(2)])
xlim([0 xlimits(2)])
legend([p1,p2],{'Pursuer','Evader'},'interpreter','latex')
ylabel('$h$ (km)','interpreter','latex')
xlabel('$d$ (km)','interpreter','latex')

figure(3)
plot( [1:ii]*Ts , x2(1,1:ii) , 'b' )
hold on
plot( [1:ii]*Ts , x2(5,1:ii) , 'r')
hold off
legend('$V_{\rm P}$','$V_{\rm E}$','interpreter','latex')
ylabel('$V$ (m/s)','interpreter','latex')
xlabel('$t$ (s)','interpreter','latex')
grid on
axis tight
ylimits = ylim;
xlimits = xlim;
ylim([0 ylimits(2)])
xlim([0 xlimits(2)])