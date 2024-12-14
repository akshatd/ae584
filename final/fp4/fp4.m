% 584 Final Project Fall 2020
% Problem 4
% Finished by: Akshat Dubey 13 Dec 2024

clc; clear; close all;

%%
% plot optimal guidance law
data_opt = load("OptimalGuidanceRunData.mat");
fig = figure(1);
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
saveas(fig, "fp4_fig_opt.svg");

%% Custom guidance law
time = 50; % seconds
Ts = 0.1;

g = 9.81; % Gravity, m/s^2
umax = 40*g; % Saturation limit of nzP for optimization
nzE = -g; % Evader normal acceleration, m/s^2

% x(1)    :: VP
% x(2)    :: gammaP
% x(3)    :: hP
% x(4)    :: dP
% x(5)    :: VE
% x(6)    :: gammaE
% x(7)    :: hE
% x(8)    :: dE
x0_dyns = [ % initial conditions for engagement simulation
	[ 450 ; 0 ; 10000 ; 0 ; 450 ; 0 ; 10000 ; 6500 ] ...
	[ 450 ; 0 ; 10000 ; 0 ; 350 ; 0.1 ; 10000 ; 9000 ] ...
	[ 450 ; 0 ; 10000 ; 0 ; 350 ; -0.1 ; 10000 ; 8000 ] ...
	[ 450 ; 0 ; 8000 ; 0 ; 350 ; -0.1 ; 10000 ; 6000 ] ...
	[ 450 ; 0 ; 4000 ; 0 ; 350 ; 0.2 ; 6000 ; 2000 ] ...
	[ 450 ; 0 ; 10000 ; 0 ; 350 ; 0.2 ; 6000 ; 2000 ] ...
	[ 450 ; 0 ; 10000 ; 0 ; 400 ; 0 ; 7000 ; 5000 ] ...
	[ 450 ; 0 ; 10000 ; 0 ; 400 ; 0.0 ; 12000 ; 6000 ] ...
	[ 450 ; 0 ; 9000 ; 0 ; 400 ; -0.1 ; 12000 ; 6000 ] ...
	[ 450 ; 0 ; 9000 ; 0 ; 320 ; -0.4 ; 12000 ; 7000 ]
	];


% %%%%%%%%%%%%% i %%%%%%%%%%%%%%%%%
missDistances = zeros(size(x0_dyns, 2), 1);
fig = figure(2);
sgtitle("Final P4: Pursuer and Evader Trajectories");

loft_coeff = 5; % lofting factor
Tloft_coeff = 2; % lofting time coefficient
hdiff_offset = 1000; % offset for hdiff so it lofts even when missile is higher than evader, m
for s=1:size(x0_dyns, 2)
	TLoft = 0; % seconds to loft
	x1 = zeros(8,time/Ts);
	nzP = zeros(1,time/Ts);
	Rdot = zeros(1,time/Ts);
	x0_dyn = x0_dyns(:,s);
	for ii = 1:time/Ts
		if ii == 1
			[~,xx] = ode45(@(t,x)dynsim(t,x,nzE,0), [ii ii+1]*Ts, x0_dyn);
		else
			[~,xx] = ode45(@(t,x)dynsim(t,x,nzE,nzP(ii-1)), [ii ii+1]*Ts, x1(:,ii-1));
		end
		x1(:,ii) = xx(end,:);
		y = output(x1(:,ii));
		
		% available data
		VP = x1(1,ii); % Pursuer velocity, m/s
		gammaP = x1(2,ii); % Pursuer flight path angle, rad
		hP = x1(3,ii); % Pursuer altitude, m
		dP = x1(4,ii); % Pursuer downrange, m
		% VE = x1(5,ii); % Evader velocity, m/s
		% gammaE = x1(6,ii); % Evader flight path angle, rad
		% hE = x1(7,ii); % Evader altitude, m
		% dE = x1(8,ii); % Evader downrange, m
		R = y(1);
		R_dot = y(2);
		beta_dot = y(3);
		hE_0 = x0_dyn(7); % Initial evader altitude, m
		dE_0 = x0_dyn(8); % Initial evader downrange, m
		
		%%% Proportinal Guidance %%%%%%%%%%
		% nzP(:,ii) = -3*abs(R_dot)*beta_dot - g*cos(gammaP);
		%%% Proportinal Guidance %%%%%%%%%%
		
		%%% Custom Guidance %%%%%%%%%%
		% how many seconds to loft
		if ii == 1
			hdiff = hE_0 - hP + hdiff_offset;
			if hdiff > 0
				TLoft = Tloft_coeff*hdiff/1000;
			end
		end
		
		if ii * Ts < TLoft
			nzP(:,ii) =  -loft_coeff*g - g*cos(gammaP);
		else
			nzP(:,ii) = -3*abs(R_dot)*beta_dot - g*cos(gammaP);
		end
		%%% Custom Guidance %%%%%%%%%%
		
		%%% Intersample Fuzing %%%%%%%%
		[detonate, missDistance ] = fuze(xx); %check for fuzing conditions in intersample xx
		if detonate
			fprintf('Engagement %d Miss Distance: %f\n', s, missDistance);
			break
		end
		%%% Intersample Fuzing %%%%%%%%
	end
	
	missDistances(s) = missDistance;
	
	subplot(5, 2, s);
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
	title("Engagement " + s);
end
%%
fig.Position(3:4) = [1000 1200];
saveas(fig, "fp4_fig1.svg");

for s=1:size(x0_dyns, 2)
	fprintf('Miss Distance for Engagement %d: %f\n', s, missDistances(s));
end
