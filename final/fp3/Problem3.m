% 584 Final Project Fall 2020
% Problem 3
% Written by Syed Aseem Ul Islam (aseemisl@umich.edu) 17 Nov 2020.

clear all
clc

time = 400; %seconds
Ts = 0.1;

% %%%%%%%%%%%%% i %%%%%%%%%%%%%%%%%
% x(1)    :: VP
% x(2)    :: gammaP
% x(3)    :: hP
% x(4)    :: dP
% x(5)    :: VE
% x(6)    :: gammaE
% x(7)    :: hE
% x(8)    :: dE

x0_dyn = ;
g = 9.81; %m/s^2

x = zeros(8,time/Ts);
for ii = 1:time/Ts
    
    % Evader normal acceleration
    nzE = ; %m/s^2
    
    
    
    if ii == 1
        [~,xx]      = ode45(@(t,x)dynsim(t,x,nzE), [ii ii+1]*Ts , x0_dyn );
    else
        [~,xx]      = ode45(@(t,x)dynsim(t,x,nzE), [ii ii+1]*Ts , x(:,ii-1) );
    end
    x(:,ii) = xx(end,:);
    
    

    %%% Intersample Fuzing %%%%%%%%
    [detonate , missDistance ] = fuze( xx );
    if detonate
        missDistance
        break
    end
    %%% Intersample Fuzing %%%%%%%%
    
    
end



figure(1)
p1 = plot( x(4,1:ii)/1000  , x(3,1:ii)/1000  , 'b' );
hold on
plot( x(4,1)/1000  , x(3,1)/1000  , 'bx' )
plot( x(4,ii)/1000  , x(3,ii)/1000  , 'bo' )
p2 = plot( x(8,1:ii)/1000  , x(7,1:ii)/1000  , 'r' );
plot( x(8,1)/1000  , x(7,1)/1000  , 'rx' )
plot( x(8,ii)/1000  , x(7,ii)/1000  , 'ro' )
hold off
ylimits = ylim;
xlimits = xlim;
axis equal
xlim(xlimits)
ylim([0 ylimits(2)])
legend([p1,p2],{'Pursuer','Evader'},'interpreter','latex')
ylabel('$h$ (km)','interpreter','latex')
xlabel('$d$ (km)','interpreter','latex')
grid on

figure(2)
plot( [1:ii]*Ts , x(1,1:ii) , 'b' )
hold on
plot( [1:ii]*Ts , x(5,1:ii) , 'r')
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

x0_dyn = ;
g = 9.81; %m/s^2
Rtemp = [];
x2 = zeros(8,time/Ts);
for ii = 1:time/Ts
    
    % Evader normal acceleration
    nzE = ; %m/s^2
    
    
    if ii == 1
        [~,xx]      = ode45(@(t,x)dynsim(t,x,nzE), [ii ii+1]*Ts , x0_dyn );
    else
        [~,xx]      = ode45(@(t,x)dynsim(t,x,nzE), [ii ii+1]*Ts , x2(:,ii-1) );
    end
    x2(:,ii) = xx(end,:);
    
    

    %%% Intersample Fuzing %%%%%%%%
    [detonate , missDistance ] = fuze( xx );
    if detonate
        missDistance
        break
    end
    %%% Intersample Fuzing %%%%%%%%
    
end


figure(3)
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
% xlim([17.5 18.5])
% ylim([10.7 11])
grid on

figure(4)
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