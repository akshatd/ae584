close all
clear all
clc

%% Optimization setup
x0 = 3;
y0 = -2;
mu = 0.5;

F = @(x) costFun(x, x0, y0, mu);
options = optimoptions(@fminunc,'OptimalityTolerance',1e-12);
xinit1 = -10;
xinit2 = 10;

%% Optimization function
[xmin1, fval1] = fminunc(F, xinit1,options);
[xmin2, fval2] = fminunc(F, xinit2,options);

%% Optimization over range
xinit = -7:2:13;
xmin = zeros(1,length(xinit));

for ii = 1:length(xinit)
    xmin(ii) = fminunc(F, xinit(ii),options);
end

xinit_C1 = abs(xmin-xmin1);
xinit_C2 = abs(xmin-xmin2);

xR = xinit(xinit_C1 <= xinit_C2);
xB = xinit(xinit_C1 > xinit_C2);

%% Plotting solutions
xr = -1:0.01:7;

figure(1)
plot(xr, F(xr), 'b', 'linewidth', 2)
hold on
scatter(xmin1,F(xmin1), 50,'r','fill')
scatter(xmin2,F(xmin2), 50,[0, 0.5, 0],'fill')
hold off
xlim([-1 7])
set(gca,'FontSize',15)
set(gca,'TickLabelInterpreter','latex');
xlabel('$x$','fontsize',18,'interpreter','latex')
ylabel('$F(x)$','fontsize',18,'interpreter','latex')
legend({'$F(x), x \in [-1, 7]$','$(x_{\rm min}, F(x_{\rm min}))$ with $x_{\rm init} = -10$', '$(x_{\rm min}, F(x_{\rm min}))$ with $x_{\rm init} = 10$'},'fontsize',16,'interpreter','latex')
grid on

figure(2)

scatter(xR, zeros(1,length(xR)),35,'r','fill')
hold on
scatter(xB, zeros(1,length(xB)),35,'b','fill')
hold off
xlim([min(xinit) max(xinit)])
ylim([-0.5 0.5])
set(gca,'FontSize',12)
set(gca,'TickLabelInterpreter','latex');
xlabel('$x_{\rm init}$','fontsize',18,'interpreter','latex')
yticks([])
xticks(xinit)
legend({'$x_{\rm min} = 1$','$x_{\rm min} = 5$'},'fontsize',16,'interpreter','latex')
grid on