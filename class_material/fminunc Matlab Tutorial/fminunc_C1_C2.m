close all
clear all
clc

%% Optimization setup
x0 = 3;
y0 = 0;
%y0 = 2;
mu = 0.5;

global h;
h.x = [];
h.fval = [];

F = @(x) costFun(x, x0, y0, mu);
options = optimoptions(@fminunc,'OptimalityTolerance',1e-12,'OutputFcn',@outfun);
xinit = -10;

%% Optimization function
[xmin, fval] = fminunc(F, xinit,options);

%% Plotting solutions
xr =1.5:0.01:4.5;

figure(1)
plot(xr, F(xr), 'b', 'linewidth', 2)
hold on
scatter(xmin,F(xmin), 50,'r','fill')
hold off
xlim([1.5 4.5])
set(gca,'FontSize',15)
set(gca,'TickLabelInterpreter','latex');
xlabel('$x$','fontsize',18,'interpreter','latex')
ylabel('$F(x)$','fontsize',18,'interpreter','latex')
legend({'$F(x), x \in [-1.5, 4.5]$','$(x_{\rm min}, F(x_{\rm min}))$'},'fontsize',16,'interpreter','latex')
grid on

figure(2)
iter = 1:length(h.x);
semilogy(iter, abs(h.x-x0),'b','linewidth',2)
xlim([min(iter) max(iter)])
xticks(1:2:max(iter))
%xticks(1:max(iter))
set(gca,'FontSize',12)
set(gca,'TickLabelInterpreter','latex');
xlabel('Iteration $i$','fontsize',18,'interpreter','latex')
ylabel('$\log_{10} (|x_{{\rm iter},i} - x_0|)$','fontsize',18,'interpreter','latex')
grid on

%%
function stop = outfun(x,optimValues,state)
     global h
     stop = false;
     switch state
         case 'iter'
             h.fval = [h.fval; optimValues.fval];
             h.x = [h.x; x];
         otherwise
     end
 end