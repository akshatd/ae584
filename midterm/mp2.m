clc; clear; close all;

r_L1_w = [1.52; 0; 0]; % Mars
r_L2_w = [0; 0; 0]; %  Sun

s1 = [0;1;0]; s2 = [0;0;1];

k = 1:50;

% load bearings and subtended angles
load("AE584_Midterm_P2.mat");

% bearingL2St1: bearing from L2 to S1, phi2_s1;
% bearingL2St2: bearing from L2 to S2, pih2_s2;
% subAngL1L2: subtended angle between L1 and L2, theta_L1L2;

r_0 = [0.52,0,-1]; % initial guess
Pk = zeros(3, length(k));
options = optimoptions('fminunc', 'OptimalityTolerance', 1e-12, 'Display', 'off');
for i=1:length(k)
	r = fminunc(@(x) costfn(x, s1, s2, r_L1_w, r_L2_w, subAngL1L2(i), bearingL2St1(i), bearingL2St2(i)), r_0, options);
	Pk(:,i) = r;
	r_0 = r;
end

%% plot
plot_components(Pk);
plot_3d(Pk, r_L1_w, r_L2_w);

%% functions
function cost = costfn(x, S1, S2, L1, L2, theta, psi1, psi2)
% setup
x = x'; % fix x's dims as used by fminunc
r_S1_Y = S1;
r_S2_Y = S2;
r_Y_L1 = x-L1;
r_L2_L1 = L2-L1;
r_L2_Y = L2-x;

% eq4
l4 = dot(r_Y_L1, r_L2_L1);
r4 = - norm(r_Y_L1) * norm(r_L2_Y) * cos(theta) + norm(r_Y_L1)^2;
err_subt_angle = l4-r4;

% eq5
l5 = dot(r_L2_Y, r_S1_Y);
r5 = norm(r_L2_Y) * cos(psi1);
err_bearing1 = l5-r5;

% eq6
l6 = dot(r_L2_Y, r_S2_Y);
r6 = norm(r_L2_Y) * cos(psi2);
err_bearing2 = l6-r6;

cost = err_subt_angle^2 + err_bearing1^2 + err_bearing2^2;
end

function plot_components(Pk)
fig = figure;
fig.Position(3:4) = [500, 1000];

sgtitle("P2: Components of $\vec{r}$ versus step k", 'Interpreter', 'latex');

subplot(3,1,1);
scatter(1:length(Pk), Pk(1,:), 'filled');
xlabel("k"); ylabel("$\vec{r}_x$", "Interpreter", "latex", "FontSize", 16);
grid on; grid minor;

subplot(3,1,2);
scatter(1:length(Pk), Pk(2,:), 'filled');
xlabel("k"); ylabel("$\vec{r}_y$", "Interpreter", "latex", "FontSize", 16);
grid on; grid minor;

subplot(3,1,3);
scatter(1:length(Pk), Pk(3,:), 'filled');
xlabel("k"); ylabel("$\vec{r}_z$", "Interpreter", "latex", "FontSize", 16);
grid on; grid minor;

end

function plot_3d(Pk, L1, L2)
figure;
s1 = scatter3(Pk(1,:), Pk(2,:), Pk(3,:), 'filled');
hold on;
s2 = scatter3(L1(1), L1(2), L1(3), 100, 'r', 'filled', 'MarkerEdgeColor', 'k');
s3 = scatter3(L2(1), L2(2), L2(3), 100, 'y', 'filled', 'MarkerEdgeColor', 'k');
hold off;
xlabel("x"); ylabel("y"); zlabel("z");
legend([s1, s2, s3], {'$\vec{r}$', 'Mars', 'Sun'}, 'Interpreter', 'latex');
grid on; grid minor;
title("P2: 3D Trajectory of spacecraft");
end
