clc; clear; close all;

% part c
L1 = [0, 0]; L2 = [4, 2];
theta1 = -165; theta2 = 150;
T1 = T(theta1); T2 = T(theta2);

A = [T1, -1;
	T2, -1];
B = [T1*L1(1) - L1(2);
	T2*L2(1) - L2(2)];

P = A\B;

% plot part c
figure;
hold on;
scatter(P(1), P(2), 35, 'r', 'filled', 'DisplayName', 'P');
scatter(L1(1), L1(2), 35, 'g', 'filled', 'DisplayName', 'L1');
scatter(L2(1), L2(2), 35, 'g', 'filled', 'DisplayName', 'L2');
xlim([-1, 6.5]); ylim([-1, 6.5]);
xlabel('x'); ylabel('y');
grid on; grid minor;
legend('Location', 'best');
title('HW3 P1c: Locations of P, L1, and L2');

% part d
L1 = [0, 0]; L2 = [4, 2]; L3 = [1, 4];
theta1 = -140; theta2 = 90; theta3 = -30;
T1 = T(theta1); T2 = T(theta2); T3 = T(theta3);

A12 = [T1, -1;
	T2, -1];
B12 = [T1*L1(1) - L1(2);
	T2*L2(1) - L2(2)];
P12 = A12\B12;

A23 = [T2, -1;
	T3, -1];
B23 = [T2*L2(1) - L2(2);
	T3*L3(1) - L3(2)];
P23 = A23\B23;

A31 = [T3, -1;
	T1, -1];
B31 = [T3*L3(1) - L3(2);
	T1*L1(1) - L1(2)];
P31 = A31\B31;

P = (P12 + P23 + P31)/3;

% plot part d
figure;
hold on;
scatter(P(1), P(2), 35, 'r', 'filled', 'DisplayName', 'P');
scatter(L1(1), L1(2), 35, 'g', 'filled', 'DisplayName', 'L1');
scatter(L2(1), L2(2), 35, 'g', 'filled', 'DisplayName', 'L2');
scatter(L3(1), L3(2), 35, 'g', 'filled', 'DisplayName', 'L3');
scatter(P12(1), P12(2), 35, 'b', 'filled', 'DisplayName', 'P12');
scatter(P23(1), P23(2), 35, 'b', 'filled', 'DisplayName', 'P23');
scatter(P31(1), P31(2), 35, 'b', 'filled', 'DisplayName', 'P31');
xlim([-1, 4.5]); ylim([-1, 4.5]);
xlabel('x'); ylabel('y');
grid on; grid minor;
legend('Location', 'best');
title('HW3 P1d: Locations of P, L1, L2, and L3');


function res = T(theta)
res = tan(pi/2 - deg2rad(theta));
end