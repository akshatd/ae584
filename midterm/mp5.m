clc; clear; close all;

% set up
T = 1;
A = [
	1, T;
	0, 1;
	];
b = [0;0];

mu = [0;0]; % E[X]
P = [
	0.5,0;
	0,0.5;
	];

% Y = AX + b, X = N(mu, P)

% Expectation: E[Y] = A*mu + b
E_Y = A*mu + b;
fprintf('E[Y]:\n');
disp(E_Y);

% Covariance: Cov[Y] = A*P*A'
Cov_Y = A*P*A';
fprintf('Cov[Y]:\n');
disp(Cov_Y);
