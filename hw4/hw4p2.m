clc; clear; close all;

% consts
g = 9.80665; % m/s^2
phi = pi/6; % rad
g_A = [0;0;-g]; % m/s^2
T = 0.01; % s, sampling interval
D = diag([0.1,0.1,0.1]); % rad^2/s^2, noise covariance matrix
noise = @() D*mvnrnd(zeros(3,1), eye(3))'; % noise function

% sensors
omega_k_noisy = @() [0;0;1] + noise(); % rad/s
a_k_noisy = @(k) [
	-1-g*sin(phi)*sin(k*T);
	-g*sin(phi)*cos(k*T);
	-g*cos(phi);
	] + noise();

% inital conditions
r_0 = [1;0;0]; % initial orientation
rdot_0 = [0;cos(phi);sin(phi)]; % initial angular velocity
omat_BA_0 = [
	1, 0,           0;
	0, cos(phi),  sin(phi);
	0, -sin(phi), cos(phi);
	];

ks = 2000;
x_0 = [r_0; rdot_0]; % pos + accel state
y_0 = omat_BA_0; % as per lecture 16 slide 5, gyro state

%% part a, just predict
data.x = zeros(6,ks+1);
data.x(:,1) = x_0;
x_k = x_0;
y_k = y_0;

for k=1:ks
	% get sensor data
	omega_k = omega_k_noisy();
	a_k = a_k_noisy(k);
	
	% get dynamics
	[Ad, Bd, OMEGAd] = get_discrete_sys(T, omega_k);
	
	% propagate gyro
	y_kp1 = OMEGAd*y_k;
	% propagate accelerometer
	x_kp1 = Ad*x_k + Bd*(y_k'*a_k - g_A);
	
	% store data
	data.x(:,k+1) = x_kp1;
	
	% prep for next iteration
	x_k = x_kp1;
	y_k = y_kp1;
end

load('rcwA.mat');
plot_trajectory(data.x, rcwA_Ts_0_01, 'Quadcopter trajectory');


%% part b, predict and update

D3 = diag([0.005, 0.005, 0.005]);
r_noisy = zeros(3,ks+1);
for i=1:ks+1
	r_noisy(:,i) = rcwA_Ts_0_01(:, i) + D3*mvnrnd(zeros(3,1), eye(3))';
end
R = 0.001*eye(3); Q = 10*eye(6);
P_0 = 10*eye(6);

data.x_01 = zeros(6,ks+1);
data.x_01(:,1) = x_0;
data.x_1 = zeros(6,ks+1);
data.x_1(:,1) = x_0;
x_k = x_0;
y_k = y_0;
P_k = P_0;
C = [eye(3), zeros(3)];
for k=1:ks
	% get sensor data
	omega_k = omega_k_noisy();
	a_k = a_k_noisy(k);
	r_k = r_noisy(:,k);
	
	% get dynamics
	[Ad, Bd, OMEGAd] = get_discrete_sys(T, omega_k);
	
	% predict
	y_kp1 = OMEGAd*y_k;
	x_kp1_p = Ad*x_k + Bd*(y_k'*a_k - g_A);
	
	% covariance stuff
	P_kp1_p = Ad*P_k*Ad' + Q;
	K_k = P_kp1_p*C'/(C*P_kp1_p*C' + R);
	P_k = P_kp1_p - K_k*C*P_kp1_p;
	
	% update
	x_kp1 = x_kp1_p + K_k*(r_k - C*x_kp1_p);
	
	% store data
	data.x_01(:,k+1) = x_kp1;
	
	% prep for next iteration
	x_k = x_kp1;
	y_k = y_kp1;
end

plot_trajectory(data.x_01, rcwA_Ts_0_01, 'Quadcopter trajectory with updates');


function crossMat = crMat(X)
crossMat = [
	0      -X(3)  X(2);
	X(3)    0    -X(1);
	-X(2)  X(1)   0;
	];
end

function [Ad, Bd, OMEGAd] = get_discrete_sys(T, omega_k)
Ad = [
	eye(3), T*eye(3);
	zeros(3), eye(3)
	];
Bd = [
	0.5*T^2*eye(3);
	T*eye(3)
	];
nhat_k = omega_k / norm(omega_k);
OMEGAd = expm(-norm(omega_k)*T*crMat(nhat_k));
end

function plot_trajectory(data, ref, title_str)
figure;
plot3(data(1,:), data(2,:), data(3,:), 'LineWidth', 2, 'DisplayName', 'Estimated');
hold on;
plot3(ref(1,:), ref(2,:), ref(3,:), 'LineWidth', 2, 'DisplayName', 'Reference');
xlabel('x');
ylabel('y');
zlabel('z');
title(title_str);
grid on;
legend("Location", "best");
end
