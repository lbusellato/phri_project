%--------------------------------------------------------------------------
%
% Assignment 5: Parameter identification with LS and RLS
%
% Author: Lorenzo Busellato, VR472249, 2022
%
%--------------------------------------------------------------------------

%% SETUP

close all;
clc;
clearvars;

addpath('../simulink/')
addpath('../functions/')
set(cstprefs.tbxprefs,'FrequencyUnits','Hz','Grid','on')
s = tf('s');
% Load the DC motor's parameters
DCmotor_parameters;
% Run the simulink model
out = sim('DCmotor_maxon_Pcontrol', 5);
position = out.positions.Data;
voltages = out.voltages.Data;
time = out.positions.Time;
actual_velocity = out.actual_vel.Data;
actual_acceleration = out.actual_acc.Data;

%% Get velocity and acceleration with Kalman smoother

[theta, omega, dOmega] = KS(position, time);

%% Least Square estimation 
X(:,1) = dOmega; X(:,2) = omega;
Y(:,1) = voltages;
[k_LS, tau_LS] = LS(X, Y)
% Recompute velocity and acceleration using the estimated parameters
X_hat = Y * pinv([tau_LS/k_LS; 1/k_LS]);
figure; subplot(221); plot(time, actual_velocity); hold on; plot(time, X_hat(:,2)); hold on; plot(time, omega);
legend('Actual', 'Reprojected', 'Kalman'); title('Least Squares'); grid on; xlabel("Time [s]"); ylabel("Velocity [rad/s]");
subplot(223); plot(time, actual_acceleration); hold on; plot(time, X_hat(:,1)); hold on; plot(time, dOmega);
legend('Actual', 'Reprojected', 'Kalman'); grid on; xlabel("Time [s]"); ylabel("Acceleration [rad/s^2]");

%% Recursive Least Square estimation 
lambda = 0.9999; % Forgetting factor
[k_RLS, tau_RLS] = RLS(X, Y, lambda)
% Recompute velocity and acceleration using the estimated parameters
X_hat = Y * pinv([tau_RLS/k_RLS; 1/k_RLS]);
subplot(222); plot(time, actual_velocity); hold on; plot(time, X_hat(:,2)); hold on; plot(time, omega);
legend('Actual', 'Reprojected', 'Kalman'); title('Recursive Least Squares'); grid on; xlabel("Time [s]"); ylabel("Velocity [rad/s]");
subplot(224); plot(time, actual_acceleration); hold on; plot(time, X_hat(:,1)); hold on; plot(time, dOmega);
legend('Actual', 'Reprojected', 'Kalman'); grid on; xlabel("Time [s]"); ylabel("Acceleration [rad/s^2]");
