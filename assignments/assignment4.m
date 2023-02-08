%--------------------------------------------------------------------------
%
% Assignment 4: Kalman smoother
%
% Author: Lorenzo Busellato, VR472249, 2022
%
%--------------------------------------------------------------------------

%% SETUP

close all;
clc;
clearvars;

addpath('../simulink/');
addpath('../functions/');
set(cstprefs.tbxprefs,'FrequencyUnits','Hz','Grid','on')
s = tf('s');
Ts = 0.001;
simTime = 10;
time = 0:Ts:simTime;

%% Load model and generate the measurements

% Simulate and get the position timeseries
out = sim('signalsKF.slx', simTime);
%x = out.position_GaussianNoise.signals.values;
x = out.position_QuantizationNoise.Data;
dx = out.velocity.signals.values;
ddx = out.acceleration.signals.values;

%% Kalman smoother

[~, s_dx, s_ddx] = KS(x, time);

%% Kalman filter for comparison

[~, f_dx, f_ddx] = KF(x, time, false);

%% Plot the results

figure(); subplot(311); grid on; ylabel("Position [m or rad]"); hold on; plot(time, x, 'LineWidth', 2);
subplot(312);  grid on; xlabel("Time [s]"); ylabel("Velocity [m/s or rad/s]"); hold on; 
plot(time, dx, 'LineWidth', 2); hold on; plot(time, f_dx, 'LineWidth', 2); hold on; 
plot(time, s_dx, 'LineWidth', 2); hold on; legend("Actual", "Filter", "Smoother");
subplot(313);  grid on; xlabel("Time [s]"); ylabel("Acceleration [m/s^2 or rad/s^2]"); hold on; 
plot(time, ddx, 'LineWidth', 2); hold on; plot(time, f_ddx, 'LineWidth', 2); hold on; 
plot(time, s_ddx, 'LineWidth', 2); hold on; legend("Actual", "Filter", "Smoother");















