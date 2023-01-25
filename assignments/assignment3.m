%--------------------------------------------------------------------------
%
% Assignment 3: Kalman filter
%
% Author: Lorenzo Busellato, VR472249, 2022
%
%--------------------------------------------------------------------------

%% SETUP

close all;
clc;
clearvars;

set(cstprefs.tbxprefs,'FrequencyUnits','Hz','Grid','on')
s = tf('s');
Ts = 0.001;
simTime = 10;
time = 0:Ts:simTime;

%% Load model and generate the measurements

% Simulate and get the position
out = sim('signalsKF.slx', simTime);
x = out.position_GaussianNoise.signals.values;
dx = out.velocity.signals.values;
ddx = out.acceleration.signals.values;


%% Set up values for Kalman

% Set the initial conditions
P0 = diag([1e-5 1e-5 1e-5]);
x0 = [0 0 0].';
% Set the A, B and C matrices
A = [1   Ts Ts^2/2;
     0   1   Ts   ;
     0   0   1   ];
B = [Ts^3/6; Ts^2/2; Ts];
C = [1 0 0];
% Set the R and Q parameters
R = var(x); 
q = 10e6;
Q = q * B * B';

%% Kalman filter

[f_x, f_dx, f_ddx] = kalmanFilter(A,B,C,x,R,Q,x0,P0);

%% Steady state Kalman filter

% Get P_inf by solving the algebraic Riccati equation
[P_inf,K,L] = idare(A.', C.', Q, R);
K_inf = P_inf*C.'*inv(C*P_inf*C.' + R);
% Apply the filter (steady state)
[ssf_x, ssf_dx, ssf_ddx] = kalmanFilter(A,B,C,x,R,Q,x0,P0,K_inf);

%% Kalman predictor

[p_x, p_dx, p_ddx] = kalmanPredictor(A,B,C,x,R,Q,x0,P0);

%% Steady state Kalman predictor

Kbar_inf = A*P_inf*C.'*inv(C*P_inf*C.' + R);
[ssp_x, ssp_dx, ssp_ddx] = kalmanPredictor(A,B,C,x,R,Q,x0,P0,Kbar_inf);

%% Plot the results

% Filter
figure; subplot(2,2,1); title("Kalman filter"); grid on; xlabel("Time [s]"); ylabel("Velocity [m/s or rad/s]"); hold on;
plot(time, f_dx); hold on; plot(time, dx); legend("Filter", "Actual");
subplot(2,2,3);  grid on; xlabel("Time [s]"); ylabel("Acceleration [m/s^2 or rad/s^2]"); hold on;
plot(time, f_ddx); hold on; plot(time, ddx); legend("Filter", "Actual");
subplot(2,2,2); title("Steady state Kalman filter"); grid on; xlabel("Time [s]"); ylabel("Velocity [m/s or rad/s]"); hold on;
plot(time, ssf_dx); hold on; plot(time, dx); legend("Filter", "Actual");
subplot(2,2,4); grid on; xlabel("Time [s]"); ylabel("Acceleration [m/s^2 or rad/s^2]"); hold on;
plot(time, ssf_ddx); hold on; plot(time, ddx); legend("Filter", "Actual");
% Predictor
figure; subplot(2,2,1); title("Kalman predictor"); grid on; xlabel("Time [s]"); ylabel("Velocity [m/s or rad/s]"); hold on;
plot(time, p_dx); hold on; plot(time, dx); legend("Predictor", "Actual");
subplot(2,2,3);  grid on; xlabel("Time [s]"); ylabel("Acceleration [m/s^2 or rad/s^2]"); hold on;
plot(time, p_ddx); hold on; plot(time, ddx); legend("Predictor", "Actual");
subplot(2,2,2); title("Steady state Kalman predictor"); grid on; xlabel("Time [s]"); ylabel("Velocity [m/s or rad/s]"); hold on;
plot(time, ssp_dx); hold on; plot(time, dx); legend("Predictor", "Actual");
subplot(2,2,4); grid on; xlabel("Time [s]"); ylabel("Acceleration [m/s^2 or rad/s^2]"); hold on;
plot(time, ssp_ddx); hold on; plot(time, ddx); legend("Predictor", "Actual");