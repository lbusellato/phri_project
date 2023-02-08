%--------------------------------------------------------------------------
%
% Assignment 6: Scattering-based bilateral teleoperation architecture
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
Ts = 0.001;

%% PARAMETERS

% Robot parameters 
Mm = 0.5; % Master robot's mass
Ms = 2;   % Slave robot's mass
Dm = 0;   % Master damping
Ds = 0;   % Slave damping
% Human parameters
Jh = 1;   % Inertia
Bh = 1;   % Damping 
Kh = 0;   % Stiffness
% Environment parameters
Je = 0;   % Inertia
Be = 100; % Damping
Ke = 200; % Stiffness
xe = 1.5;   % Environment position (>1 contact - <1 free motion)
% Sinusoidal reference signal
A = 1; % Amplitude
Fc = 1;   % Frequency
% Step reference signal
Flp = 0.5;
% Master controller
Bm = 20*0.8;  % Derivative
Km = 10*1;    % Proportional
% Human controller
Dh = 20*0.8; % Derivative
Ph = 10*1;   % Proportional
% Slave controller
Bs = 4*Bm;  % Derivative
Ks = 4*Km;  % Proportional
% Input delay
d = 5;
% Input noise
variance = 1e-5;
% Kalman Filter
Ak = [1 Ts; 0 1]; Bk = [Ts^2/2 ; Ts]; Ck = [1 0];
q = 10e5;
Q = q*(Bk*Bk.');
R = 1;
% Scattering - characteristic impedance
b = 1;

%% FORCE-POSITION

% Load and open the Simulink system
open('scattering_force_position.slx');

%% POSITION-POSITION

% Load and open the Simulink system
open('scattering_position_position.slx');


