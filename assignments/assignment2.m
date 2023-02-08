%--------------------------------------------------------------------------
%
% Assignment 2: 2-channel and 3-channel teleoperation architectures
%
% Author: Lorenzo Busellato, VR472249, 2022
%
%--------------------------------------------------------------------------

%% SETUP

close all;
clc;
clearvars;
addpath('../simulink/')
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
xe = 1.5;   % Environment position
% Sinusoidal reference signal
A = 1; % Amplitude
Fc = 1;   % Frequency
% Step reference signal
Flp = 0.5;
% Master controller
Bm = 10;  % Derivative
Km = 20;    % Proportional
% Human controller
Dh = 50;% 16; % Derivative
Ph = 2000;% 10; % Proportional
% Slave controller
Bs = 500;  % Derivative
Ks = 4000;  % Proportional
% Transport delay
d = 0.02; % 20ms
    
%% Load the model

% Load and open the Simulink system
open('teleop_2_3ch.slx');
