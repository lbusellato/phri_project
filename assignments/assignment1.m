%--------------------------------------------------------------------------
%
% Assignment 1: SISO 4-channel bilateral teleoperation
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
Dm = 0;%5;   % Master damping
Ds = 0;%10;   % Slave damping
% Human parameters
Jh = 1;   % Inertia
Bh = 1;   % Damping 
% Environment parameters
Be = 100; % Damping
Ke = 200; % Stiffness
xe = 1.5;   % Environment position
% Sinusoidal reference signal
A = 1; % Amplitude
Fc = 1;   % Frequency
% Step reference signal
Flp = 0.5;
% Master controller
Bm = 20*0.8;
Km = 10*1;
% Human controller
Ph = 10*1; 
Dh = 20*0.8; 
% Slave controller
Bs = 4*Bm; 
Ks = 4*Km; 

%% Load the model

% Load and open the Simulink system
open('teleop_4ch.slx');
