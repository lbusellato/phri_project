%--------------------------------------------------------------------------
%
% Assignment 7: Tank-based bilateral teleoperation architecture
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
Ds = 0;
Dm = 0;
% Human parameters
Jh = 1;   % Inertia
Bh = 1;   % Damping          
% Environment parameters
Be = 100; % Damping
Ke = 200; % Stiffness
xe = 1.5;   % Environment position
% Sinusoidal reference signal
A = 1; % Amplitude
Fc = 0.5;   % Frequency
% Master controller
Bm = 40*0.8;
Km = 10*1;
% Human controller
Ph = 10*1; 
Dh = 20*0.8; 
% Slave controller
Bs = 4*Bm; 
Ks = 4*Km; 
% Transport delay
d = 10;
% Noise
noisePos = 0;
% Kalman
x0 = [0 0];
Ak = [1 Ts ; 
     0 1 ]; 
B = [Ts^2/2; Ts];
C = [1 0];
q = 1e10;
Q_m = q*(B*B');
Q_s = q*(B*B');
R = 1;
Flp = 1;
tlcAlpha = 2;
beta = 0.1;
Hd = 1;
Hm_init = 0.5; % FP
Hs_init = 0.5; % FP
%Hm_init = 0; % PP
%Hs_init = 0; % PP

%% FORCE-POSITION
% Load and open the Simulink system
open('tank_FP.slx');

%% POSITION-POSITION
% Load and open the Simulink system
open('tank_PP.slx');