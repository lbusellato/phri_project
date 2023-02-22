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
Dm = 5;   % Master damping
Ds = 0;   % Slave damping
% Human parameters
Jh = 1;   % Inertia
Bh = 1;   % Damping 
Kh = 0;   % Stiffness               
% Environment parameters
Be = 100; % Damping
Ke = 200; % Stiffness
xe = 1.5;   % Environment position
% Sinusoidal reference signal
A = 1; % Amplitude
Fc = 3;   % Frequency
% Master controller
Bm = 20*0.8;
Km = 10*1;
% Human controller
Ph = 10*1; 
Dh = 20*0.8; 
% Slave controller
Bs = 4*Bm; 
Ks = 4*Km; 
% Transport delay
d = 10;
% Characteristic impedance for the scattering
b = 1;
% Wave filtering
Fip = 5;
% Noise
noise = 0;
% Kalman
x0 = [0 0];
Ak = [1 Ts ; 
     0 1 ]; 
B = [Ts^2/2; Ts];
C = [1 0];
q = 10000000;
Q_m = q*(B*B');
Q_s = q*(B*B');
R = 1;
% Low-pass filter for the derivative in Zh
Flp = 0.01;

%% FORCE-POSITION

% Load and open the Simulink system
open('scattering_FP.slx');

%% POSITION-POSITION
Dm = 0;
% Load and open the Simulink system
open('scattering_PP.slx');