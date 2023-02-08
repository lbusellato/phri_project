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

%% Z width

% Transfer functions
Cm = Bm + Km/s; % Master controller
Cs = Bs + Ks/s; % Slave controller
Zm = Mm*s + Dm; Zm_inv = 1/Zm; % Master impedance/admittance
Zs = Ms*s + Ds; Zs_inv = 1/Zs; % Slave impedance/admittance
% Hybrid matrix under perfect transparency
H_bar = [0  1; 
         -1 0]; 
H11 = H_bar(1,1); H12 = H_bar(1,2); 
H21 = H_bar(2,1); H22 = H_bar(2,2);  
% Environment impedance
Ze = Je*s + Be + Ke/s;
Zt = (H11 - H12*Ze) / (H21 - H22*Ze);
Zt_width = (H12*H21 - H11*H22)/(H22*H21); % Should be = Inf (perfect transparency)
fprintf("Zt_width = %f\n", Zt_width);
    
%% Load the model

% Load and open the Simulink system
open('teleop_4ch.slx');
