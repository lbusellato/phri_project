%--------------------------------------------------------------------------
%
% LS.m
%
% This function uses Least Squares to estimate the parameters of a DC motor
% model, given the vector X of angular acceleration and velocity and the
% vector Y of voltages.
%
% Author: Lorenzo Busellato, VR472249, 2022
%
%--------------------------------------------------------------------------
function [k, tau] = LS(X, Y)
    beta_hat = inv(X.'*X)*X.' * Y;  
    % Parameters estimation
    k = 1/beta_hat(2)
    tau = k*beta_hat(1)
end