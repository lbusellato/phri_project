%--------------------------------------------------------------------------
%
% RLS.m
%
% This function uses Recursive Least Squares to estimate the parameters of 
% a DC motor model, given the vector X of angular acceleration and velocity,
% the vector Y of voltages and the forgetting factor lambda.
%
% Author: Lorenzo Busellato, VR472249, 2022
%
%--------------------------------------------------------------------------
function [k, tau] = RLS(X, Y, lambda)    
    % Initialization
    error = zeros(size(X, 1),1); 
    for i = 1:size(X, 1)
        Xk{i} = X(i,:);           
        P{i} = eye(2);           
        K{i} = zeros(2,1);       
        beta_hat{i} = zeros(2,1); 
    end
    % Recursive Least Squares (slide 27)
    for k = 2:(size(X, 1))   
       % Prediction error
       error(k) = Y(k) - Xk{k}*beta_hat{k-1}; 
       % Current estimation
       K = P{k-1} * Xk{k}.'; 
       beta_hat{k} = beta_hat{k-1} + K*error(k,1);      
       P{k} = (1/lambda)*(P{k-1} - (P{k-1}*Xk{k}.'*Xk{k}*P{k-1})/(lambda + Xk{k}*P{k-1}*Xk{k}.')); 
    end
    beta_hat = beta_hat{end};
    k = 1/beta_hat(2);
    tau = k*beta_hat(1);
end