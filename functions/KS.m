%--------------------------------------------------------------------------
%
% KS.m
%
% This function uses Kalman smoothing to estimate velocity and acceleration
% from noisy position measures.
%
% Author: Lorenzo Busellato, VR472249, 2022
%
%--------------------------------------------------------------------------
function [s_x, s_dx, s_ddx] = KS(x, t)
    % Initialize parameters
    Ts = mean(diff(t));
    A = [ 1 Ts Ts^2/2; 
           0 1 Ts; 
           0 0 1];
    % Forward step - Standard Kalman filtering
    [~,~,~,xfkk,xfk1k,Pfkk,Pfk1k] = KF(x,t,false);
    % First iteration
    xskN = zeros(3,size(x,1));
    PskN = zeros(3,3,size(x,1));
    xskN(:,size(x,1)) = xfkk(:,size(x,1));
    PskN(:,:,size(x,1)) = Pfkk(:,:,size(x,1));
    % Backward step - Smoothing
    for k = size(x,1)-1:-1:1
        K = Pfkk(:,:,k)*A.'*inv(Pfk1k(:,:,k));
        xskN(:,k) = xfkk(:,k) + K*(xskN(:,k+1) - xfk1k(:,k));
        PskN(:,:,k) = Pfkk(:,:,k) + K*(PskN(:,:,k+1) - Pfk1k(:,:,k));
    end
    s_x = xskN(1,:);
    s_dx = xskN(2,:);
    s_ddx = xskN(3,:);
end
