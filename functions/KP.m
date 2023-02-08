%--------------------------------------------------------------------------
%
% KP.m
%
% This function uses Kalman filtering to estimate velocity and acceleration
% from noisy position measures.
%
% Author: Lorenzo Busellato, VR472249, 2022
%
%--------------------------------------------------------------------------
function [p_x, p_dx, p_ddx, xf, Pf] = KP(x, t, ss)
    % Initialize parameters
    Ts = mean(diff(t));
    A = [ 1 Ts Ts^2/2; 
           0 1 Ts; 
           0 0 1];
    B = [Ts^3/6 Ts^2/2 Ts]';
    C = [1 0 0];
    Q = (B*B')*10000000;
    R = var(x);
    P0 = diag([1e-5,1e-5,1e-5]);
    x0 = [0 0 0]';
    % First iteration
    xfk1k(:,1) = x0;
    Pfk1k(:,:,1) = P0;
    if ss == true % Steady state
        % Get P_inf by solving the Riccati equation
        [Pinf,~,~] = idare(A.', C.', Q, R);
        K = A*Pinf*C.'*inv(C*Pinf*C.' + R); % Steady state Kalman gain
    end
    % Recursive Kalman predictor
    for k = 2:size(x,1)
        if ss == false
            xprev = xfk1k(:,k-1);
            Pprev = Pfk1k(:,:,k-1);
            K = A*Pprev*C.'*inv(C*Pprev*C.'+R);
            xfk1k(:,k) = A*xprev + K*(x(k,1) - C*xprev);
            Pfk1k(:,:,k) = A*Pprev*A.'-K*C*Pprev*A.'+Q;
        else % Steady state
            xprev = xfk1k(:,k-1);
            % Update the estimation
            xfk1k(:,k) = A*xprev + K*(x(k,1)-C*xprev);
        end
    end
    p_x = xfk1k(1,:);
    p_dx = xfk1k(2,:);
    p_ddx = xfk1k(3,:);
end