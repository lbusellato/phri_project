%--------------------------------------------------------------------------
%
% KF.m
%
% This function uses Kalman filtering to estimate velocity and acceleration
% from noisy position measures.
%
% Author: Lorenzo Busellato, VR472249, 2022
%
%--------------------------------------------------------------------------
function [f_x, f_dx, f_ddx, xfkk, xfk1k, Pfkk, Pfk1k] = KF(x, t, ss)
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
    xfkk(:,1) = x0;
    Pfkk(:,:,1) = P0;
    xfk1k(:,1) = A*x0;
    Pfk1k(:,:,1) = A*P0+Q;
    if ss == true
        % Get P_inf by solving the Riccati equation
        [Pinf,~,~] = idare(A.', C.', Q, R);
        K = Pinf*C.'*pinv(C*Pinf*C.' + R); % Steady state Kalman gain
    end
    % Recursive Kalman filter
    for k = 2:size(x,1)
        if ss == false
            xprev = xfk1k(:,k-1);
            Pprev = Pfk1k(:,:,k-1);
            % Kalman gain
            K = Pprev*C.'*inv(C*Pprev*C.'+R);
            % Update current estimations
            xfkk(:,k) = xprev + K*(x(k,1)-C*xprev);
            Pfkk(:,:,k) = Pprev - K*C*Pprev;
            % Update previous estimations
            xfk1k(:,k) = A*xfkk(:,k);
            Pfk1k(:,:,k) = A*Pfkk(:,:,k)*A.' + Q;
        else % Steady-state
            xprev = xfkk(:,k-1);
            xfkk(:,k) = A*xprev + K*(x(k,1)-C*A*xprev);
        end
    end
    f_x = xfkk(1,:);
    f_dx = xfkk(2,:);
    f_ddx = xfkk(3,:);
end