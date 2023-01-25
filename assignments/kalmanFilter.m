function [f_x, f_dx, f_ddx] = kalmanFilter(A,B,C,x,R,Q,x0,P0,Klim)
    % KALMANFILTER  
    %   kalmanFilter(A,B,C,x,R,Q,x0,P0,Klim)

    K1 = P0*C.'*inv(C*P0*C.' + R); % Initial gain
    x1 = A*x0 + K1*(x(1) - C*A*x0); % Initial filter
    xk_k = x1; % x k|k
    Pk_k_1 = P0; % P k|k-1
    % Initialize filtered position, velocity and acceleration vectors
    f_x = zeros(size(x));
    f_dx = zeros(size(x));
    f_ddx = zeros(size(x));
    % Recursive kalman filter
    for k = 1:size(x,1)-1
        yk_1 = x(k+1); % y k+1
        if nargin == 9 % Steady state
            xk_1_k_1 = A*xk_k + Klim*(yk_1 - C*A*xk_k); % x k+1|k+1
            xk_k = xk_1_k_1; % Update the previous estimation x k|k
        else
            Pk_1_k = A*Pk_k_1*A.' - ...
                A*Pk_k_1*C.'*inv(C*Pk_k_1*C.' + R)*C*Pk_k_1*A.' + Q; % P k+1|k
            Pk_k_1 = Pk_1_k; % Update the previous P k|k
            Kk_1 = Pk_1_k*C.'*inv(C*Pk_1_k*C.' + R); % Update the gain K k+1
            xk_1_k_1 = A*xk_k + Kk_1*(yk_1 - C*A*xk_k); % x k+1|k+1
            xk_k = xk_1_k_1; % Update the previous estimation x k|k
        end
        % Update the filtered vectors
        f_x(k) = xk_1_k_1(1);
        f_dx(k) = xk_1_k_1(2);
        f_ddx(k) = xk_1_k_1(3);
    end
end