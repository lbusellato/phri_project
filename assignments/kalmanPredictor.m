function [p_x, p_dx, p_ddx] = kalmanPredictor(A,B,C,x,R,Q,x0,P0,Kbar_inf)
    % KALMANPREDICTOR
    %   KALMANPREDICTOR(A,B,C,x,R,Q,x0,P0,Kbar_inf)

    Kbar_0 = P0*C.'*inv(C*P0*C.' + R);      % Initial gain
    x_1 = A*x0 + Kbar_0 *(x(1) - C*A*x0);   % Initial filter
    xk_k_1 = x_1; % x k|k-1
    Pk_k_1 = P0;  % P k|k-1
    % Initialize filtered position, velocity and acceleration vectors
    p_x = zeros(size(x));
    p_dx = zeros(size(x));
    p_ddx = zeros(size(x));
    % Recursive Kalman predictor
    for k = 1:(size(x,1)-1)
        yk_1 = x(k); % y k+1
        if nargin == 9 % Steady state  
            xk_1_k = A*xk_k_1 + Kbar_inf*(yk_1 - C*xk_k_1);
            xk_k_1 = xk_1_k; % Update the previous estimation
        else
            Pk_1_k = A*Pk_k_1*A.' - ...
                A*Pk_k_1*C.'*inv(C*Pk_k_1*C.' + R) + C*Pk_k_1*A.' + Q; % P k+1|k
            % Update the gain
            Kbar_k = A*Pk_k_1 * C.' * inv(C*Pk_k_1*C.' + R);
            % Update P
            Pk_k_1 = Pk_1_k; 
            %Estimate the "next" value of x (slides: x at time (K+1) given measurements until K)
            xk_1_k = A*xk_k_1 + Kbar_k*(yk_1 - C*xk_k_1);
            xk_k_1 = xk_1_k; % Update the previous estimation
        end
        % Update the filtered vectors
        f_x(k) = xk_1_k(1);
        f_dx(k) = xk_1_k(2);
        f_ddx(k) = xk_1_k(3);
    end
end