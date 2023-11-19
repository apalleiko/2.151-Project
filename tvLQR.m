function K = tvLQR(A, B, Q, R, tspan)
    % A, B, Q, R are arrays representing the time-varying matrices
    % tspan is the time vector
    
    % Initialize the gains
    K = zeros(2,size(tspan,2));

    % Solve LQR for each time step
    for i = 1:length(tspan)
        t = tspan(i);
        
        % Extract time-varying matrices at time t
        Ai = A(t);
        Bi = B(t);
        Qi = Q(t);
        Ri = R(t);

        % Solve LQR for the current time step
        % [K_i, ~, ~] = lqr(Ai, Bi, Qi, Ri);
        % disp(K_i)

        % different ways to do lqr
        % P = Q + A' * P * A - A' * P * B * inv(R + B' * P * B) * B' * P * A
        [~,K_i,~] = icare(Ai,Bi,Qi,Ri,[],[],[]);

        % Compute feedback matrix K
        % K_i = inv(R + B' * P * B) * B' * P * A

        % Save the calculated gain for the current time step
        K(:,i) = K_i';
    end
end
