function K = tvLQR(A, B, Q, R, q, u, tspan)
    % A, B, Q, R are arrays representing the time-varying matrices as
    % functions of q & u
    % tspan is the time vector
    % q,u are the state/input trajectory functions w.r.t. time
    
    % n states
    n = length(q,1);

    % Initialize the gains
    K = zeros(n,size(tspan,2));

    % Solve LQR for each time step
    for i = 1:length(tspan)
        t = tspan(i);
        qi = double(q(i));
        ui = double(u(i));
        % Extract time-varying matrices at time t
        Ai = subs(A,[q;u],[qi;ui]);
        Bi = subs(B,[q;u],[qi;ui]);

        % Let's assume Q and R are constant for now
        % Qi = subs(Q,[q;u],[qi;ui]);
        % Ri = subs(R,[q;u],[qi;ui]); 

        % Solve LQR for the current time step
        % [K_i, ~, ~] = lqr(Ai, Bi, Qi, Ri);
        % disp(K_i)

        % different ways to do lqr
        % P = None
        % P = Q + A' * P * A - A' * P * B * inv(R + B' * P * B) * B' * P * A
        [~,K_i,~] = icare(Ai,Bi,Qi,Ri,[],[],[]);

        % Compute feedback matrix K
        % K_i = inv(R + B' * P * B) * B' * P * A

        % Save the calculated gain for the current time step
        K(:,i) = K_i';
    end
end
