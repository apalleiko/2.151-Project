function [A,B] = linearize_around_state(M,C,B)
    % Linearize manipulator equations around nominal state q:
    % M(q)*q_ddot+C(q)*q_dot = B*torques + g(q)
    
    % q is vector of joint positions
    % returns A,B matrices for statespace equations of x = [q, q_dot] and
    % u = [torques]

    n = size(M,1); % n joint states
    assert(n == size(B,2)); % should be n control torques
    A_lin = zeros(2*n);
    B_lin = zeros(2*n,n);

    A_lin(1:n,n+1:end) = eye(n); % d/dt of q is q_dot
    A


end