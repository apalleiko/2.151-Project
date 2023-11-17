function [A,B] = linearize_around_state(M,C,B,g,q,u)
    % Linearize manipulator equations around nominal joint positions q(t).
    % q(t) should be a vector of joint states as a function of time, eg.
    % [theta_1(t); theta_2(t),...]
    % See test_functions.mlx for an example

    % This function assumes that M, C, B, and g are symbolic functions of
    % q(t):  M(q)*q_ddot+C(q,q_dot)*q_dot = B*u + g(q)
    
    % returns A,B matrices for statespace equations of x = [q, q_dot] and
    % u = [torques]
    q_ddot = simplify(inv(M)*(tg + B*u - C*q_dot));

    n = size(q); % n joint states
    assert(n == size(u)); % should be n control torques
    A_lin = zeros(2*n);
    B_lin = zeros(2*n,n);

    A_lin(1:n,n+1:end) = eye(n); % d/dt of q is q_dot
    A


end