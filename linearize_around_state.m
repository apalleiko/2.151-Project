function [A,B,dq,du] = linearize_around_state(M,C,B,tg,q,u)
    % Linearize manipulator equations around nominal joint positions q(t) 
    % and inputs u(t). Assumes constants (like mass, length, gravity) are
    % plugged in

    % q(t) should be a function of joint states as a function of time, eg.
    % [theta_1(t); theta_2(t),...] and u(t) should be a set of reference
    % inputs over time (could be zero?)

    % See test_functions.mlx for an example

    % This function assumes that M, C, B, and g are symbolic functions of
    % q(t):  M(q)*q_ddot+C(q,q_dot)*q_dot = B*u + g(q)
    
    % returns A,B matrices for statespace equations of x = [q, q_dot] and
    % u = [torques]
    n = length(formula(q));
    m = length(formula(u));
    
    syms t
    q_dot = diff(q,t);
    % M(q)*q_ddot+C(q,q_dot)*q_dot = B*u + g(q)
    % Solve for symbolic nonlinear q_ddot
    q_ddot = simplify(inv(M)*(tg + B*u - C*q_dot));
    
    dq = sym('dq_%d',[n,1]);
    for i=1:n
        dq_i = sym(['dq_',int2str(i)]);
        dq(i) = dq_i; % Assemble symbolic joint deltas
    end

    du = sym('du_%d',[m,1]);
    for j=1:m
        du_i = sym(['du_',int2str(i)]);
        du(i) = du_i; % Assemble symbolic input deltas
    end

    % f(x,y) = f(x0,y0) + (x-x0)*df/dx|(x0,y0) + (y-y0)*df/dy|(x0,y0)
    % Get linearized q_ddot as a function of current state q(t) and
    % deviations from that state dq
    q_time_inv = sym('q_ti_',[n,1]);
    u_time_inv = sym('u_ti_',[m,1]);

    % Temporarily remove time dependence
    q_ddot_time_inv = subs(q_ddot,[q;u],[q_time_inv;u_time_inv]);
    
    q_ddot_lin = q_ddot_time_inv; % f(q0)

    for i=1:n % evaluate partial for each state variable
        dq_ddot_dqi = subs(diff(q_ddot_time_inv,q_time_inv(i))); % diffentiate wrt current state
        q_ddot_lin = q_ddot_lin + dq(i)*dq_ddot_dqi; % add partial to linearzation
    end
    
    for i=1:m % evaluate partial for each input variable
        dq_ddot_du_i = subs(diff(q_ddot_time_inv,u_time_inv(i)));
        q_ddot_lin = q_ddot_lin + du(i)*dq_ddot_du_i;
    end
    
    % q_ddot_lin is now a function of each q_i(t) and dq_i

    A_lin = sym(zeros(2*n)); % 2n joint states, [q; q_dot]
    B_lin = sym(zeros(2*n,m)); % 2n x m input combos
    A_lin(1:n,n+1:end) = eye(n); % d/dt of q is just q_dot
    
    % Now need to collect coefficients of the linearized equation to insert
    % into A_lin and B_lin
    % Since it is now linear with respect to the new state variables dq, 
    % can just get the coefficients with respect to each dq_i for each
    % q_ddot_j
    q_lin_body = formula(q_ddot_lin);
    q_coeffs = sym('qc',[n,n]); % Joint accels not dependent on velocity
    u_coeffs = sym('uc',[n,m]); % Joint velocity not dependent on inputs

    for i=1:n
        for j=1:n
            dq_j = dq(j);
            [c_qij,t_quij] = coeffs(q_lin_body(i),dq_j);
            if t_quij(1) == dq_j
                q_coeffs(i,j) = c_qij(1); % Take only first (linear) element
            else
                q_coeffs(i,j) = 0;
            end   
        end

        for k=1:m
            du_k = du(k);
            [c_uik,t_uik] = coeffs(q_lin_body(i),du_k);
            if t_uik(1) == du_k
                u_coeffs(i,k) = c_uik(1); % Take only first (linear) element
            else
                u_coeffs(i,k) = 0;
            end   
        end
    end
    
    % Now we can set the corresponding entries of A_lin and B_lin relative 
    % to the coefficients 
    A_lin(n+1:end,1:n) = q_coeffs;
    B_lin(n+1:end,:) = u_coeffs;

    A = simplify(subs(A_lin,[q_time_inv;u_time_inv],[q;u]));
    B = simplify(subs(B_lin,[q_time_inv;u_time_inv],[q;u]));
end