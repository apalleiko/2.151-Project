function [q_ddot, dq, q, u, vars] = robot_arm_dynamics()
    
    % Define joint states and inputs
    syms m L t g theta1(t) theta2(t) tau1(t) tau2(t)
    
    % Define symbolic variables with it
    vars = [m L g];
    
    % Create forward dynamics
    x1 = L*cos(theta1);
    y1 = L*sin(theta1);
    x2 = x1+L*cos(theta1+theta2);
    y2 = y1+L*sin(theta1+theta2);
    dx1 = diff(x1,t);
    dx2 = diff(x2,t);
    dy1 = diff(y1,t);
    dy2 = diff(y2,t);
    dtheta1 = diff(theta1,t);
    dtheta2 = diff(theta2,t);
    I_1 = (1/3)*m*L^2;
    I_2 = (1/3)*m*L^2;

    % Define energies
    KE=(1/2)*m*(L^2)*((((5/3)+cos(theta2))*(dtheta1^2))+(((2/3)+cos(theta2))*dtheta1*dtheta2+(1/3)*dtheta2^2));
    P=(0.5*m*g*L*sin(theta1))+(m*g*(L*sin(theta1)+0.5*L*sin(theta1+theta2)));
    Le = KE-P;
    
    q = [theta1; theta2];
    u = [tau1; tau2];
    dq = diff(q,t);
    
    L_mat = [Le];
    dLdq = jacobian(L_mat,q);
    dLdqd = jacobian(L_mat,dq);
    EL = dLdq-diff(dLdqd,t);
    EL_eq = transpose(EL) == [tau1;tau2];
    EL_eq_simp = formula(simplify(EL_eq));
    
    syms d2q_1 d2q_2
    EL_eq_sym = subs(EL_eq_simp, [diff(theta1(t), t, t), diff(theta2(t), t, t)],[d2q_1, d2q_2]);
    q_ddot = solve(EL_eq_sym,[d2q_1, d2q_2]);
    q_ddot = [q_ddot.d2q_1;q_ddot.d2q_2];
end