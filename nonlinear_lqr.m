function dydt = nonlinear_lqr(t,y,K,x_ss,t1_ss,u_ss)
% Simulate using an lqr controller on nonlinear system
    mc = 1;
    mp = 1;
    l = 1;
    g = 1;
    x = y(1);
    t1 = y(2);
    x_dot = y(3);
    t1_dot = y(4);
    fx = -K * [x-x_ss, t1-t1_ss, x_dot, t1_dot]';
    x_ddot = 1/(mc+mp*sin(t1)^2) * (fx + mp*sin(t1) * (l*t1_dot^2+g*cos(t1)));
    t1_ddot = 1/(l*(mc+mp*sin(t1)^2)) * (-fx*cos(t1)-mp*l*t1_dot^2*cos(t1)*sin(t1)-(mc+mp)*g*sin(t1));
    dydt = [x_dot, t1_dot, x_ddot, t1_ddot]';
end

