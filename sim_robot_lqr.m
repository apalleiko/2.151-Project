function [t,x] = sim_robot_lqr(robot,Kt,q_traj,x0,tspan)
% simulate the nonlinear response of a robot to given inputs
[t,x] = ode45(@(t,x) robot_dynamics(t,x,Kt,q_traj,robot),tspan,x0);
end

function dxdt = robot_dynamics(t,x,Kt,q_traj,robot)
% x is current state, u is function of time
n = length(x)/2; % number of states
q = x(1:n); % get current configuration of the robot
q_dot = x(n+1:end); % get current velocities of the robot
M = massMatrix(robot,q);
C = velocityProduct(robot,q,q_dot);
G = gravityTorque(robot,q);
% Assume F_ext is zero
q0_t = q_traj(t);
x0_t = [q0_t; 0; 0];
K_t = Kt(t);
tau = double(K_t*(x-x0_t));
q_ddot = M\(-C-G+tau);

dxdt = [q_dot; q_ddot];
end