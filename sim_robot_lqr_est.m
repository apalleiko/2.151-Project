function [t,x] = sim_robot_lqr_est(robot,Kt,q_traj,u_traj,x0,tspan,options)
% simulate the nonlinear response of a robot to given inputs
% Assume state is now [x; x_est]
    [t,x] = ode45(@(t,x) robot_dynamics_est(t,x,Kt,q_traj,u_traj,robot),tspan,x0,options);
end

function dxdt = robot_dynamics_est(t,x,Kt,q_traj,u_traj,robot)
% x is current state, u is function of time, x_est is estimated current
% state, if applicable.
% This function appends x0_est to states in order to track this over time
% for accurate controller simulation
n = length(x)/4; % number of states

q = x(1:n); % get current true configuration of the robot
q_dot = x(n+1:2*n); % get current true velocities of the robot

x_est = x(2*n+1:end); % get current estimated state of the system
q_est = x_est(1:n);
q_dot_est = x_est(n+1:2*n);

% Get time changing vars
q0_t = q_traj(t);
K_t = Kt(t);
u_t = double(u_traj(t));

% Use x_est in LQR feedback
if length(q0_t) == n % This means no velocity in q_traj
    K_t(:,n+1:end) = 0; % Disregards velocity differences
    dx = (x_est-[q0_t;0;0]);
elseif length(q0_t) == 2*n
    dx = (x_est-q0_t);
end
G = double(K_t*dx);
tau = u_t + G;

q_ddot = forwardDynamics(robot,q,q_dot,tau,[]);
q_ddot_est = forwardDynamics(robot,q_est,q_dot_est,tau,[]);

dxdt = [q_dot; q_ddot; q_dot_est; q_ddot_est];
end