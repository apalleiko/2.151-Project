function [t,x] = sim_robot_lqr(robot,Kt,q_traj,u_traj,x0,tspan,options)
% simulate the nonlinear response of a robot to given inputs
% Assume we know the true state for control
    [t,x] = ode45(@(t,x) robot_dynamics(t,x,Kt,q_traj,u_traj,robot),tspan,x0,options);
end

function dxdt = robot_dynamics(t,x,Kt,q_traj,u_traj,robot)
% x is current state, u is function of time, x_est is estimated current
% state, if applicable.

n = length(x)/2; % number of states
q = x(1:n); % get current configuration of the robot
q_dot = x(n+1:end); % get current velocities of the robot

% Get time changing vars
q0_t = q_traj(t);
K_t = Kt(t);
u_t = double(u_traj(t));

% Use current state to calc feedback
if length(q0_t) == n % This means no velocity in q_traj
    K_t(:,n+1:end) = 0; % Disregards velocity differences
    dx = (x-[q0_t;0;0]);
elseif length(q0_t) == 2*n
    dx = (x-q0_t);
end
G = double(K_t*dx);
tau = u_t + G;
q_ddot = forwardDynamics(robot,q,q_dot,tau,[]);

dxdt = [q_dot; q_ddot];
end