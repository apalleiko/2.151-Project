function xNext = robot_state_model(dt,t,x,Kt,q_traj,u_traj,v,robot)
% x is current state, u is function of time
% Use forward Euler
n = length(x)/2; % number of states
q = x(1:n); % get current configuration of the robot
q_dot = x(n+1:end); % get current velocities of the robot
% Assume F_ext is zero
q0_t = q_traj(t);
K_t = Kt(t);
u_t = double(u_traj(t));
if length(q0_t) == n % This means no velocity in q_traj
    K_t(:,n+1:end) = 0; % Disregards velocity differences
    dx = (x-[q0_t;0;0]);
elseif length(q0_t) == 2*n
    dx = (x-q0_t);
end
G = double(K_t*dx);
tau = u_t + G + v;
q_ddot = forwardDynamics(robot,q,q_dot,tau,[]);

xNext = x + dt*[q_dot; q_ddot];
end

