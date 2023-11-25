clear, clc
[q_ddot, q, u, vars] = robot_arm_dynamics();
[A,B,dq,du] = linearize_around_state(q_ddot,q,u);
%%
robot = create_robot_tree();
