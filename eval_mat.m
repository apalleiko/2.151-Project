function m = eval_mat(mat,vars,eval_vars,t)
% Evaluated A,B matrices at a given point in a trajectory
m = double(subs(mat,vars,eval_vars(t)));
end