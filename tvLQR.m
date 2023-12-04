function [time,K] = tvLQR(A, B, Q, R, S_tf, tspan)

intg_tspan = flip(tspan);
s0 = reshape(S_tf,[],1);

options = odeset('RelTol',1e-8,'AbsTol',1e-8);

inFun=@(t,s) matOde(t,s,A,B,Q,R);

sol = ode45(inFun,intg_tspan,s0,options);
K = zeros(size(R(0),1),size(Q(0),1),length(sol.x));

% Solve LQR for each time step
for i = 1:length(sol.x)
    t = sol.x(i);

    % Extract time-varying matrices at time t
    Ai = A(t);
    Bi = B(t);
    Qi = Q(t);
    Ri = R(t);

    S = reshape(sol.y(:,i),size(Ai,1),size(Ai,1));

    % Compute feedback matrix K

    K_i = Ri\(Bi'*S);

    % [~,K_i,~] = icare(Ai,Bi,Qi,Ri,[],[],[]);

    % Save the calculated gain for the current time step
    K(:,:,i) = K_i;
end
time = sol.x;
end

function dx = matOde(t,x,A,B,Q,R)
n = sqrt(size(x,1));
S = reshape(x,n,n);
S_dot =@(t,S) -(S*A(t) + A(t)'*S - S*B(t)*(R(t)\(B(t)'*S)) + Q(t));
S_dot_mat = S_dot(t,S);
dx = reshape(S_dot_mat,[],1);
end