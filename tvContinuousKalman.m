function [time,L] = tvContinuousKalman(A, C, V, W, P_0, tspan)

p0 = reshape(P_0,[],1);

options = odeset('RelTol',1e-4,'AbsTol',1e-6);

inFun=@(t,p) matOde(t,p,A,C,V,W);

sol = ode45(inFun,tspan,p0,options);

% Multiplied by output y (which will have one measurement per joint)
L = zeros(size(A(0),1),size(C(0),1),length(sol.x));

% Solve Kalman Gains for each time step
for Vi = 1:length(sol.x)
    t = sol.x(Vi);

    % Extract time-varying matrices at time t
    Ai = A(t);
    Ci = C(t);
    Vi = V(t);
    Wi = W(t);

    P = reshape(sol.y(:,Vi),size(Ai,1),size(Ai,1));

    % Compute feedback matrix L
    L_i = P*Ci\Wi;

    % Save the calculated gain for the current time step
    L(:,:,Vi) = L_i;
end
time = sol.x;
end

function dx = matOde(t,x,A,C,V,W)
n = sqrt(size(x,1));
P = reshape(x,n,n);
P_dot = @(t,P) A(t)*P + P*A(t)' - P*C(t)'*(W(t)\(C(t)*P)) + V(t);
P_dot_mat = P_dot(t,P);
dx = reshape(P_dot_mat,[],1);
end