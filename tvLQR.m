function K_t = tvLQR(A,B,Q,R)
% pseudocode for the tvLQR

% to be changed
forwardKinematics = @(x) [cos(x), -sin(x); sin(x), cos(x)];  

tspan = linspace(0, 1, 100);  

for i = 1:length(tspan)
    t = tspan(i);
    A_t = A(t);
    B_t = B(t);
    
    x = someFunctionToGetRobotConfiguration(t); 
    
    J = forwardKinematics(x);
    
    A_augmented = [A_t, zeros(size(A_t)); J, zeros(size(J))];
    B_augmented = [B_t; zeros(size(J, 1), 1)];
    
    % i should use icare will update 
    P_t = icare(A_augmented, B_augmented, Q, R);
    
    K_t = -B_augmented' * P_t;
    
end
end
