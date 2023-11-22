function [Wc,Wo] = WcWo(A,B,C,t_span)

% Define the state transition matrix integrator
% this needs to be checked
Phi_integrator = @(t) expm(A(t)*(t_span(2) - t));
% Phi_integrator = @(t) integral(expm(A(t)), t_span(1), t, 'ArrayValued', true);


% Define the controllability Gramian integrator
Q_integrator = @(t) Phi_integrator(t) * B(t) * B(t)' * Phi_integrator(t)';

% Compute the controllability Gramian using the integral over time
Wc = integral(Q_integrator, t_span(1), t_span(2), 'ArrayValued', true);

% Display or use the resulting controllability Gramian matrix Wc
disp('Controllability Gramian:');
disp(Wc);

W0_integrator = @(t) Phi_integrator(t)' * C(t)' * C(t) * Phi_integrator(t);

% Compute the observability Gramian using the integral over time
Wo = integral(W0_integrator, t_span(1), t_span(2), 'ArrayValued', true);

% Display or use the resulting observability Gramian matrix Wo
disp('Observability Gramian:');
disp(Wo);
if all(eig(Wc)>0)
    disp("Controllable")
else
    disp("Not Controllable")
end

if all(eig(Wo)>0)
    disp("Observable")
else
    disp("Not Observable")
end

end