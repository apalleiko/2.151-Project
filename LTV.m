function [A,B,C,D,E,dx0,x0,u0,y0,Delays] = LTV(t)
% SISO, first order
m = 1 + 0.1*sin(t);  % Mass varying with time
c = 0.5 + 0.2*cos(t);  % Damping coefficient varying with time
k = 2 + 0.3*sin(2*t);  % Spring constant varying with time

% State-space matrices
A = [0 1; -k/m -c/m];
B = [0; 1/m];
C = eye(2);
D = zeros(2, 1);

E = [];
dx0 = [];
x0 = [];
u0 = [];
y0 = [0;0];
Delays = [];