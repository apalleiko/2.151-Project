% springDamperSystem
clear;
ltvSys = ltvss(@LTV);
x0 = [1;0];



% respOpt = RespConfig(InitialState=x0,Delay=1);
% t = (0:.01:tspan)';
% step(ltvSys,t,respOpt)
% tspan = 10;
% t = (0:.01:tspan)';
% Q = [1 0;0 0];
% R = 1;
% A = zeros(2,2,size(t,1));
% B = zeros(2,1,size(t,1));
% C = zeros(2,2,size(t,1));
% D = zeros(2,1,size(t,1));


% count = 1;
% for i = t'
% [Ai,Bi,~,~,~,~,~,~,~,~] = LTV(i);
% A(:,:,count) = Ai;
% B(:,:,count) = Bi;
% count = count + 1;
% 
% end

m = @(t) 1 + 0.1*sin(t);  % Mass varying with time
c = @(t) 0.5 + 0.2*cos(t);  % Damping coefficient varying with time
k = @(t) 2 + 0.3*sin(2*t);  % Spring constant varying with time

% State-space matrices
A = @(t) [0 1; -1*k(t)/m(t) -1*c(t)/m(t)];
B = @(t) [0; 1/m(t)];
C = @(t) [0 1];

Q = @(t) [1/0.1^2 0;0 0];
R = @(t) 1;
tspan = [0, 100];
[Wc,Wo] = WcWo(A,B,C,tspan);

% C = @(t) eye(2);
% D = zeros(2, 1);
% tspan = (0:.1:1);
% K = tvLQR(A, B, Q, R, tspan);
% % K = tvLQR(A, B, Q, R, tspan);
% for i = 1:length(tspan)
%         t = tspan(i);
%         % pole(ss(A(t),B(t),[1 0],0))
%         pzmap(ss(A(t),B(t),[1 0],0))
%         hold on
% end
function springDamperSystem
    m = 1;      
    k0 = 1;     
    c0 = 0.1;   


    tspan = [0 100];
    
    odeFcn = @(t, y) springDamperODE(t, y, k0, c0,m);

    [t, y] = ode45(odeFcn, tspan, [0.1; 0]);

    % figure;
    % subplot(2,1,1);
    % plot(t, y(:, 1), 'LineWidth', 2);
    % title('Displacement vs Time');
    % xlabel('Time');
    % ylabel('Displacement');
    % 
    % subplot(2,1,2);
    % plot(t, y(:, 2), 'LineWidth', 2);
    % title('Velocity vs Time');
    % xlabel('Time');
    % ylabel('Velocity');
end

function dydt = springDamperODE(t, y, k0, c0,m)
    k = k0 * (1 + 5 * sin(2 * pi * t));
    c = c0 * (1 + 5 * cos(2 * pi * t));

    x = y(1); 
    v = y(2); 

    dxdt = v;
    dvdt = -(k / m) * x - (c / m) * v;

    dydt = [dxdt; dvdt];
end