% springDamperSystem

ltvSys = ltvss(@LTV);
x0 = [1;0];

respOpt = RespConfig(InitialState=x0,Delay=1);
t = (0:.01:10)';
step(ltvSys,t,respOpt)

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