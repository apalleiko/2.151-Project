function [] = animate_traj(robot,q,rate)
% Animate trajectory
count = size(q,2);
show(robot,q(:,1));
xlim([-.01,.01])
view(90,0)
while true
    for i = 1:count
        show(robot,q(:,i),'PreservePlot',false);
        xlim([-.01,.01])
        view(90,0)
        drawnow
        pause(rate);
    end
    pause(0.5);
end
end

