clc
clf
clear all
% r = DobotGripper
r = DobotMagician
p3 = DobotGripper
% q = r.model.getpos();

axis([-0.2,0.4,-0.2,0.4,0,0.4])

% r.model.teach(q)
steps = 100;

% for i = 1:9
% robot_pos = [r.model.getpos];  
% qMatrix = jtraj(robot_pos,steps);
% for j = 1:1:steps
    % r.model.animate(qMatrix(j,:))
    pos = r.model.getpos;
    T = r.model.fkine(pos)
    p3.model.base = T;
    p3.model.animate(p3.model.getpos);
% end
% end