clf;
clc;
clear

axis([-2,2,-2.5,2,-0.05,3]);
view(120,30);
camlight(0,15);
hold on

PlaceObject('Curved_Counter_6_Meshlab.PLY', [0.5, -0.8, 0.1]);
PlaceObject('Cafe_Back_Counter5.ply', [-1.2, -0.2, 0.1]);
PlaceObject('Cafe_Counter_Middle3.PLY',[0.5,-0.8,0.1]);
PlaceObject('fireExtinguisher2.ply', [-1.4, -2, 1.45]);
PlaceObject('eStop4.ply', [0.3, -0.3, 1.3]);
PlaceObject('Light_Curtain.PLY',[0.25,-0.88,0]);
PlaceObject('Siren3.PLY',[0,-0.5,1.3]);
PlaceObject('Siren4.PLY',[0,-0.2,1.3]);
PlaceObject('Light2.PLY',[0.45,0.1,1.2]);
PlaceObject('Light2.PLY',[0.45,-0.7,1.2]);
redTray = PlaceObject('Food_Tray_Red2.PLY',[0.04,0.6,1.29]);
greenTray = PlaceObject('Food_Tray_Green2.PLY',[-0.2,1.2,1.3]);
blueTray = PlaceObject('Food_Tray_Blue2.PLY',[-0.8,1.4,1.3]);
redMuffin = PlaceObject('Red_muffin.PLY',[-1.6, -0.01, 1.29]);
chef = PlaceObject('Human_with_Chefs_hat.PLY',[1,-1.5,0]);
surf([-2,-2;2,2],[-2.5,2;-2.5,2],[0,0;0,0],'CData',imread('check3.jpg'),'FaceColor','texturemap');
surf([-2,-2;-2,-2],[2,-2.5;2,-2.5],[3,3;0,0],'CData',imread('Cafe_Wall5.jpg'),'FaceColor','texturemap');
surf([-2,2;-2,2],[-2.5,-2.5;-2.5,-2.5],[3,3;0,0],'CData',imread('Cafe_Wall3.jpg'),'FaceColor','texturemap');
surf([0.5,0.5;0.5,0.5],[-0.6,0;-0.6,0],[1.2,1.2;0.6,0.6],'CData',imread('Robot_Warning.jpg'),'FaceColor','texturemap');

% % ptCloud = pcread('teapot.ply');
% % pcshow(ptCloud);

DobotBaseTransform = transl(-1.6, 0.3, 1.25) * rpy2tr(0,0,0);
s = DobotMagician(DobotBaseTransform);

GripperTransform = transl(-1.6, 0.3, 1.25) * rpy2tr(0,0,0);
p3 = DobotGripper(GripperTransform)

MitsubushiTransform = transl(-0.8, 0.6, 1.25) * rpy2tr(0,0,0);
r = Mitsubishi(MitsubushiTransform);


pickupPoint_global = transl(-1.6, 0.6525, 1.27) * rpy2tr(0, 0, 0);
dropoffPoint_global = transl(-1.6, 0.6525, 1.435) * rpy2tr(0, 0, 0);

pos = r.model.getpos
 r.model.teach(pos)


startPos = r.model.ikcon(transl(-0.4, 0.566, 1.622)*rpy2tr(-pi/2,-pi/2,-pi/2), [-0.0838    1.1729   -0.3281         0   -0.7540         0]);

redTraySlide = r.model.ikcon(transl(-0.485, 0.6, 1.385)*rpy2tr(-pi/2,-pi/2,-pi/2), [0    0.5027   -0.5515         0   -1.5708         0]); 


redTrayPick = r.model.ikcon(transl(-0.270, 0.6, 1.364)*rpy2tr(pi/2,pi/2,pi/2), [0   -0.1257    0.9006         0   -0.8029         0]);
% 0   -7.2021   51.6006         0  -46.0028         0

redTrayAbove = r.model.ikcon(transl(-0.219, 0.6, 1.539)*rpy2tr(pi/2,pi/2,pi/2), [0   -0.0314    1.4033         0   -0.2094         0]); 

redTrayDropAbove = r.model.ikcon(transl(-1.379, 0.6, 1.579)*rpy2tr(-pi/2,-pi/2,-pi/2), [ 3.1416   -0.0419    1.5704    0   -0.0419    0]);

redTrayDrop = r.model.ikcon(transl(-1.342, 0.6, 1.36)*rpy2tr(-pi/2,-pi/2,-pi/2), [3.1416   -0.4887    1.5708    0   -0.5027         0]); 
%   180.0004  -28.0004   90.0002         0  -28.8026         0

redTraySlideUp = r.model.ikcon(transl(-0.518, 0.6, 1.504)*rpy2tr(-pi/2,-pi/2,-pi/2), [0    1.1310   -0.9146         0   -1.3404         0]); 
 
% % % greenDropPoint = 0.7854  -0.1676    1.0123         0   -0.7121         0

% % % pickPointALL =     3.1416   -0.4887    1.5708         0   -0.5027         0

% % % redDropPoint =          0   -0.1571    1.0123         0   -0.7121         0

% % % blueDropPoint =  1.5708   -0.1571    1.0123         0   -0.7121         0

%%% XYZ LOCATION OF TRAYS [-1.6,0.6,1.3]);

% % % ROLL AND YAW WILL STAY AT 90 or -90 AS ARM IS TURNING



% pos = s.model.getpos
%  s.model.teach(pos)

% pickupPoint_dobot = (DobotBaseTransform - pickupPoint_global) + eye(4)
%pickupPoint_dobot2 = eye(4) + pickupPoint_dobot

% defaultRealQ  = [0,pi/4,pi/4,0,0];
% T_0A = (transl(-1.6, -0.6525, 1.27));
% trplot(T_0A, 'frame', 'T_A', 'color', 'g', 'length', 0.5);
% 
% sq{01} = [-pi/2, pi/4, pi/4, pi/2, 0];
% sq{02} = s.model.ikcon(T_0A);
% %sq{02} = s.model.ikcon(transl(-1.6, -0.6525, 1.27));
% % sq{02} = s.model.ikcon(transl(0, 0.2, 0.2) * rpy2tr(0, 0, -pi/2));
% 
% sq{03} = sq{01};
% sq{04} = [0,pi/4,pi/4,pi/2,0];
% % sq{05} = s.model.ikcon(dropoffPoint);
% 
% for i = 1:2
% % for i = 1:width(sq)
%     newQ = [sq{i}];    
%     jointTrajectory = jtraj(s.model.getpos(),sq{i}, 300);
%         for trajStep = 1:size(jointTrajectory,1)
%             s.model.animate(jointTrajectory(trajStep,:));
%             drawnow();
%         end
%     currentPostion = s.model.fkine(s.model.getpos)
% end



steps = 100;

robot_pos = [r.model.getpos];  
qMatrixPlace = jtraj(robot_pos,startPos,steps);


r.model.fkine(robot_pos)
verts = get(redTray, 'Vertices');

for j = 1:1:steps
    r.model.animate(qMatrixPlace(j,:))   % animate the UR3 to animate through the q Matrix
    pos = r.model.getpos;                   % get pos of UR3
    T = r.model.fkine(pos);                 % get end affector position of UR3

    drawnow();
end

robot_pos = [r.model.getpos];  
qMatrixPlace = jtraj(robot_pos,redTraySlide,steps);


for j = 1:1:steps
    r.model.animate(qMatrixPlace(j,:))   % animate the UR3 to animate through the q Matrix
    pos = r.model.getpos;                   % get pos of UR3
    T = r.model.fkine(pos);                 % get end affector position of UR3

    drawnow();

end

robot_pos = [r.model.getpos];
qMatrixPlace = jtraj(robot_pos,redTrayPick,steps);


for j = 1:1:steps
    r.model.animate(qMatrixPlace(j,:))   % animate the UR3 to animate through the q Matrix
    pos = r.model.getpos;                   % get pos of UR3
    T = r.model.fkine(pos);                 % get end affector position of UR3

    drawnow();
end
robot_pos = [r.model.getpos];
qMatrixPlace = jtraj(robot_pos,redTrayAbove,steps);

for j = 1:1:steps
    r.model.animate(qMatrixPlace(j,:))   % animate the UR3 to animate through the q Matrix
    pos = r.model.getpos;                   % get pos of UR3
    T = r.model.fkine(pos);                 % get end affector position of UR3

    transformedVertices = [verts,ones(size(verts,1),1)] * (T.T * transl(-1.36,-0.6,0.35)*rpy2tr(0,pi/2,0))';
    set(redTray,'Vertices',transformedVertices(:,1:3));
    drawnow();
end

robot_pos = [r.model.getpos];
qMatrixPlace = jtraj(robot_pos,redTrayDrop,steps);



for j = 1:1:steps
    r.model.animate(qMatrixPlace(j,:))   % animate the UR3 to animate through the q Matrix
    pos = r.model.getpos;                   % get pos of UR3
    T = r.model.fkine(pos);                 % get end affector position of UR3

    transformedVertices = [verts,ones(size(verts,1),1)] * (T.T * transl(-1.36,-0.6,0.35)*rpy2tr(0,pi/2,0))';
    set(redTray,'Vertices',transformedVertices(:,1:3));
    drawnow();
end

% Start loading muffins

redMuffinPick = s.model.ikcon(transl(-1.6, -0.009, 1.398)*rpy2tr(0,0,-pi/2), [-1.5708    1.0210    0.9809    1.1432]); 
% -90.0002   58.4990   56.2014   65.5005

redMuffinPlace = s.model.ikcon(transl(-1.600, 0.562, 1.439)*rpy2tr(0,0,pi/2), [1.5709    0.5498    1.4835    1.0996]); 

redMuffinAbovePlace = s.model.ikcon(transl(-1.600, 0.552, 1.539)*rpy2tr(0,0,pi/2), [1.5708    0.3534    1.0594    1.7279]); 

redMuffinAbovePick = s.model.ikcon(transl(-1.600, 0.026, 1.517)*rpy2tr(0,0,pi/2), [-1.5708    0.5236    0.9809    1.6232]); 



robot_pos = [s.model.getpos];  
qMatrixRedMuffin = jtraj(robot_pos,redMuffinAbovePick,steps);


s.model.fkine(robot_pos)
muffinVerts = get(redMuffin, 'Vertices');

for j = 1:1:steps
    s.model.animate(qMatrixRedMuffin(j,:))   % animate the UR3 to animate through the q Matrix
    pos = s.model.getpos;                   % get pos of UR3
    T = s.model.fkine(pos);                 % get end affector position of UR3
    p3.model.base = T;                            % assign the base of the gripper
    p3.model.animate(p3.model.getpos);                  %animate the gripper

    drawnow();
end


robot_pos = [s.model.getpos];  
qMatrixRedMuffin = jtraj(robot_pos,redMuffinPick,steps);

for j = 1:1:steps
    s.model.animate(qMatrixRedMuffin(j,:))   % animate the UR3 to animate through the q Matrix
    pos = s.model.getpos;                   % get pos of UR3
    T = s.model.fkine(pos);                 % get end affector position of UR3
    p3.model.base = T;                            % assign the base of the gripper
    p3.model.animate(p3.model.getpos);                  %animate the gripper

    drawnow();
end


robot_pos = [s.model.getpos];  
qMatrixRedMuffin = jtraj(robot_pos,redMuffinAbovePlace,steps);

for j = 1:1:steps
    s.model.animate(qMatrixRedMuffin(j,:))   % animate the UR3 to animate through the q Matrix
    pos = s.model.getpos;                   % get pos of UR3
    T = s.model.fkine(pos);                 % get end affector position of UR3
    p3.model.base = T;                            % assign the base of the gripper
    p3.model.animate(p3.model.getpos);                  %animate the gripper
    %%transformedVertices = [verts,ones(size(verts,1),1)] * (T.T * transl(-1.37,-0.6,0.35)*rpy2tr(0,pi/2,0))';
    muffinTransform = [muffinVerts,ones(size(muffinVerts,1),1)] * (T.T * transl(1.6, 0.01, -1.395)*rpy2tr(0,0,0))';
    set(redMuffin,'Vertices',muffinTransform(:,1:3));
    drawnow();
end

robot_pos = [s.model.getpos];  
qMatrixRedMuffin = jtraj(robot_pos,redMuffinPlace,steps);

for j = 1:1:steps
    s.model.animate(qMatrixRedMuffin(j,:))   % animate the UR3 to animate through the q Matrix
    pos = s.model.getpos;                   % get pos of UR3
    T = s.model.fkine(pos);                 % get end affector position of UR3
    p3.model.base = T;                            % assign the base of the gripper
    p3.model.animate(p3.model.getpos);                  %animate the gripper
    %%transformedVertices = [verts,ones(size(verts,1),1)] * (T.T * transl(-1.37,-0.6,0.35)*rpy2tr(0,pi/2,0))';
    muffinTransform = [muffinVerts,ones(size(muffinVerts,1),1)] * (T.T * transl(1.6, 0.01, -1.395)*rpy2tr(0,0,0))';
    set(redMuffin,'Vertices',muffinTransform(:,1:3));
    drawnow();
end

robot_pos = [s.model.getpos];  
qMatrixRedMuffin = jtraj(robot_pos,redMuffinAbovePlace,steps);

for j = 1:1:steps
    s.model.animate(qMatrixRedMuffin(j,:))   % animate the UR3 to animate through the q Matrix
    pos = s.model.getpos;                   % get pos of UR3
    T = s.model.fkine(pos);                 % get end affector position of UR3
    p3.model.base = T;                            % assign the base of the gripper
    p3.model.animate(p3.model.getpos);                  %animate the gripper

    drawnow();
end

robot_pos = [s.model.getpos];  
qMatrixRedMuffin = jtraj(robot_pos,redMuffinAbovePick,steps);

for j = 1:1:steps
    s.model.animate(qMatrixRedMuffin(j,:))   % animate the UR3 to animate through the q Matrix
    pos = s.model.getpos;                   % get pos of UR3
    T = s.model.fkine(pos);                 % get end affector position of UR3
    p3.model.base = T;                            % assign the base of the gripper
    p3.model.animate(p3.model.getpos);                  %animate the gripper

    drawnow();
end

% resume tray
% redTrayPlace = PlaceObject('Food_Tray_Red2.PLY',[-1.65, 0.6, 1.3]);


robot_pos = [r.model.getpos];
qMatrixPlace = jtraj(robot_pos,redTrayDropAbove,steps);

% % r.model.fkine(robot_pos)
% % verts = get(redTray, 'Vertices');


for j = 1:1:steps
    r.model.animate(qMatrixPlace(j,:))   % animate the UR3 to animate through the q Matrix
    pos = r.model.getpos;                   % get pos of UR3
    T = r.model.fkine(pos);                 % get end affector position of UR3

    transformedVertices = [verts,ones(size(verts,1),1)] * (T.T * transl(-1.36,-0.6,0.35)*rpy2tr(0,pi/2,0))';  %
    set(redTray,'Vertices',transformedVertices(:,1:3));
    muffinTransform = [muffinVerts,ones(size(muffinVerts,1),1)] * (T.T * transl(-1.32,-0.03,-1.34)*rpy2tr(0,pi/2,0))';
    set(redMuffin,'Vertices',muffinTransform(:,1:3))
    drawnow();
end

robot_pos = [r.model.getpos];
qMatrixPlace = jtraj(robot_pos,redTrayAbove,steps);



for j = 1:1:steps
    r.model.animate(qMatrixPlace(j,:))   % animate the UR3 to animate through the q Matrix
    pos = r.model.getpos;                   % get pos of UR3
    T = r.model.fkine(pos);                 % get end affector position of UR3

    transformedVertices = [verts,ones(size(verts,1),1)] * (T.T * transl(-1.36,-0.6,0.35)*rpy2tr(0,pi/2,0))';
    set(redTray,'Vertices',transformedVertices(:,1:3));
    muffinTransform = [muffinVerts,ones(size(muffinVerts,1),1)] * (T.T * transl(-1.32,-0.03,-1.34)*rpy2tr(0,pi/2,0))';
    set(redMuffin,'Vertices',muffinTransform(:,1:3))
    drawnow();
end


robot_pos = [r.model.getpos];
qMatrixPlace = jtraj(robot_pos,redTrayPick,steps);



for j = 1:1:steps
    r.model.animate(qMatrixPlace(j,:))   % animate the UR3 to animate through the q Matrix
    pos = r.model.getpos;                   % get pos of UR3
    T = r.model.fkine(pos);                 % get end affector position of UR3

    transformedVertices = [verts,ones(size(verts,1),1)] * (T.T * transl(-1.36,-0.6,0.35)*rpy2tr(0,pi/2,0))';
    set(redTray,'Vertices',transformedVertices(:,1:3));
    muffinTransform = [muffinVerts,ones(size(muffinVerts,1),1)] * (T.T * transl(-1.32,-0.03,-1.34)*rpy2tr(0,pi/2,0))';
    set(redMuffin,'Vertices',muffinTransform(:,1:3))
    drawnow();
end

robot_pos = [r.model.getpos];
qMatrixPlace = jtraj(robot_pos,redTraySlide,steps);


for j = 1:1:steps
    r.model.animate(qMatrixPlace(j,:))   % animate the UR3 to animate through the q Matrix
    pos = r.model.getpos;                   % get pos of UR3
    T = r.model.fkine(pos);                 % get end affector position of UR3
    drawnow();
end


robot_pos = [r.model.getpos];
qMatrixPlace = jtraj(robot_pos,redTraySlideUp,steps);



for j = 1:1:steps
    r.model.animate(qMatrixPlace(j,:))   % animate the UR3 to animate through the q Matrix
    pos = r.model.getpos;                   % get pos of UR3
    T = r.model.fkine(pos);                 % get end affector position of UR3
    % p3.base = T;                            % assign the base of the gripper
    % p3.animate(p3.getpos);                  %animate the gripper

    drawnow();
end
%% 


            % % for i = 1:10
            % %     % uavTRStart = chef;
            % %     % uavTRGoal = chef * transl(0,0,5);
            % %     % tranimate(uavTRStart,uavTRGoal,'fps',100);
            % %     moveChef = get(chef, 'Vertices');
            % %     transformedVertices = [moveChef,ones(size(moveChef,1),1)] * transl(i,-1.5,0);
            % %     set(chef,'Vertices',transformedVertices(:,1:3));
            % %     drawnow()
            % % end

% 
% redMuffinPick = s.model.ikcon(transl(-1.6, -0.009, 1.398)*rpy2tr(0,0,-pi/2), [-1.5708    1.0210    0.9809    1.1432]); 
% 
% redMuffinPlace = s.model.ikcon(transl(-1.557, 0.569, 1.393)*rpy2tr(0,0,pi/2), [1.4137    0.7854    1.4364    0.9547]); 
% 
% 
% 
% 
% 
% 
% robot_pos = [s.model.getpos];  
% qMatrixRedMuffin = jtraj(robot_pos,redMuffinPick,steps);
% 
% 
% s.model.fkine(robot_pos)
% verts = get(redMuffin, 'Vertices');
% 
% for j = 1:1:steps
%     s.model.animate(qMatrixRedMuffin(j,:))   % animate the UR3 to animate through the q Matrix
%     pos = s.model.getpos;                   % get pos of UR3
%     T = s.model.fkine(pos);                 % get end affector position of UR3
%     p3.model.base = T;                            % assign the base of the gripper
%     p3.model.animate(p3.model.getpos);                  %animate the gripper
% 
%     drawnow();
% end
% 
% robot_pos = [s.model.getpos];  
% qMatrixRedMuffin = jtraj(robot_pos,redMuffinPlace,steps);
% 
% for j = 1:1:steps
%     s.model.animate(qMatrixRedMuffin(j,:))   % animate the UR3 to animate through the q Matrix
%     pos = s.model.getpos;                   % get pos of UR3
%     T = s.model.fkine(pos);                 % get end affector position of UR3
%     p3.model.base = T;                            % assign the base of the gripper
%     p3.model.animate(p3.model.getpos);                  %animate the gripper
% 
%     drawnow();
% end
