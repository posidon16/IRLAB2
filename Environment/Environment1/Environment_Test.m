clf;
clc;
clear

axis([-2,2,-2.5,2,-0.05,3]);
view(120,30);
camlight(0,15);
hold on

PlaceObject('Cafe_Counter7.ply', [0.5, -0.8, 0.1]);
PlaceObject('Cafe_Back_Counter5.ply', [-1.2, -0.2, 0.1]);
PlaceObject('Cafe_Counter_Middle3.PLY',[0.5,-0.8,0.1]);
PlaceObject('fireExtinguisher2.ply', [-1.4, -2, 1.45]);
PlaceObject('eStop4.ply', [0.2, 0.3, 1.3]);
PlaceObject('tempFence7r.PLY',[0.25,-1.6,0.075]);
PlaceObject('Siren3.PLY',[0.2,-0.5,1.3]);
PlaceObject('Siren4.PLY',[0.2,-0.2,1.3]);
PlaceObject('Light2.PLY',[0.45,0.1,1.2]);
PlaceObject('Light2.PLY',[0.45,-0.7,1.2]);
PlaceObject('Food_Tray_Red2.PLY',[0,0.8,1.3]);
PlaceObject('Food_Tray_Green2.PLY',[-0.3,1.2,1.3]);
PlaceObject('Food_Tray_Blue2.PLY',[-0.7,1.5,1.3]);
surf([-2,-2;2,2],[-2.5,2;-2.5,2],[0,0;0,0],'CData',imread('check3.jpg'),'FaceColor','texturemap');
surf([-2,-2;-2,-2],[2,-2.5;2,-2.5],[3,3;0,0],'CData',imread('Cafe_Wall5.jpg'),'FaceColor','texturemap');
surf([-2,2;-2,2],[-2.5,-2.5;-2.5,-2.5],[3,3;0,0],'CData',imread('Cafe_Wall3.jpg'),'FaceColor','texturemap');
surf([0.5,0.5;0.5,0.5],[-0.6,0;-0.6,0],[1.2,1.2;0.6,0.6],'CData',imread('Robot_Warning.jpg'),'FaceColor','texturemap');


DobotBaseTransform = transl(-1.6, -0.35, 1.25) * rpy2tr(0,0,0);
s = DobotMagician(DobotBaseTransform);

pickupPoint_global = transl(-1.6, 0.6525, 1.27) * rpy2tr(0, 0, 0);
dropoffPoint_global = transl(-1.6, 0.6525, 1.435) * rpy2tr(0, 0, 0);


pickupPoint_dobot = (DobotBaseTransform - pickupPoint_global) + eye(4)
%pickupPoint_dobot2 = eye(4) + pickupPoint_dobot

% defaultRealQ  = [0,pi/4,pi/4,0,0];
T_0A = (transl(-1.6, -0.6525, 1.27));
trplot(T_0A, 'frame', 'T_A', 'color', 'g', 'length', 0.5);

sq{01} = [-pi/2, pi/4, pi/4, pi/2, 0];
sq{02} = s.model.ikcon(T_0A);
%sq{02} = s.model.ikcon(transl(-1.6, -0.6525, 1.27));
% sq{02} = s.model.ikcon(transl(0, 0.2, 0.2) * rpy2tr(0, 0, -pi/2));

sq{03} = sq{01};
sq{04} = [0,pi/4,pi/4,pi/2,0];
% sq{05} = s.model.ikcon(dropoffPoint);

for i = 1:2
% for i = 1:width(sq)
    newQ = [sq{i}];    
    jointTrajectory = jtraj(s.model.getpos(),sq{i}, 300);
        for trajStep = 1:size(jointTrajectory,1)
            s.model.animate(jointTrajectory(trajStep,:));
            drawnow();
        end
    currentPostion = s.model.fkine(s.model.getpos)
end



% cowHerd = RobotCows1();
% % 2.2 Check how many cow there are
% cowHerd.cowCount
% % 2.3 Plot on single iteration of the random step movement
% cowHerd.PlotSingleRandomStep();
% 
% % input('Finished question 2.3, press enter to continue')
% % 
% % % 2.4 Clear then create another instance with 10 cows
% % clf;
% try delete(cowHerd); end 
% cowHerd = RobotCows(1);
% % 2.5 Test many random steps
% numSteps = 100;
% delay = 0.01;
% cowHerd.TestPlotManyStep(numSteps,delay);
% % 2.6 Query the location of the 2nd cow 
% cowHerd.cowModel{1}.base
