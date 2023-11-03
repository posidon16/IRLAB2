clear all
clf
clc
hold on

%% Create Robots:
a = arduino('Com6', 'uno', 'Libraries', 'Ultrasonic');
UltrasonicSensor = ultrasonic(a, 'A0', 'A1');
s = DobotMagician;
r = Mitsubishi;
g = DobotGripper;

% Plot Dobot in position
Dobot_Transform = transl(-1.6, 0.3, 1.25) * rpy2tr(0,0,0);
s.model.base = Dobot_Transform;
s.model.animate(Dobot_Transform)

% Plot Mitsubishi in position
MitsubushiTransform = transl(-0.8, 0.6, 1.25) * rpy2tr(0,0,0);
r.model.base = MitsubushiTransform;
r.model.animate(r.model.getpos)

% Plot Dobot gripper in position
GripperTransform = transl(-1.6, 0.3, 1.25) * rpy2tr(0,0,0);
g.model.base = GripperTransform;
g.model.animate(g.model.getpos)


%% Create Environment
% Set up camera stuff
axis([-2,2,-2.5,2,-0.05,3]);
view(120,30);
camlight(0,15);
hold on

% Spawn static objects
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

% Spawn images
surf([-2,-2;2,2],[-2.5,2;-2.5,2],[0,0;0,0],'CData',imread('check3.jpg'),'FaceColor','texturemap');
surf([-2,-2;-2,-2],[2,-2.5;2,-2.5],[3,3;0,0],'CData',imread('Cafe_Wall5.jpg'),'FaceColor','texturemap');
surf([-2,2;-2,2],[-2.5,-2.5;-2.5,-2.5],[3,3;0,0],'CData',imread('Cafe_Wall3.jpg'),'FaceColor','texturemap');
surf([0.5,0.5;0.5,0.5],[-0.6,0;-0.6,0],[1.2,1.2;0.6,0.6],'CData',imread('Robot_Warning.jpg'),'FaceColor','texturemap');


%% Define the red q_Matrix
% NOTE: BOTH Q-MATRICES SHOULD BE THE SAME SIZE TO RUN PROPERLY
% Create the q-matrix of Dobot waypoints
q_s_red{1} = s.model.ikcon(transl(-1.600, 0.026, 1.517)*rpy2tr(0,0,pi/2), [-1.5708    0.5236    0.9809    1.6232]); 
q_s_red{2} = s.model.ikcon(transl(-1.6, -0.009, 1.398)*rpy2tr(0,0,-pi/2), [-1.5708    1.0210    0.9809    1.1432]); 
q_s_red{3} = q_s_red{1}; % holding first red muffin
q_s_red{4} = q_s_red{1};
q_s_red{5} = q_s_red{1};
q_s_red{6} = q_s_red{1};
q_s_red{7} = s.model.ikcon(transl(-1.600, 0.552, 1.539)*rpy2tr(0,0,pi/2), [1.5708    0.3534    1.0594    1.7279]);
q_s_red{8} = s.model.ikcon(transl(-1.600, 0.562, 1.439)*rpy2tr(0,0,pi/2), [1.5709    0.5498    1.4835    1.0996]); 
q_s_red{9} = s.model.ikcon(transl(-1.600, 0.552, 1.539)*rpy2tr(0,0,pi/2), [1.5708    0.3534    1.0594    1.7279]);
q_s_red{10} = q_s_red{1};
q_s_red{11} = q_s_red{1};
q_s_red{12} = q_s_red{1};
q_s_red{13} = q_s_red{1};
q_s_red{14} = q_s_red{1};
q_s_red{15} = q_s_red{1};

% Create the q-matrix of Mitsubishi waypoints
q_r_red{1} = r.model.ikcon(transl(-0.4, 0.566, 1.622)*rpy2tr(-pi/2,-pi/2,-pi/2), [-0.0838    1.1729   -0.3281         0   -0.7540         0]);
q_r_red{2} = r.model.ikcon(transl(-0.485, 0.6, 1.385)*rpy2tr(-pi/2,-pi/2,-pi/2), [0    0.5027   -0.5515         0   -1.5708         0]); 
q_r_red{3} = r.model.ikcon(transl(-0.270, 0.6, 1.364)*rpy2tr(pi/2,pi/2,pi/2), [0   -0.1257    0.9006         0   -0.8029         0]);
q_r_red{4} = r.model.ikcon(transl(-0.219, 0.6, 1.539)*rpy2tr(pi/2,pi/2,pi/2), [0   -0.0314    1.4033         0   -0.2094         0]); 
q_r_red{5} = r.model.ikcon(transl(-1.379, 0.6, 1.579)*rpy2tr(-pi/2,-pi/2,-pi/2), [ 3.1416   -0.0419    1.5704    0   -0.0419    0]);
q_r_red{6} = r.model.ikcon(transl(-1.342, 0.6, 1.36)*rpy2tr(-pi/2,-pi/2,-pi/2), [3.1416   -0.4887    1.5708    0   -0.5027         0]); 
q_r_red{7} = q_r_red{6}; % loading red muffins
q_r_red{8} = q_r_red{6};
q_r_red{9} = q_r_red{6};
q_r_red{10} = q_r_red{6};
q_r_red{11} = q_r_red{5}; % return red tray to 
q_r_red{12} = q_r_red{4};
q_r_red{13} = q_r_red{3};
q_r_red{14} = q_r_red{2}; % pull out spatula from red tray
q_r_red{15} = r.model.ikcon(transl(-0.518, 0.6, 1.504)*rpy2tr(-pi/2,-pi/2,-pi/2), [0    1.1310   -0.9146         0   -1.3404         0]); 


%% Define the blue q_Matrix
% NOTE: BOTH Q-MATRICES SHOULD BE THE SAME SIZE TO RUN PROPERLY
% Create the q-matrix of Dobot waypoints
q_s_blue{1} = s.model.ikcon(transl(-1.600, 0.026, 1.517)*rpy2tr(0,0,pi/2), [-1.5708    0.5236    0.9809    1.6232]); 
q_s_blue{2} = s.model.ikcon(transl(-1.4, -0.009, 1.398)*rpy2tr(0,0,-pi/2), [-1.5708    1.0210    0.9809    1.1432]); 
q_s_blue{3} = q_s_blue{1}; % holding first red muffin
q_s_blue{4} = q_s_blue{1};
q_s_blue{5} = q_s_blue{1};
q_s_blue{6} = q_s_blue{1};
q_s_blue{7} = s.model.ikcon(transl(-1.600, 0.552, 1.539)*rpy2tr(0,0,pi/2), [1.5708    0.3534    1.0594    1.7279]);
q_s_blue{8} = s.model.ikcon(transl(-1.600, 0.562, 1.439)*rpy2tr(0,0,pi/2), [1.5709    0.5498    1.4835    1.0996]); 
q_s_blue{9} = s.model.ikcon(transl(-1.600, 0.552, 1.539)*rpy2tr(0,0,pi/2), [1.5708    0.3534    1.0594    1.7279]);
q_s_blue{10} = q_s_blue{1};
q_s_blue{11} = q_s_blue{1};
q_s_blue{12} = q_s_blue{1};
q_s_blue{13} = q_s_blue{1};
q_s_blue{14} = q_s_blue{1};
q_s_blue{15} = q_s_blue{1};

% Create the q-matrix of Mitsubishi waypoints
q_r_blue{1} = r.model.ikcon(transl(-1.379, 0.6, 1.579)*rpy2tr(-pi/2,-pi/2,-pi/2), [ 3.1416   -0.0419    1.5704    0   -0.0419    0]);
q_r_blue{2} = r.model.ikcon(transl(-0.8, 1.181, 1.539)*rpy2tr(-pi/2,-pi/2,0), [pi/2   -0.0314    1.4033         0   -0.2094         0]);
q_r_blue{3} = r.model.ikcon(transl(-0.8, 1.130, 1.364)*rpy2tr(-pi/2,-pi/2,0), [pi/2   -0.1257    0.9006         0   -0.8029         0]);
q_r_blue{4} = r.model.ikcon(transl(-0.8, 0.915, 1.385)*rpy2tr(-pi/2,-pi/2,0), [pi/2    0.5027   -0.5515         0   -1.5708         0]);
q_r_blue{5} = r.model.ikcon(transl(-0.8, 0.882, 1.504)*rpy2tr(-pi/2,-pi/2,0), [ pi/2   1.1248   -0.9130    0   -1.3589   0]);
q_r_blue{6} = r.model.ikcon(transl(-1.342, 0.6, 1.36)*rpy2tr(-pi/2,-pi/2,-pi/2), [3.1416   -0.4887    1.5708    0   -0.5027         0]); 
q_r_blue{7} = q_r_blue{6}; % loading red muffins
q_r_blue{8} = q_r_blue{6};
q_r_blue{9} = q_r_blue{6};
q_r_blue{10} = q_r_blue{6};
q_r_blue{11} = q_r_blue{5}; % return red tray to 
q_r_blue{12} = q_r_blue{4};
q_r_blue{13} = q_r_blue{3};
q_r_blue{14} = q_r_blue{2}; % pull out spatula from red tray
q_r_blue{15} = r.model.ikcon(transl(-0.518, 0.6, 1.504)*rpy2tr(-pi/2,-pi/2,-pi/2), [0    1.1310   -0.9146         0   -1.3404         0]); 



%% Define the green q_Matrix
% NOTE: BOTH Q-MATRICES SHOULD BE THE SAME SIZE TO RUN PROPERLY
% Create the q-matrix of Dobot waypoints
q_s_green{1} = s.model.ikcon(transl(-1.600, 0.026, 1.517)*rpy2tr(0,0,pi/2), [-1.5708    0.5236    0.9809    1.6232]); 
q_s_green{2} = s.model.ikcon(transl(-1.3, -0.009, 1.398)*rpy2tr(0,0,-pi/2), [-1.5708    1.0210    0.9809    1.1432]); 
q_s_green{3} = q_s_green{1}; % holding first red muffin
q_s_green{4} = q_s_green{1};
q_s_green{5} = q_s_green{1};
q_s_green{6} = q_s_green{1};
q_s_green{7} = s.model.ikcon(transl(-1.600, 0.552, 1.539)*rpy2tr(0,0,pi/2), [1.5708    0.3534    1.0594    1.7279]);
q_s_green{8} = s.model.ikcon(transl(-1.600, 0.562, 1.439)*rpy2tr(0,0,pi/2), [1.5709    0.5498    1.4835    1.0996]); 
q_s_green{9} = s.model.ikcon(transl(-1.600, 0.552, 1.539)*rpy2tr(0,0,pi/2), [1.5708    0.3534    1.0594    1.7279]);
q_s_green{10} = q_s_green{1};
q_s_green{11} = q_s_green{1};
q_s_green{12} = q_s_green{1};
q_s_green{13} = q_s_green{1};
q_s_green{14} = q_s_green{1};
q_s_green{15} = q_s_green{1};

% Create the q-matrix of Mitsubishi waypoints
q_r_green{1} = r.model.ikcon(transl(-0.601, 0.799, 1.504)*rpy2tr(-pi/2,-pi/2,-pi/4), [ 0.7854    1.1248   -0.9130    0   -1.3589   0]);
q_r_green{2} = r.model.ikcon(transl(-0.577, 0.823, 1.385)*rpy2tr(-pi/2,-pi/2,-pi/4), [0.7854    0.5027   -0.5515         0   -1.5708         0]);
q_r_green{3} = r.model.ikcon(transl(-0.425, 0.975, 1.364)*rpy2tr(-pi/2,-pi/2,-pi/4), [0.7854   -0.1257    0.9006         0   -0.8029         0]);
q_r_green{4} = r.model.ikcon(transl(-0.389, 1.011, 1.539)*rpy2tr(-pi/2,-pi/2,-pi/4), [0.7854   -0.0314    1.4033         0   -0.2094         0]);
q_r_green{5} = r.model.ikcon(transl(-1.379, 0.6, 1.579)*rpy2tr(-pi/2,-pi/2,-pi/2), [ 3.1416   -0.0419    1.5704    0   -0.0419    0]);
q_r_green{6} = r.model.ikcon(transl(-1.342, 0.6, 1.36)*rpy2tr(-pi/2,-pi/2,-pi/2), [3.1416   -0.4887    1.5708    0   -0.5027         0]); 
q_r_green{7} = q_r_green{6}; % loading red muffins
q_r_green{8} = q_r_green{6};
q_r_green{9} = q_r_green{6};
q_r_green{10} = q_r_green{6};
q_r_green{11} = q_r_green{5}; % return red tray to 
q_r_green{12} = q_r_green{4};
q_r_green{13} = q_r_green{3};
q_r_green{14} = q_r_green{2}; % pull out spatula from red tray
q_r_green{15} = q_r_green{1};


%% 

% Spawn movable objects
redTray = PlaceObject('Food_Tray_Red2.PLY',[0.04,0.6,1.29]);
greenTray = PlaceObject('Food_Tray_Green2.PLY',[-0.2,1.2,1.3]);
blueTray = PlaceObject('Food_Tray_Blue2.PLY',[-0.8,1.4,1.3]);
redMuffin = PlaceObject('Red_muffin.PLY',[-1.6, -0.01, 1.29]);
blueMuffin = PlaceObject('Blue_muffin.PLY',[-1.5, -0.01, 1.29]);
greenMuffin = PlaceObject('Green_muffin.PLY',[-1.4, -0.01, 1.29]);
chef = PlaceObject('Chef3.PLY',[1,-1.5,0]);


% Initialise the status variable for the toggle switch
status = false;

% Define number of steps for each movement
% NOTE THIS MUST BE THE SAME FOR BOTH ROBOTS
steps = 30;


for j = 1 :3
    switch j
        case 1
            q_s = q_s_red;
            q_r = q_r_red;

            robot_pos = [r.model.getpos]; 
            r.model.fkine(robot_pos)
            verts = get(redTray, 'Vertices');
            
            robot_pos = [s.model.getpos]; 
            s.model.fkine(robot_pos)
            muffinVerts = get(redMuffin, 'Vertices');

            colourTray = redTray;
            colourMuffin = redMuffin;

            trayLoc = transl(-1.36,-0.6,0.35)*rpy2tr(0,pi/2,0);
            muffinLoc = transl(1.6, 0.01, -1.395)*rpy2tr(0,0,0);
            muffinFinalLoc = transl(-1.32,-0.03,-1.34)*rpy2tr(0,pi/2,0);
        case 2
            q_s = q_s_blue;
            q_r = q_r_blue;

            colourTray = blueTray;
            colourMuffin = blueMuffin;

            robot_pos = [r.model.getpos]; 
            r.model.fkine(robot_pos)
            verts = get(blueTray, 'Vertices');
            
            robot_pos = [s.model.getpos]; 
            s.model.fkine(robot_pos)
            muffinVerts = get(blueMuffin, 'Vertices');
            
            trayLoc = transl(-1.36,-1.4,-0.525)*rpy2tr(0,pi/2,0);
            muffinLoc = transl(1.5, 0.01, -1.395)*rpy2tr(0,0,0);
            muffinFinalLoc = transl(-1.32,-0.03,-1.14)*rpy2tr(0,pi/2,0);
        case 3
            q_s = q_s_green;
            q_r = q_r_green;

            robot_pos = [r.model.getpos]; 
            r.model.fkine(robot_pos)
            verts = get(greenTray, 'Vertices');
            
            robot_pos = [s.model.getpos]; 
            s.model.fkine(robot_pos)
            muffinVerts = get(greenMuffin, 'Vertices');

            trayLoc = transl(-1.375,-0.713,-0.70)*rpy2tr(pi/2,pi/4,pi/2);
            muffinLoc = transl(1.4, 0.01, -1.395)*rpy2tr(0,0,0);
            muffinFinalLoc = transl(-1.32,-0.03,-1.24)*rpy2tr(0,pi/2,0);
    end


    % Main robot movement loop
    for i = 1:width(q_s)
        % iteration counter
        disp(i);
        if i == 3 || 4 || 5 || 6
            qMatrixRedMuffin = jtraj(robot_pos,q_s{i},steps);
        end
    
        % Calculate the joint trajectory between q matrix points
        newQ_s = q_s{i};
        jointTrajectory_s = jtraj(s.model.getpos(), newQ_s, steps);
        newQ_r = q_r{i};
        jointTrajectory_r = jtraj(r.model.getpos(), newQ_r, steps);
    
       
        % What happens in every step of a movement
        for trajStep = 1:size(jointTrajectory_s, 1)
    
            % Update Dobot gripper location
            pos = s.model.getpos;                   % get pos of UR3
            T = s.model.fkine(pos);                 % get end affector position of UR3
            g.model.base = T;                            % assign the base of the gripper
            g.model.animate(g.model.getpos);
            drawnow();
    
                    % Read the halt button and ultrasonic sensor
                    haltButtonState = readDigitalPin(a, "D13");
                    distance = readDistance(UltrasonicSensor);
                    
                    % if distance is too close or halt button is pressed
                    if haltButtonState == 0
                        disp("Emergency Halt Button Pressed");
                        status = ~status;
                    elseif distance < 0.1
                        status = ~status;
                        disp("Distance too close / Light curtain tripped");
                    end
                    
                    % What happens if the robot is running normally
                    if status == 1
                        writeDigitalPin(a, 'D3', 1); % Show status on LED
                        writeDigitalPin(a, 'D2', 0); % Show status on LED
                        s.model.animate(jointTrajectory_s(trajStep, :));
                        r.model.animate(jointTrajectory_r(trajStep, :));
                        
                        switch i
                            case {4,5,6,7,8, 11, 12, 13}
                                pos = r.model.getpos;
                                T = r.model.fkine(pos); 
                                transformedVertices = [verts,ones(size(verts,1),1)] * (T.T * trayLoc)';
                                set(colourTray,'Vertices',transformedVertices(:,1:3));
                        end
    
                        drawnow();
                        disp("Running as normal");
    
                    % What happens if e-stop is activated    
                    else 
                        disp("E-stop is active");
                        writeDigitalPin(a, 'D3', 0); % Show status on LED
                        writeDigitalPin(a, 'D2', 1); % Show status on LED
    
                        % Checking function to get out of e-stop
                        while 1
                            % Read the halt button and ultrasonic sensor
                            goButtonState = readDigitalPin(a, "D12");
                            haltButtonState = readDigitalPin(a, "D13");
    
                            % if both buttons are pressed at once
                            if goButtonState == 0 && haltButtonState == 0
                                disp("E-stop has been deactivated");
                                status = ~status;
                                pause(0.3) % Debouncer
                                break % Return to Main robot movement loop
                            end
                        end
                    end
    
                    s.model.animate(jointTrajectory_s(trajStep, :));
                    r.model.animate(jointTrajectory_r(trajStep, :));
                    
                    switch i
                        case 3
                            pos = s.model.getpos;                   
                            T = s.model.fkine(pos); 


                            g.model.base = T;                            
                            g.model.animate(g.model.getpos);                  
                            muffinTransform = [muffinVerts,ones(size(muffinVerts,1),1)] * (T.T * muffinLoc)';
                            set(colourMuffin,'Vertices',muffinTransform(:,1:3));
                            drawnow();
    
                        case {4,5,6,7,8}
                            pos = r.model.getpos;
                            T = r.model.fkine(pos);

                            transformedVertices = [verts,ones(size(verts,1),1)] * (T.T * trayLoc)';
                            set(colourTray,'Vertices',transformedVertices(:,1:3));
                            pos = s.model.getpos;                   
                            T = s.model.fkine(pos);                 
                            g.model.base = T;                            
                            g.model.animate(g.model.getpos);                  
                            muffinTransform = [muffinVerts,ones(size(muffinVerts,1),1)] * (T.T * muffinLoc)';
                            set(colourMuffin,'Vertices',muffinTransform(:,1:3));
                            drawnow();
    
                        case {11, 12, 13}
                            pos = r.model.getpos;                   
                            T = r.model.fkine(pos);

                            transformedVertices = [verts,ones(size(verts,1),1)] * (T.T * trayLoc)';  %
                            set(colourTray,'Vertices',transformedVertices(:,1:3));
                            muffinTransform = [muffinVerts,ones(size(muffinVerts,1),1)] * (T.T * muffinFinalLoc)';
                            set(colourMuffin,'Vertices',muffinTransform(:,1:3))
                            drawnow();
                    end
        end
    end
end                            
