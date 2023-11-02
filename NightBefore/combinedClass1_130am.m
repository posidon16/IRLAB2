classdef combinedClass1
    properties
        % Declare all the objects used in this class
        Arduino
        UltrasonicSensor
        q_s
        q_r
        r
        s
        g
        e_stop_connected
       
    end

    methods (Static)
        function execution()
            % Clear all previous data
            clear all
            clf
            clc
            hold on

            % Create instance of contructor function
            operation = combinedClass1;

            operation.createEnvironment();

            % Main robot movement loop fucntion
            operation.runTrajectories();

            pause(15)
        end

        function createEnvironment()
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
        end
        
    end
    
    methods
        function obj = combinedClass1() % Constructor function
            % Define all the objects
            usb_check = serialportlist;
            obj.e_stop_connected = 0;

            % check if Arduino is connected
            if isempty(usb_check)
                obj.e_stop_connected = 0;
                obj.s = DobotMagician;
                obj.r = Mitsubishi;
                obj.g = DobotGripper;
            else
                obj.e_stop_connected  = 1;
                obj.Arduino = arduino('Com5', 'uno', 'Libraries', 'Ultrasonic');
                obj.UltrasonicSensor = ultrasonic(obj.Arduino, 'A0', 'A1');
                obj.s = DobotMagician;
                obj.r = Mitsubishi;
                obj.g = DobotGripper;
            end

            % Plot Dobot in position
            Dobot_Transform = transl(-1.6, 0.3, 1.25) * rpy2tr(0,0,0);
            obj.s.model.base = Dobot_Transform;
            obj.s.model.animate(Dobot_Transform)

            % Plot Mitsubishi in position
            MitsubushiTransform = transl(-0.8, 0.6, 1.25) * rpy2tr(0,0,0);
            obj.r.model.base = MitsubushiTransform;
            obj.r.model.animate(obj.r.model.getpos)

            % Plot Dobot gripper in position
            GripperTransform = transl(-1.6, 0.3, 1.25) * rpy2tr(0,0,0);
            obj.g.model.base = GripperTransform;
            obj.g.model.animate(obj.g.model.getpos)



            % NOTE: BOTH Q-MATRICES SHOULD BE THE SAME SIZE TO RUN PROPERLY
            % Create the q-matrix of Dobot waypoints
            obj.q_s{1} = obj.s.model.ikcon(transl(-1.600, 0.026, 1.517)*rpy2tr(0,0,pi/2), [-1.5708    0.5236    0.9809    1.6232]); 
            obj.q_s{2} = obj.s.model.ikcon(transl(-1.6, -0.009, 1.398)*rpy2tr(0,0,-pi/2), [-1.5708    1.0210    0.9809    1.1432]); 
            obj.q_s{3} = obj.q_s{1}; % holding first red muffin
            obj.q_s{4} = obj.q_s{1};
            obj.q_s{5} = obj.q_s{1};
            obj.q_s{6} = obj.q_s{1};
            obj.q_s{7} = obj.s.model.ikcon(transl(-1.600, 0.552, 1.539)*rpy2tr(0,0,pi/2), [1.5708    0.3534    1.0594    1.7279]);
            obj.q_s{8} = obj.s.model.ikcon(transl(-1.600, 0.562, 1.439)*rpy2tr(0,0,pi/2), [1.5709    0.5498    1.4835    1.0996]); 
            obj.q_s{9} = obj.s.model.ikcon(transl(-1.600, 0.552, 1.539)*rpy2tr(0,0,pi/2), [1.5708    0.3534    1.0594    1.7279]);
            obj.q_s{10} = obj.q_s{1};
            obj.q_s{11} = obj.q_s{1};
            obj.q_s{12} = obj.q_s{1};
            obj.q_s{13} = obj.q_s{1};
            obj.q_s{14} = obj.q_s{1};
            obj.q_s{15} = obj.q_s{1};

            % Create the q-matrix of Mitsubishi waypoints
            obj.q_r{1} = obj.r.model.ikcon(transl(-0.4, 0.566, 1.622)*rpy2tr(-pi/2,-pi/2,-pi/2), [-0.0838    1.1729   -0.3281         0   -0.7540         0]);
            obj.q_r{2} = obj.r.model.ikcon(transl(-0.485, 0.6, 1.385)*rpy2tr(-pi/2,-pi/2,-pi/2), [0    0.5027   -0.5515         0   -1.5708         0]); 
            obj.q_r{3} = obj.r.model.ikcon(transl(-0.270, 0.6, 1.364)*rpy2tr(pi/2,pi/2,pi/2), [0   -0.1257    0.9006         0   -0.8029         0]);
            obj.q_r{4} = obj.r.model.ikcon(transl(-0.219, 0.6, 1.539)*rpy2tr(pi/2,pi/2,pi/2), [0   -0.0314    1.4033         0   -0.2094         0]); 
            obj.q_r{5} = obj.r.model.ikcon(transl(-1.379, 0.6, 1.579)*rpy2tr(-pi/2,-pi/2,-pi/2), [ 3.1416   -0.0419    1.5704    0   -0.0419    0]);
            obj.q_r{6} = obj.r.model.ikcon(transl(-1.342, 0.6, 1.36)*rpy2tr(-pi/2,-pi/2,-pi/2), [3.1416   -0.4887    1.5708    0   -0.5027         0]); 
            obj.q_r{7} = obj.q_r{6}; % loading red muffins
            obj.q_r{8} = obj.q_r{6};
            obj.q_r{9} = obj.q_r{6};
            obj.q_r{10} = obj.q_r{6};
            obj.q_r{11} = obj.q_r{5}; % return red tray to 
            obj.q_r{12} = obj.q_r{4};
            obj.q_r{13} = obj.q_r{3};
            obj.q_r{14} = obj.q_r{2}; % pull out spatula from red tray
            obj.q_r{15} = obj.q_r{1};
        end


        
        function runTrajectories(obj) % Main robot movement loop fucntion
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

            robot_pos = [obj.r.model.getpos]; 
            obj.r.model.fkine(robot_pos)
            verts = get(redTray, 'Vertices');

            robot_pos = [obj.s.model.getpos]; 
            obj.s.model.fkine(robot_pos)
            muffinVerts = get(redMuffin, 'Vertices');

            % Main robot movement loop
            for i = 1:width(obj.q_s)
                % iteration counter
                disp(i);
                if i == 3 || 4 || 5 || 6
                    qMatrixRedMuffin = jtraj(robot_pos,obj.q_s{i},steps);
                end

                % Calculate the joint trajectory between q matrix points
                newQ_s = obj.q_s{i};
                jointTrajectory_s = jtraj(obj.s.model.getpos(), newQ_s, steps);
                newQ_r = obj.q_r{i};
                jointTrajectory_r = jtraj(obj.r.model.getpos(), newQ_r, steps);

               
                % What happens in every step of a movement
                for trajStep = 1:size(jointTrajectory_s, 1)

                    % Update Dobot gripper location
                    pos = obj.s.model.getpos;                   % get pos of UR3
                    T = obj.s.model.fkine(pos);                 % get end affector position of UR3
                    obj.g.model.base = T;                            % assign the base of the gripper
                    obj.g.model.animate(obj.g.model.getpos);
                    drawnow();

                        if obj.e_stop_connected == 1
                            % Read the halt button and ultrasonic sensor
                            haltButtonState = readDigitalPin(obj.Arduino, "D13");
                            distance = readDistance(obj.UltrasonicSensor);
                            
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
                                writeDigitalPin(obj.Arduino, 'D3', 1); % Show status on LED
                                writeDigitalPin(obj.Arduino, 'D2', 0); % Show status on LED
                                obj.s.model.animate(jointTrajectory_s(trajStep, :));
                                obj.r.model.animate(jointTrajectory_r(trajStep, :));
                                
                                switch i
                                    case {4,5,6,7,8}
%                                         verts = get(redTray, 'Vertices');
                                        pos = obj.r.model.getpos;
                                        T = obj.r.model.fkine(pos); 
                                        transformedVertices = [verts,ones(size(verts,1),1)] * (T.T * transl(-1.36,-0.6,0.35)*rpy2tr(0,pi/2,0))';
                                        set(redTray,'Vertices',transformedVertices(:,1:3));
                                end
        
                                drawnow();
                                disp("Running as normal");
        
                            % What happens if e-stop is activated    
                            else 
                                disp("E-stop is active");
                                writeDigitalPin(obj.Arduino, 'D3', 0); % Show status on LED
                                writeDigitalPin(obj.Arduino, 'D2', 1); % Show status on LED
        
                                % Checking function to get out of e-stop
                                while 1
                                    % Read the halt button and ultrasonic sensor
                                    goButtonState = readDigitalPin(obj.Arduino, "D12");
                                    haltButtonState = readDigitalPin(obj.Arduino, "D13");
        
                                    % if both buttons are pressed at once
                                    if goButtonState == 0 && haltButtonState == 0
                                        disp("E-stop has been deactivated");
                                        status = ~status;
                                        pause(0.3) % Debouncer
                                        break % Return to Main robot movement loop
                                    end
                                end
                            end
    
                        elseif obj.e_stop_connected == 0
                            obj.s.model.animate(jointTrajectory_s(trajStep, :));
                            obj.r.model.animate(jointTrajectory_r(trajStep, :));
                            
                            switch i
                                case 3
%                                     obj.s.model.animate(qMatrixRedMuffin(trajStep,:))   % animate the UR3 to animate through the q Matrix
                                    pos = obj.s.model.getpos;                   % get pos of UR3
                                    T = obj.s.model.fkine(pos);                 % get end affector position of UR3
                                    obj.g.model.base = T;                            % assign the base of the gripper
                                    obj.g.model.animate(obj.g.model.getpos);                  %animate the gripper
                                    %%transformedVertices = [verts,ones(size(verts,1),1)] * (T.T * transl(-1.37,-0.6,0.35)*rpy2tr(0,pi/2,0))';
                                    muffinTransform = [muffinVerts,ones(size(muffinVerts,1),1)] * (T.T * transl(1.6, 0.01, -1.395)*rpy2tr(0,0,0))';
                                    set(redMuffin,'Vertices',muffinTransform(:,1:3));
                                    drawnow();

                                case {4,5,6,7,8}
                                    pos = obj.r.model.getpos;
                                    T = obj.r.model.fkine(pos); 
                                    transformedVertices = [verts,ones(size(verts,1),1)] * (T.T * transl(-1.36,-0.6,0.35)*rpy2tr(0,pi/2,0))';
                                    set(redTray,'Vertices',transformedVertices(:,1:3));
                                    %if i == 3 || 4 || 5 || 6 || 7 || 8
%                                         obj.s.model.animate(qMatrixRedMuffin(trajStep,:))   % animate the UR3 to animate through the q Matrix
                                        pos = obj.s.model.getpos;                   % get pos of UR3
                                        T = obj.s.model.fkine(pos);                 % get end affector position of UR3
                                        obj.g.model.base = T;                            % assign the base of the gripper
                                        obj.g.model.animate(obj.g.model.getpos);                  %animate the gripper
                                        %%transformedVertices = [verts,ones(size(verts,1),1)] * (T.T * transl(-1.37,-0.6,0.35)*rpy2tr(0,pi/2,0))';
                                        muffinTransform = [muffinVerts,ones(size(muffinVerts,1),1)] * (T.T * transl(1.6, 0.01, -1.395)*rpy2tr(0,0,0))';
                                        set(redMuffin,'Vertices',muffinTransform(:,1:3));
                                        drawnow();
                                    %end
                                case {11, 12, 13}
%                                         obj.r.model.animate(qMatrixPlace(j,:))   % animate the UR3 to animate through the q Matrix
                                        pos = obj.r.model.getpos;                   % get pos of UR3
                                        T = obj.r.model.fkine(pos);                 % get end affector position of UR3
                                    
                                        transformedVertices = [verts,ones(size(verts,1),1)] * (T.T * transl(-1.36,-0.6,0.35)*rpy2tr(0,pi/2,0))';  %
                                        set(redTray,'Vertices',transformedVertices(:,1:3));
                                        muffinTransform = [muffinVerts,ones(size(muffinVerts,1),1)] * (T.T * transl(-1.32,-0.03,-1.34)*rpy2tr(0,pi/2,0))';
                                        set(redMuffin,'Vertices',muffinTransform(:,1:3))
                                        drawnow();

                            end
    
                            drawnow();
                            disp("No E-stop Connected");
                        end
                    end                            
                end
            end
        end
    end
