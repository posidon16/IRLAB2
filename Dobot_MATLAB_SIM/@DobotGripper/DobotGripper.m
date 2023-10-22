classdef DobotGripper < RobotBaseClass
    %% Dobot Magician Pneumatic Gripper
    %
    % WARNING: This model has been created by UTS students in the subject
    % 41013. No guarentee is made about the accuracy or correctness of the
    % of the DH parameters of the accompanying ply files. Do not assume
    % that this matches the real robot!

    properties(Access = public)   
        plyFileNameStem = 'DobotGripper';
    end
    
    methods
%% Constructor
function self = DobotGripper(baseTr)
			self.CreateModel();
            if nargin < 1			
				baseTr = eye(4);
            end
            self.model.base = self.model.base.T * baseTr; % * trotx(pi/2) * troty(pi/2);
            
            self.PlotAndColourRobot();
            
        end

%% CreateModel
        function CreateModel(self)

            link(1) = Link([pi/2   -0.11    0      pi/2     0]);  
            link(2) = Link([pi    0.015   0     pi/2    1]); % PRISMATIC Link
            link(3) = Link([0       0       0     pi/2    1]); % PRISMATIC Link
            link(4) = Link([pi     0    0   pi/2     1]);% PRISMATIC Link -0.02201

            
            % Incorporate joint limits
            link(1).qlim = [-pi pi];
            link(2).qlim = [-0.0175 0];
            link(3).qlim = [0 0];
            link(4).qlim = [-0.0353 0];

            self.model = SerialLink(link,'name',self.name);


            
        end 
        end 


end
