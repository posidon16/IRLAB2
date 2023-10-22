classdef Mitsubishi < RobotBaseClass
    %% UR3 Universal Robot 3kg payload robot model
    %
    % WARNING: This model has been created by UTS students in the subject
    % 41013. No guarentee is made about the accuracy or correctness of the
    % of the DH parameters of the accompanying ply files. Do not assume
    % that this matches the real robot!

    properties(Access = public)   
        plyFileNameStem = 'Mitsubishi';
    end
    
    methods
%% Constructor
        function self = Mitsubishi(baseTr)
                self.CreateModel();
                self.homeQ = [0,0,pi/2,0,0,0];
                if nargin < 1			
				    baseTr = eye(4);
                end
                self.model.base = self.model.base.T * baseTr; % * trotx(pi/2) * troty(pi/2);
                
                self.PlotAndColourRobot();
            
        end
           

%% CreateModel
        function CreateModel(self)
            
              link(1) = Link('d',0.3, 'a',0,  'alpha', pi/2,  'offset',0,     'qlim',deg2rad([-240 240]));
              link(2) = Link('d',0, 'a',0.23,  'alpha',0,      'offset',0,     'qlim',deg2rad([-30 210]));
              link(3) = Link('d',0, 'a',0.05,  'alpha',-pi/2,  'offset',0,     'qlim',deg2rad([-70 90]));
              link(4) = Link('d',-0.27, 'a',0,  'alpha',-pi/2,  'offset',0,     'qlim',deg2rad([-200,200]));
              link(5) = Link('d',0,    'a',0,   'alpha',-pi/2,      'offset',0,         'qlim',deg2rad([-120,120]));
              link(6) = Link('d',0.077,    'a',0,   'alpha',0,      'offset',0,         'qlim',deg2rad([-360,360]));




            self.model = SerialLink(link,'name',self.name);
        end      
    end
end
