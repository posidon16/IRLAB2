%% Initialise Dobot
safetyStatusSubscriber = rossubscriber('/dobot_magician/safety_status');
pause(2); %Allow some time for MATLAB to start the subscriber
currentSafetyStatus = safetyStatusSubscriber.LatestMessage.Data;

[safetyStatePublisher,safetyStateMsg] = rospublisher('/dobot_magician/target_safety_status');
safetyStateMsg.Data = 2;
send(safetyStatePublisher,safetyStateMsg);

%% Go Home
jointTarget = [0,0,0,0]; % Remember that the Dobot has 4 joints by default.

[targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
trajectoryPoint.Positions = jointTarget;
targetJointTrajMsg.Points = trajectoryPoint;

send(targetJointTrajPub,targetJointTrajMsg);
%% 

[zRed, zBlue, zGreen, xRed,xBlue, xGreen] = mask_test1_Callum()


%% Run Dobot
% % PickPos = [    0.2324 -0.0029 -0.023];
% % LiftPos = [0.2419 -0.0058 0.1030];
% % AbovePlacePos = [   -0.0007 -0.2704 0.1041];
% % PlacePos = [    0.0017 -0.2798 -0.0023]; 
% % PickupGround = [0.1365 0.01696 -0.03040483856201172]
RedPickupGround = [zRed xRed -0.03040483856201172] % should be x: 0.2344410400390625 y: -0.016490365982055662
GreenPickupGround = [zBlue xBlue -0.03040483856201172] % should be x: 0.23359539794921874 y: -0.05244132614135742
BluePickupGround = [zGreen xGreen -0.03040483856201172] %should be x: 0.2353419952392578 y: 0.017756294250488282
PickupLift = [0.2467842559814453 0.02160359001159668 0.09542754364013672]
RedLift = [0.08017970275878906 0.2341857604980469 0.09966698455810546]
RedDrop = [0.08640805053710937 0.2908858947753906 -0.03446934890747071]
GreenLift = [0.2035739288330078 -0.2281324615478516 0.09625828552246093]
GreenDrop = [0.1974341735839844 -0.2147364807128906 -0.03001092529296875]
BlueLift = [-0.01237137222290039 -0.2940900573730469 0.1293307952880859]
BlueDrop = [0.01036699390411377 -0.3115442810058594 -0.0322947883605957]



q{1} = RedPickupGround
q{2} = PickupLift
q{3} = RedLift
q{4} = RedDrop
q{5} = RedLift
q{6} = PickupLift


q{7} = GreenPickupGround
q{8} = PickupLift
q{9} = GreenLift
q{10} = GreenDrop
q{11} = GreenLift
q{12} = PickupLift


q{13} = BluePickupGround
q{14} = PickupLift
q{15} = GreenLift 
q{16} = BlueLift
q{17} = BlueDrop
q{18} = BlueLift
q{19} = GreenLift 
q{20} = PickupLift

% Qmatrix{1} = PickPos;
% Qmatrix{2} = LiftPos;
% Qmatrix{3} = AbovePlacePos;
% Qmatrix{4} = PlacePos;

%% Open Tool
% [toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
% toolStateMsg.Data = [0 0];
% send(toolStatePub,toolStateMsg);
% 
% %% Close Tool
% [toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
% toolStateMsg.Data = [1 1];
% send(toolStatePub,toolStateMsg);

%% Run through the Stored Parameters

for i = 1:20
    endEffectorPosition = q{i}
% endEffectorPosition = EndEffectorLift;
endEffectorRotation = [0,0,0];

[targetEndEffectorPub,targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');

targetEndEffectorMsg.Position.X = endEffectorPosition(1);
targetEndEffectorMsg.Position.Y = endEffectorPosition(2);
targetEndEffectorMsg.Position.Z = endEffectorPosition(3);

qua = eul2quat(endEffectorRotation);
targetEndEffectorMsg.Orientation.W = qua(1);
targetEndEffectorMsg.Orientation.X = qua(2);
targetEndEffectorMsg.Orientation.Y = qua(3);
targetEndEffectorMsg.Orientation.Z = qua(4);

send(targetEndEffectorPub,targetEndEffectorMsg);
disp(i)
pause(5)
end
%% Original Code to move end effector
% % endEffectorPosition = Qmatrix{4};
% % endEffectorRotation = [0,0,0];
% % 
% % [targetEndEffectorPub,targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');
% % 
% % targetEndEffectorMsg.Position.X = endEffectorPosition(1);
% % targetEndEffectorMsg.Position.Y = endEffectorPosition(2);
% % targetEndEffectorMsg.Position.Z = endEffectorPosition(3);
% % 
% % qua = eul2quat(endEffectorRotation);
% % targetEndEffectorMsg.Orientation.W = qua(1);
% % targetEndEffectorMsg.Orientation.X = qua(2);
% % targetEndEffectorMsg.Orientation.Y = qua(3);
% % targetEndEffectorMsg.Orientation.Z = qua(4);
% % 
% % send(targetEndEffectorPub,targetEndEffectorMsg);
%% End effector getpos
% endEffectorPosition = [0.2,0,0.1];
% endEffectorRotation = [0,0,0];
% 
% [targetEndEffectorPub,targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');
% 
% targetEndEffectorMsg.Position.X = endEffectorPosition(1);
% targetEndEffectorMsg.Position.Y = endEffectorPosition(2);
% targetEndEffectorMsg.Position.Z = endEffectorPosition(3);