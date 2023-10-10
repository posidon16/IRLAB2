%rosinit

% [safetyStatePublisher,safetyStateMsg] = rospublisher('/dobot_magician/target_safety_status');
% safetyStateMsg.Data = 2;
% send(safetyStatePublisher,safetyStateMsg);


%%



% Turn on the tool
[toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
toolStateMsg.Data = [1 1];
send(toolStatePub,toolStateMsg);
% [1 1] Close
% [1 0] Open