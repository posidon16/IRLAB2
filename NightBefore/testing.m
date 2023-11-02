

Es = 0;



while true
Es(1)

propListener = addlistener(Es,'PostSet',@(src,evnt)disp('Color changed'));

pause(0.5);
end

% classdef testing
%     properties (Access = public)
%         Es
%     end
%     methods
%         function obj = testing(property)
%             obj.Es = property;
%             addlistener(obj, 'Property', 'PostSet', @obj.onPropertySet);
%         end
% 
%         function onPropertySet(obj, ~, ~)
%             % Update the value of the property in the class.
%             obj.Es = obj.Es + 1;
%         end
%     end
% end