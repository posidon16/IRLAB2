
%% READ ME:
% To run this code you first need to install the arduino hardware support
% package. To get it, go to 'Home' --> 'Environment' --> 'Add-Ons' --> 
% 'Get Hardware Support Packages' and search for 'MATLAB Support Package
% for Arduino Hardware'.


%% THE CODE:
% Clear previous and initalise new instance of arduino connection
clear
a = arduino('com3', 'uno'); % specify the serial-com port and arduino board

% Define the intial button state
buttonState = false;

% Main loop
while 1
    value = readDigitalPin(a, "D2"); % read the button signal
    
    if value == buttonState % if the signal is the same as the inital state
        buttonState = true; % toggle the button state
        writeDigitalPin(a, 'D13', 1) % turn the LED on
    else
        buttonState = false; % retain the inital buttons state
        writeDigitalPin(a, 'D13', 0) % turn the LED off
    end

   buttonState % Print the button state
   pause(0.1) % Delay 0.1 seconds before repeating the loop
end
