%% Michael Larkham Official Release 2

%% READ ME:
% To run this code you first need to install the arduino hardware support
% package. To get it, go to 'Home' --> 'Environment' --> 'Add-Ons' --> 
% 'Get Hardware Support Packages' and search for 'MATLAB Support Package
% for Arduino Hardware'.


%% THE CODE:
% Clear previous and initalise new instance of arduino connection
clear
clc
a = arduino('COM5', 'uno'); % specify the serial-com port and arduino board

% Define the intial button state
goButtonState = false;
haltButtonState = false;
e_halt = true;
lastHaltValue = false;

% Initialize variables for debounce filtering
lastGoButtonState = 0;    % Previous state of the button
lastHaltButtonState = 0;    % Previous state of the button

% Main loop
while 1
    goValue = readDigitalPin(a, "D12"); % read the go button signal
    haltValue = readDigitalPin(a, "D13"); % read the go button signal


    % checking if E_halt button has been pressed
    if haltValue ~= haltButtonState && lastHaltValue == false % if pressed AND was not already in e_halt mode
        e_halt = false; % get out of e_halt mode
    else
        e_halt = true; % stay in e_halt mode
    end

    % checking if both buttons are pressed at once.
    if goValue == 0 && haltValue == 0
        e_halt = false; % get out of e_halt mode
        writeDigitalPin(a, 'D2', 1) % turn the green LED off
        writeDigitalPin(a, 'D3', 1) % turn the red LED off
    end


    % main button order sequence
    if e_halt == false % if not in e_halt mode
            if goValue == goButtonState % if the signal is the same as the inital state
                goButtonState = true; % toggle the button state
                writeDigitalPin(a, 'D2', 0) % turn the green LED on
                writeDigitalPin(a, 'D3', 1) % turn the red LED off
        
            else % if the signal is the same as the inital state
                goButtonState = false; % retain the inital buttons state
                writeDigitalPin(a, 'D2', 1) % turn the green LED off
            end

    elseif e_halt == true % if in e_halt mode
        goButtonState = false; % go button becomes false
        writeDigitalPin(a, 'D2', 1) % turn the green LED off
        writeDigitalPin(a, 'D3', 0) % turn the red LED on
    end
   
   lastHaltValue = e_halt; % records e_halt value
   lastGoButtonState = goValue; % records go value
   lastHaltButtonState = haltValue; % records halt value
   pause(0.1) % Delay 0.1 seconds before repeating the loop
end