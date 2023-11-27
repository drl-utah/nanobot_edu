%%%%%%%%%%%%%
% ECE 3610
% LAB 1 -- Toolchain Intro 1 - Changing Colors of the Onboard LED
%%%%%%%%%%%%%

%%%%%%%%%%%%%
% Congratulations! You've managed to set up and connect to the Arduino's 
% wifi network! Now that you have connected, we will use some functions as
% provided in nanobot.m as well as some MATLAB code to change the color of
% the onboard LED. Follow the instructions below and ask for help if you
% feel stuck

% Deliverables:
% - Set the onboard LED color
% - Make onboard LED blink at 5 Hz

% Possible Extensions:
% - Make the LED change colors every 2 seconds
% - Make the LED blink a different color every 5th blink, then return to
%   the original color.
%%%%%%%%%%%%%%

%% 1. CONNECT TO YOUR NANOBOT
%  Remember to replace the first input argument with text corresponding to
%  the correct serial port you found through the Arduino IDE. Note that the
%  clc and clear all will clear your command window and all saved
%  variables!

clc
clear all
nb = nanobot('COM7', 115200, 'wifi');

%% 2. SET LED COLOR
%  Find a way to set the color of the onboard LED to an RGB value of your
%  choice.

% HINT: take a look in nanobot.m for useful functions!
% FIXME: Figure out way to integrate color selection to nanobot.m

%% 3. SET LED BLINK RATE
% Set the onboard LED to blink at a rate of 5 Hz

% HINT: Use frequency's relationship to period to determine cycle time
% HINT: Remember to split the cycle time into on and off periods (think
% duty cycle)

% Solution:
% Setup:
ledFreq = 5;
dtyCyc = 0.5; % have LED on for half the cycle period
cyclePeriod = 1/ledFreq;
onTime = cyclePeriod * dtyCyc;
offTime = cyclePeriod - onTime;

% Loop portion (easy loop)
fprintf("Hit STOP to stop the infinite loop\n");
while(1)
    nb.ledWrite(1);
    pause(onTime);
    nb.ledWrite(0);
    pause(offTime);
end

%% 4. EXTENSION (optional)
%  Extension 1: Try changing the LED's color every two seconds.
%  Extension 2: Make the LED blink a different color every 5th blink, then 
%  return to the original color.
%
%  FIXME: Add corresponding code once RGB implementation is figured out.

%% 5. DISCONNECT
%  Clears the workspace and command window, then
%  disconnects from the nanobot, freeing up the serial port.

clc
clear all
%delete(nb); % DEBUG: I think clear all already does this?
clear('nb');