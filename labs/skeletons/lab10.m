%%%%%%%%%%%%%
% ECE 3610
% LAB 10 -- Actuators 3: Combining Sensorimotor Loops
%%%%%%%%%%%%%

%%%%%%%%%%%%%
% Modern cyberphysical systems are simultaneously handling sensor input 
% from a number of sources, then intelligently combining that into a 
% number of actuator signals. So far you have only mapped sensors and 
% actuators in a 1:1 fashion. This lab is purposefully open-ended; 
% hopefully it will give you time to both catch up if you're behind and 
% explore if you're ahead. 

% Deliverables:
%   - A circuit and associated code which uses no fewer than six of your 
%   components and contains at least two different feedback/sensorimotor
%   loops operating at the same time. At least one of the sensorimotor
%   loops should fuse sensor data from at least two different sources.
%   - Tell me a story about what your device is meant to do. This can be
%   just part of a larger (imaginary) system, you can use analogies 
%   ("instead of an led, this would be a ____ "). Get creative!
%
% Extensions:
%   - Work with a partner to have your systems interact.
%%%%%%%%%%%%%%

%% 1. CONNECT TO YOUR NANOBOT
%  Remember to replace the first input argument with text corresponding to
%  the correct serial port you found through the Arduino IDE. Note that the
%  clc and clear all will clear your command window and all saved
%  variables!

clc
clear all
nb = nanobot('?', 115200, 'serial');

%% 2. Building your sensorimotor loops
% Take stock of your available input and output devices. Write down a list
% of each. Categorize each input as either BINARY or CONTINUOUS. Next to
% each input in your list, if it is a sensor, write down the associated
% physical quantity being transduced.

% HINT: When working with design challenges, it can help to clearly define
% your objectives, list usable components, and brainstorm possible next
% steps. Once you have a decent idea pinned down, start by breaking your
% design task into sections.

% E.g. something for this lab could look like:
%
% Objective: 6 components, 2 interacting sensorimotor loops
%
% Components: DC Motor, LED, potentometer, photoresistor, etc.
%
% Possible concepts: System for robot that changes LED with ultrasonic,
% motor speed dependent on another sensor, etc.
%
% CODE STRUCUTRE:
% 1. pin assignments and initialization
% 2. defining useful variables and ranges
% 3. start of while loop
% 4. data collection/processing
% 5. condition testing
% 6. sensorimotor loops
% ...
% X. End of while loop/ resetting functions

% You've been setting up sensorimotor loops almost every lab so far, so go
% crazy with finding ways to combine multiple sensors and actuators/transducers!



%% X. EXTENSION (optional)
% - Work with a partner to have your systems interact.


%% X. DISCONNECT
%  Clears the workspace and command window, then
%  disconnects from the nanobot, freeing up the serial port.

clc
delete(nb);
clear('nb');
clear all