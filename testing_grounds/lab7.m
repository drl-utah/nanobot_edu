%%%%%%%%%%%%%
% ECE 3610
% LAB 7 -- Ultrasonic Mapping
%%%%%%%%%%%%%

% TODO - Add the initial trigger/echo pin thought experiment? It seems to
% rely a lot on pictures, though. Also test with RGB module once available.

% FIXME - ask about nonlinearity, scale factor (slope?) and correcting for
% it? Also FSO. Also which MATLAB functions are useful in analyzing this
% data

%%%%%%%%%%%%%
% Ultrasonic rangefinding was one of the first techniques for mobile robot 
% obstacle detection and avoidance. Even in a world where onboard computer 
% vision is near ubiquitous, ultrasonic ranging provides a simple, 
% affordable, and computationally inexpensive method of obstacle detection.
% A vast number of small wheeled robots still use ultrasonic and/or 
% infrared rangefinding for autonomy. We will be developing some of the 
% code required to interface with this new sensor, and then characterizing 
% the sensor performance. Getting this right is important -- you'll be 
% using this same module for your final project. We wouldn't want your 
% robots smashing into any walls.
%
% Deliverables:
% - Code that continuously graphs the ultrasonic rangefinder's distance 
% measurement (correctly converted), corrects for any detected 
% nonlinearity, and turns on an LED with varying intensity depending on 
% obstacle distance.
%%%%%%%%%%%%%%

%% 1. CONNECT TO YOUR NANOBOT
%  Remember to replace the first input argument with text corresponding to
%  the correct serial port you found through the Arduino IDE. Note that the
%  clc and clear all will clear your command window and all saved
%  variables!

clc
clear all
nb = nanobot('COM7', 115200, 'serial');

%% 2. Connecting and initializing the ultrasonic rangefinder
%  First, we need to set up the rangefinding module. Find some code in
%  nanobot_demo.m to help you initialize the ultrasonic rangefinder. Wire
%  up your rangefinder according to Vcc, GND, and the trigger/echo pins you
%  selected for initialization.

% Solution:
% Initialize the ultrasonic sensor with TRIGPIN, ECHOPIN
nb.initUltrasonic('D8','D7')

%% 3. Exploring the rangefinder
% Using your ruler, find the scale factor, resolution, max and min range,
% and linearity of your ultrasonic rangefinder. See included diagrams for
% help with visualizing each of these.

% Solution:
% This will depend a lot on the recorded data, probably by hand. One
% example would be to set up an array where fixed distances are
% tested against the ultrasonic rangefinder reading at that point, then
% stored into an array for further data analysis.
%% Graphing custom data:
dist = [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30]; % in cm
val = [141, 226, 329, 516, 584, 721, 858, 951, 1100, 1228, 1397, 1560, 1560, 1654, 1715];
fit = polyfit(dist, val, 1);
scatter(dist, val);
hold on
plot(dist, polyval(fit,dist), '-r');
hold off

%% 4. Varying LED intensity
% Write code to make your LED vary its brightness based on the distance the
% ultrasonic sensor reads.

% Solution:
while (1)
    pulseVal = nb.ultrasonicRead();
    fprintf("Last read: %0.0f \n",pulseVal);
    relBright = round(255 * (pulseVal/(1715-141))); % based on custom data
    if (relBright > 255)
        relBright = 255;
    end
    nb.setRGB(255-relBright, 0, 0);
    pause(0.01); % Don't gotta go too fast
end

%% 5. DISCONNECT
%  Clears the workspace and command window, then
%  disconnects from the nanobot, freeing up the serial port.

clc
delete(nb);
clear('nb');
clear all