%%%%%%%%%%%%%
% ECE 3610
% LAB 7 -- Sensors Milestone
%%%%%%%%%%%%%

% TODO: Affixing Arduino to robot, or freely wiring? If so, allen keys?

%%%%%%%%%%%%%
% This is the first lab where you will work with the robots used in
% your final project. To start, we will work on line detection and
% interfacing with the IR reflectance sensors on the robot.
% As there are only 20 robots to work with, you will be working in groups to
% accomplish this lab. Start by 
%
% Deliverables:
% - Demonstrate functioning code that allows the robot to detect the
% presence of a line of black electrical tape against a white background,
% and determine its location relative to the line.
%%%%%%%%%%%%%%

%% 1. CONNECT TO YOUR NANOBOT
%  Remember to replace the first input argument with text corresponding to
%  the correct serial port you found through the Arduino IDE. Note that the
%  clc and clear all will clear your command window and all saved
%  variables!

clc
clear all
nb = nanobot('COM7', 115200, 'serial');

%% 2. Connecting to the reflectance array
% Follow the wiring diagram on Canvas to connect your Arduino to the
% reflectance arrays. Included below is a short key with the connections
% for reference.

% Once you've double-checked your wiring, lets initialize the reflectance
% array:

%% 3. How the reflectance array works
% In essence, this reflectance array works using an infrared (IR) LED, a
% phototransistor, and a capacitor to detect how much IR light is able to
% reflect off of the surface beneath the sensor. Prior to a reading, the
% capacitor is charged and stands by. When a reading is taken, the
% capacitor is allowed to discharge at a rate dictated by the
% phototransistor, dependent on the light it recieves. The time it takes
% for the capacitor to discharge to a certain voltage is recorded and
% reported as a measure of the reflectance of the surface below the sensor.
% On your reflectance sensors, this time is measured in microseconds.
% Higher values correspond to less reflective surfaces, and vice versa.

% For more detailed info, https://www.pololu.com/docs/0J13/2 is a good
% resource.

%% 4. 