%% Intelligence Lab 1: Intro to Intelligence

clear; clc; close all; % initialization
r = nanobot('COM47', 115200, 'serial'); %connect to MKR
r.ledWrite(0);

%% Collect Accelerometer Data

% this section of the code connects to the MKR, enables accelerometer
% streaming, and then collects a 1.5-second segment of data.



countdown("Beginning in", 3); %display a countdown to prep user to move accelerometer 

r.ledWrite(1); %Turn on the LED to signify start of recording
disp("Make A Gesture!"); %prompt user to make a movement

numreads = 100; % about 1.5 seconds (on serial)
vals = zeros(3,numreads);
for i = 1:numreads
    val = r.accelRead();
    vals(1,i) = val.x;
    vals(2,i) = val.y;
    vals(3,i) = val.z;
end
r.ledWrite(0); %Turn off the LED to signify end of recording
clc;

data = [vals(1,:);vals(2,:);vals(3,:)]'; % stop recording


%% Determine what gesture the accelerometer trace was

% you need to write your own code here to determine what number the user
% gestured with the accelerometer. At a minimum, you need to write code
% capable of determining if the user gestures a ZERO or a ONE. Currently,
% the code just randomly sets the variable "label" to ONE or ZERO. You
% should write code to automatically set the variable "label" to ZERO, ONE,
% TWO, THREE, FOUR, FIVE, SIX, SEVEN, EIGHT, or NINE. Start with
% differentiating between ZERO and ONE. Then try to incoprorate TWO. Then
% add THREE, then FOUR, etc.


%%%%%%%%%%%%% DETERMINE WHICH GESTURE (YOUR CODE GOES HERE) %%%%%%%%%%%%%%
        
if sum(abs(diff(data)),'all') < 20 % REPLACE
    label = {'ONE'};
else
    label = {'ZERO'};
end

%%%%%%%%%%%%%%%%%%%%%%%%%%% END OF YOUR CODE %%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Plot the accelerometer trace alongside the 
label = categorical(label); %convert the text into a categorial label
figure(); plot(data, 'LineWidth', 1.5); %plot accelerometer traces
legend('X','Y','Z'); ylabel('Acceleration'); xlabel('Time') %label axes
title("Classification:", string(label)); %title plot with the label