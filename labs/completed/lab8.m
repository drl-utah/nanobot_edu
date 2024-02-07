%%%%%%%%%%%%%
% ECE 3610
% LAB 8 -- Actuators 1: DC Motors and Encoders
%%%%%%%%%%%%%

%%%%%%%%%%%%%
% Feedback between sensor inputs and actuator outputs is at the core of a
% huge number of robotic and cyberphysical system applications. In this
% lab, we will explore two different methods of sensorimotor feedback with
% naive control implementations. We will build on some of the underlying
% skills later, but for now we will largely be able to use code and
% concepts from earlier labs.
%
% Deliverables:
%   - An IMU tilt sensor that controls the speed of a DC motor
%   rotation, and reverses direction depending on tilt direction
%   - A flex sensor that sets the angle of a servo motor to match the
%   bend angle.
%
% Extensions:
%   - Use the IMU tilt sensor to control the speed of the DC motor using
%   one axis, and change the angle of the servo motor using the other.
%   (Speed and steering control!)
%%%%%%%%%%%%%%

%% 1. CONNECT TO YOUR NANOBOT
%  Remember to replace the first input argument with text corresponding to
%  the correct serial port you found through the Arduino IDE. Note that the
%  clc and clear all will clear your command window and all saved
%  variables!

clc
clear all
nb = nanobot('COM47', 115200, 'serial');

%% 2. DC motor speed based on IMU Tilt
%  Solution:
tic

while (toc < 20)
    %Here is an example of taking 5 accelerometer readings, then averaging
    %each axis:
    numreads = 5;
    vals = zeros(3,numreads);
    for i = 1:numreads
        val = nb.accelRead();
        vals(1,i) = val.x;
        vals(2,i) = val.y;
        vals(3,i) = val.z;
    end
    %Note the index, getting every column in a specific row for each axis:
    meanx = mean(vals(1,:));
    meany = mean(vals(2,:));
    meanz = mean(vals(3,:));
    
    % Conversion of IMU value to duty cycle
    % Since the onboard IMU is already normalized to a range between -1 and 1,
    % we can just use a direct scale factor to get the duty cycle
    nb.setMotor(1, meany * 50); % Chose 50 because 100 scares me

    pause(0.01); % No need to update too frequently
end
% DONT FORGET THIS!
nb.setMotor(1, 0);

%% 3. Servo angle matching flex sensor
% Solution:
% Keep in mind, the resistance of the flex sensor is likely to only change
% significantly between 0 and 90 degrees.
tic

while (toc < 30)
    % Find average value over 10 samples of A1 to get a steady signal
    numreads = 10;
    vals = zeros(1,numreads);
    for i = 1:numreads
        vals(i) = nb.analogRead('A1');
    end
    meanval = mean(vals);

    % Ensure clean range
    if meanval > 550
        meanval = 550;
    elseif meanval < 380
        meanval = 380;
    end

    maxVal = 550 - 380;
    flippedVal = 550 - meanval;
    
    servoAngle = (flippedVal / maxVal) * 90;
    nb.setServo(1, servoAngle);
    pause(0.01);

end
nb.setServo(1, 0);

%% 4. EXTENSION (optional)
%  Use the IMU to control motor speed and the servo angle at the same time!

%  Solution:
tic

while (toc < 20)
    %Here is an example of taking 5 accelerometer readings, then averaging
    %each axis:
    numreads = 5;
    vals = zeros(3,numreads);
    for i = 1:numreads
        val = nb.accelRead();
        vals(1,i) = val.x;
        vals(2,i) = val.y;
        vals(3,i) = val.z;
    end
    %Note the index, getting every column in a specific row for each axis:
    meanx = mean(vals(1,:));
    meany = mean(vals(2,:));
    meanz = mean(vals(3,:));
    

    nb.setMotor(1, round(50 * meanx));
    nb.setServo(1, 90 + round(90 * meany));

    pause(0.01); % No need to update too frequently
end
nb.setMotor(1,0);
nb.setServo(1,0);

%% Run if manually stopped to clear motors
nb.setMotor(1,0);
nb.setServo(1,0);

%% X. DISCONNECT
%  Clears the workspace and command window, then
%  disconnects from the nanobot, freeing up the serial port.

clc
delete(nb);
clear('nb');
clear all