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
nb = nanobot('COM7', 115200, 'serial');

%% 2. Building your sensorimotor loops
% Take stock of your available input and output devices. Write down a list
% of each. Categorize each input as either BINARY or CONTINUOUS. Next to
% each input in your list, if it is a sensor, write down the associated
% physical quantity being transduced.

% Solution:
% Used Components:
%   - DC motor
%   - RGB LED
%   - 10k resistor
%   - 10k potentiometer (CONTINUOUS) - angular position
%   - Photoresistor (CONTINUOUS) - light brightness
%   - Ultrasonic ranger (CONTINUOUS) - distance

% Concept:
% Use a brightness threshold to determine motor on or off
% Use distance from ultrasonic to change relative brightness of LED
% Potentiometer changes speed of motor

% Pins:
% - R G B: D12, D11, D10
% - Ultrasonic: D8 trig, D7 echo
% - Photoresistor: A1
% - Potentiometer: A2
% - Motor: M1

% Initialization:
nb.pinMode('A1','ainput');
nb.pinMode('A2','ainput');
nb.initRGB('D12','D11','D10');
nb.initUltrasonic('D8','D7');
nb.setRGB(0, 0, 0);
% Set thresholds:
slowThresh = 80;
stopThresh = 40;
maxRange = 3000;
minRange = 200;

tic
% Begin loop
while toc < 30
    % Read inputs
    numSamples = 3;
    photoResVals = zeros(1,numSamples);
    potVals = zeros(1, numSamples);
    for i = 1:numSamples
        photoResVals(i) = nb.analogRead('A1');
        potVals = nb.analogRead('A2');
    end
    avgPhoto = mean(photoResVals);
    avgPot = mean(potVals);
    ultraDist = nb.ultrasonicRead();

    % Have data now
    % Use pot to increase motor speed
    % Add limits so we don't short the circuit
    potFlip = 1023 - avgPot;

    if potFlip < 200
        potFlip = 200;
    elseif potFlip > 800
        potFlip = 800;
    end
    
    % Make 200-800 range correspond to 0-100 duty cycle
    dCyc = round(0 + (potFlip - 200) * ((100 - 0)/(800 - 200)));

    gLED = 0;

    % Control Logic
    if avgPhoto < stopThresh
        dCyc = 0;
        gLED = 255;
    elseif avgPhoto < slowThresh
        dCyc = round(dCyc/2);
        gLED = 128;
    end
    nb.setMotor(1, dCyc);

    rLED = round(dCyc * (255)/(100));

    if ultraDist > maxRange
        ultraDist = maxRange;
    elseif ultraDist < minRange
        ultraDist = minRange;
    end

    shiftedRange = maxRange - ultraDist;
    bLED = round((shiftedRange) * ((255)/(maxRange - minRange)));

    nb.setRGB(rLED, gLED, bLED);
    

    %fprintf("photo: %d      pot: %d       ultra: %d\n", avgPhoto, avgPot, ultraDist);
    pause(0.1);
end
nb.setMotor(1, 0);
nb.setRGB(0, 0, 0);


%% X. EXTENSION (optional)
% - Work with a partner to have your systems interact.


%% X. DISCONNECT
%  Clears the workspace and command window, then
%  disconnects from the nanobot, freeing up the serial port.

clc
delete(nb);
clear('nb');
clear all