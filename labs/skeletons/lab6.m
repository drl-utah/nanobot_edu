%%%%%%%%%%%%%
% ECE 3610
% LAB 6 -- Ultrasonic Mapping
%%%%%%%%%%%%%

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
nb = nanobot('?', 115200, 'serial');

%% 2. Connecting and initializing the ultrasonic rangefinder
%  First, we need to set up the rangefinding module. Find some code in
%  nanobot_demo.m to help you initialize the ultrasonic rangefinder. Wire
%  up your rangefinder according to Vcc, GND, and the trigger/echo pins you
%  selected for initialization.

% Solution:
% Initialize the ultrasonic sensor with TRIGPIN, ECHOPIN
nb.initUltrasonic('?','?') % Use any of the (D)igital pins, ex. D8, D7

%% 3. Exploring the rangefinder
% Using your ruler, find the scale factor, resolution, max and min range,
% and linearity of your ultrasonic rangefinder. See included diagrams for
% help with visualizing each of these.

% This will depend a lot on the recorded data, which will be done by hand.
% Create an array of values representing distances along your ruler in cm.
% Record the ultrasonic reading you get at a spacing of every 2 cm in a
% separate array.

%% Reading Ultrasonic Data:
% Use this section to record the reading at different distances
while(true)
    %Take a single ultrasonic reading
    val = nb.ultrasonicRead();
    fprintf('val = %i\n', val)
    pause(0.5); % adjust me if you want faster or slower samples
end

%% Graphing custom data:
dist = [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30]; % in cm
% Replace the strings below with the appropriate value measured at the
% corresponding distance:
val = ['?', '?', '?', '?', '?', '?', '?', '?', '?', '?', '?', '?', '?', '?', '?'];
% For plotting your data:
fit = polyfit(dist, val, 1);
scatter(dist, val);
hold on
plot(dist, polyval(fit,dist), '-r');
hold off

%% Finding Scale Factor
% The scale factor of our ultrasonic rangefinder can be thought of as the
% coefficient that converts between the rangefinder's numeric output, and
% the physical distance that the measurement corresponds to.
% In this case, we can solve for an average scale factor by using the
% measured values we recorded at 2 cm increments
arraySize = size(dist, 2); 
scaleFactorList = zeros(1, arraySize);
for i = 1:arraySize
    scaleFactorList(i) = '?'; % In [units/cm], according to array index
end
avgScaleFactor = mean(scaleFactorList);
fprintf("The average scale factor is %.3f units/cm\n", avgScaleFactor);

%% Finding Sensor Resolution
% To find the resolution of your sensor, find the smallest distance change
% that results in a reading change of your rangefinder.
resolution = '?';

%% Measuring Linearity
% To get an estimate of the linearity of your sensor, we can use the R^2
% value of a linear regression to get a sense of how well our rangefinder
% correlates with a true linear response.
% Using your scale factor, create an array of expected outputs at each
% value of dist
yCalc = zeros(1, arraySize);
for i = 1:arraySize
    yCalc(i) = '?'; % y(i) = m * x(i), what do m and x correspond to? 
end
Rsq = 1 - sum((val - yCalc).^2)/sum((val - mean(val)).^2); % Formula for R^2
fprintf("The R^2 value for a pure linear fit based on the scale factor is: %f\n", Rsq);

%% Max and Min Range
% These are the extreme values at which your rangefinder still measures a
% stable value. Due to the crowded classroom with many other active
% rangefinder pulses, measuring a stable max range could prove difficult.
% Experiment and find out what you can get as your minimum and maximum
% effective ranges
minRange = '?';
maxRange = '?';

%% 4. Varying LED intensity
% Write code to make your LED vary its brightness based on the distance the
% ultrasonic sensor reads.

% Solution:
nb.initUltrasonic('?', '?'); % Fill in with your ultrsonic trigger and echo pins
nb.initRGB('?','?','?'); % Fill in with your RGB module pins
while (true)
    pulseVal = nb.ultrasonicRead();
    cmVal = pulseVal / avgScaleFactor;
    fprintf("Last read: %0.1f cm\n",cmVal);
    relBright = round('?'); % linear interpolation based on custom data. HINT: see lab3 or lab4's use of it
    if (relBright > 255)
        relBright = 255;
    elseif (relBright < 0)
        relBright = 0;
    end
    nb.setRGB(255-relBright, 0, 0); % So that LED intensity decreases at further distance
    pause(0.05); % Don't gotta go too fast
end

%% 5. DISCONNECT
%  Clears the workspace and command window, then
%  disconnects from the nanobot, freeing up the serial port.

clc
delete(nb);
clear('nb');
clear all