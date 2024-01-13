%%%%%%%%%%%%%
% ECE 3610
% LAB 3 -- Analog Inputs and Variable resistances
%%%%%%%%%%%%%

%%%%%%%%%%%%%
% This lab will serve as a basic introduction to one of the most important
% functions of a microcontroller: interfacing with external circuitry and
% sensors to read measured stimulus and convert it into a
% computationally-tractable form. We will use an essential basic circuit
% topology, the voltage divider, along with two common components: a
% potentiometer and a flex sensor.
%
% Deliverables:
% - Print the incoming analog value of the potentiometer circuit, converted
% to a voltage, at approximately 2 Hz.
% - Change the RGB LED's brightness with the potentiometer, and change the
% color of the LED from green to red depending on the flex sensor angle.
%
% Extensions:
% - Use the potentiometer knob position to set the flex sensor bend angle
% which will turn on the onboard LED.
%%%%%%%%%%%%%%

%% 1. CONNECT TO YOUR NANOBOT
%  Remember to replace the first input argument with text corresponding to
%  the correct serial port you found through the Arduino IDE. Note that the
%  clc and clear all will clear your command window and all saved
%  variables!

clc
clear all
nb = nanobot('COM47', 115200, 'serial');

%% 2. Building a voltage divider
%  Using the blue potentiometer and the 10k resistor, build a voltage
%  divider between VCC (+3.3 V on Arduino) and GND, with the 10k resistor
%  nearer VCC. Connect your analog read pin A1 to the space between the
%  resistor and potentiometer.

%% 3. Measuring analog voltage
%  Now we will convert the analog value we recieve into a voltage. First,
%  average 10 samples of the input to get the average analog value
%  recorded.

% Solution:
nb.pinMode('A1', 'ainput'); % Set the analog pin to read analog input
% Here is an example of taking 10 readings, then averaging:
numreads = 10;
vals = zeros(1,numreads);
for i = 1:numreads
    vals(i) = nb.analogRead('A1');
end
meanval = mean(vals);
fprintf('mean val = %.0f\n', meanval);

%  To find the measured voltage, we can use the equation:
%   ADC_res / system_voltage = ADC_reading / analog_voltage_measured
%  The Arduino uses 10-bit ADC, which corresponds to a resolution of 1023.
%  Find the analog voltage measured based off what you know. Use this
%  analog voltage to estimate the potentiometer resistance using what you
%  know about voltage dividers.
%
%  HINT: The equation for a voltage divider can be expressed as:
%   Vout = Vin * (R2 / (R1 + R2))

% Solution:
aVolt = (meanval / 1023) * 3.3;
r2 = (aVolt / (3.3 - aVolt)) * 10; % R2 = (Vout / (Vin - Vout)) * R1
fprintf('Total Pot Resistance = %f\n', r2);

%% 4. Exploring potentiometers
%  Move the pin connecting A1 to the center of the voltage divider to the
%  wiper pin of the potentiometer. Use a live plot to see how rotating the
%  potentiometer knob affects your analog voltage values.

% Solution:
%You can also plot an analog read value. Note that I have to initialize the pin as an analog input ("ainput")
%first:
nb.pinMode('A1','ainput');
nb.livePlot('analog','A1');

%% 5. Printing analog voltages
%  Convert the incoming analog value to a voltage and print it to the
%  command window at 2 Hz

% Solution:
nb.pinMode('A1', 'ainput');
while(true)
    tic
    numreads = 10;
    vals = zeros(1,numreads);
    for i = 1:numreads
        vals(i) = nb.analogRead('A1');
    end
    meanval = mean(vals);
    potVolt = (meanval / 1023) * 3.3;
    fprintf('Pot voltage = %f V\n', potVolt);
    while(toc < 0.5)
        pause(0.01);
    end
end

%% 6. Analyzing Potentiometers
%  Imagine that the potentiometer knob position is changing the values of
%  two resistors it splits in half, R1 and R2. Use the analog voltage
%  values and the value of the known 10k resistor to solve for R1 and R2
%  when the potentiometer is turned fully in both directions. Think about
%  what that means a potentiometer does!

% Solution: (assuming I understand)
% maxV = 1.69; R1 ~ 0 here and R2 ~ 10k
% minV = 0.005; R1 ~ 10k here and R2 ~ 0

%% 7. Using a flex sensor
%  Make a voltage divider circuit on your breadboard using the flex sensor
%  and the 10k resistor. Connect Vout to A1 on your Arduino. Be careful
%  when plugging the flex sensor in; grip it as close to the base as
%  possible.
%
%  Use a live plot to see what happens to the analog value when you bend
%  the flex sensor. Note, only the striped portion of the sensor matters
%  when bending

% Solution:
nb.pinMode('A1', 'ainput');
nb.livePlot('analog', 'A1');

%% 8. Calculating approximate 45-degree resistance
%  Use the live plot analog values to calculate the approximate resistance
%  of the sensor when it is bent 45 degrees.

% Solution:
bentV = (560 / 1023) * 3.3;
bentR = (bentV / (3.3 - bentV)) * 10;
fprintf('Approximate resistance of flex sensor at 45 degrees is: %f kOhms\n', bentR);

%% 9. Using the flex sensor and potentiometer to control an RGB LED
% Now we're going to control an RGB LED using the flex sensor and
% potentiometer. To hook up your RGB LED, connect R to pin D12, G to D11,
% and B to D10. Connect the '-' sign on the LED to GND. Leaving your flex
% sensor voltage divider intact, hook up your potentiometer to 3.3V and
% GND. Hook up the wiper pin of the potentiometer to pin A2 on the Arduino.
%
% We're going to control the brightness of the LED using the potentiometer,
% and we'll make the LED change color from green to red depending on how
% much it's been flexed.

nb.pinMode('A1', 'ainput'); % For flex sensor
nb.pinMode('A2', 'ainput'); % For potentiometer
nb.initRGB('D12','D11','D10'); % Initialize the RGB

% First, we want to find our ADC values that bound the range of each
% sensor. The easiest way to do this could be by doing a live plot of each
% sensor and noting what ADC values the range seems to be bound by.

% Uncomment the following line(s) to run a live plot
% nb.livePlot('analog', 'A1');
% nb.livePlot('analog', 'A2');

% Record your max and min ADC values for each sensor:
flexMax = 675;
flexMin = 465;
potMax = 1023;
potMin = 0;

% Let's start our loop:
tic
while toc < 30
    % Since the pot simply controls the brightness, and both the ADC and RGB
    % LED ranges have a minimum value of zero, we can scale the brightness
    % using a ratio of current reading over the max value.
    potVal = nb.analogRead('A2');
    bright = potVal / potMax;

    % Now we normalize the current flex reading to a range between 0 and
    % 255. Let's compare ratios (in this case, read reflects our ADC, and
    % targ reflects the RGB range) This is called linear interpolation:
    % (readVal - readMin) / (readMax - readMin) = (targVal - targMin) / (targMax - targMin)
    % Rearranging:
    % targVal = ((readVal - readMin)/(readMax - readMin))*(targMax - targMin) + targMin

    % Now let's take a reading from the flex sensor. We want to ensure our
    % value is within the range we declared in order for our linear
    % interpolation to work.
    flexVal = nb.analogRead('A1');
    if flexVal > flexMax
        flexVal = flexMax;
    elseif flexVal < flexMin
        flexVal = flexMin;
    end
    
    % Time to linearly interpolate the RGB value from our flex reading.
    rawRGB = ((flexVal - flexMin)/(flexMax - flexMin))*(255 - 0) + 0;

    % To color shift, lets make green correspond to the minimum flex, and
    % red correspond to maximum flex. We'll set redRGB to roundRGB, and
    % greenRGB to whatever is left over.
    redRGB = rawRGB;
    greenRGB = 255 - redRGB;

    % Now we can scale the values by brightness
    redRGB = redRGB * bright;
    greenRGB = greenRGB * bright;

    % Note that our RGB values may not be integers, and they need to be
    % to work with the function we use to control the LED. Lets round them.
    redRGB = round(redRGB);
    greenRGB = round(greenRGB);

    % Finally, set the RGB to the right values (and include an optional
    % pause for less computation/noise)
    nb.setRGB(redRGB, greenRGB, 0);
    pause(0.05);
end
nb.setRGB(0,0,0);

%% 10. EXTENSION (optional)
%  - Use the potentiometer knob position to set the flex sensor bend angle
% which will turn on the onboard LED.

%% X. DISCONNECT
%  Clears the workspace and command window, then
%  disconnects from the nanobot, freeing up the serial port.

clc
delete(nb);
clear('nb');
clear all