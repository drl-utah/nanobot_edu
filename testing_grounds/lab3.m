%%%%%%%%%%%%%
% ECE 3610
% LAB 3 -- Analog Inputs and Variable resistances
%%%%%%%%%%%%%

% FIXME SWAP FLEX SENSOR FOR RGB BREAKOUT

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
% - Turn on the onboard LED when the flex sensor is bent more than 45
% degrees
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
nb = nanobot('COM7', 115200, 'wifi');

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

%% 8. Using the flex sensor to turn on an LED
%  Write code to turn on the onboard LED when the flex sensor is bent more
%  than 45 degrees, and off when it is flat. I recommend using the live
%  plot to get a sense of how much you need to bend it.

% Solution:
nb.pinMode('A1', 'ainput');
while(true)
    numreads = 10;
    vals = zeros(1,numreads);
    for i = 1:numreads
        vals(i) = nb.analogRead('A1');
    end
    meanval = mean(vals);
    if(meanval > 560)
        nb.ledWrite(1);
    else
        nb.ledWrite(0);
    end
end

%% 9. Calculating approximate 45-degree resistance
%  Use the analog values to calculate the approximate resistance of the
%  sensor when it is bent 45 degrees.

% Solution:
bentV = (560 / 1023) * 3.3;
bentR = (bentV / (3.3 - bentV)) * 10;
fprintf('Approximate resistance of flex sensor at 45 degrees is: %f kOhms\n', bentR);
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