%%%%%%%%%%%%%
% ECE 3610
% LAB 4 -- Force-Sensitive Resisitors
%%%%%%%%%%%%%

%%%%%%%%%%%%%
% In this lab, we will try to get you thinking about the conversion factor
% between a sensor physical stimulus, output electrical quantity, and your
% ADC output, given a circuit you design. We will use a new type of sensor,
% the force-sensitive resistor (FSR).  Perhaps disappointingly, the way these
% sensors operate is not based on midi-chlorians. The FSRs we will be
% working with today are based on mechanical stimuli, and can experience an
% appreciable delay between between the application of stimulus and the
% resulting electrical change. We will directly measure this for your FSR
% (albeit crudely). We will then dive deeper into the interplay between
% sensor sampling and code, specifically to design an interactive game
% using our FSRs. This lab will be significantly more or less difficult
% depending on your programming experience; don't hesitate to ask
% instruction staff for help.
%
% Deliverables:
% - Make a "Hit the Target" thumb wrestling game.
%
% Extensions:
% - Turn the target, player 1, and player 2 values into a live plot
%       - There are multiple ways to do this: one is to append the new
%       values every loop to a sample array, then re-plot the whole thing
%       each time.
% - Think of fun variations on this game using any components you have on
%   hand
%%%%%%%%%%%%%%

%% 1. CONNECT TO YOUR NANOBOT
%  Remember to replace the first input argument with text corresponding to
%  the correct serial port you found through the Arduino IDE. Note that the
%  clc and clear all will clear your command window and all saved
%  variables!

clc
clear all
nb = nanobot('COM47', 115200, 'serial');

%% 2. Physical setup
% We will be creating a voltage divider with the FSR and a single
% resistor. Use the 10k resistor and the FSR to build a voltage divider
% between 3.3V and ground, with the FSR nearer 3.3V.
% Look at the data sheet for the FSR provided on Canvas. Notice Figure 3.
% Assuming the voltage divider operated at 5V with a 10k resistor, what
% output from the ADC would you see at 20g? 100g? 1000g? 

% NOTE: be careful when inserting the FSR into your breadboard, try to
% grab it from as close to the pins as possible.
%
% NOTE: Depending on the quality of your FSR, the minimum resistance could
% range from a few hundred Ohms, to several kOhms. This means that you may
% not be able to read the entire 0-1023 ADC range.
%
% Solution:
% For 10k in the Figure 3 diagram, voltage values appear to be about 0V at
% 20g, 1.8V at 100g, and 3.4V at 1000g. Using a 5V ADC, this would
% correspond to values of ADC = (V / Vcc) * 1023 = 0, 368, 696. 

%% 2.5 Setting up the RGB LED
% Similarly to last lab, wire up the RGB LED. Some example pins could be
% D12 for R, D11 for G, D10 for B, and GND for '-'

%% 3. Changing LED intensity with the FSR
% Write code that makes your onboard LED brighter the harder you squeeze
% the FSR.
% As your FSR may not reduce its resistance to a value negligible compared
% to 10kΩ, you should find the max ADC value of your voltage divider setup.
% This can be done through live plotting, or recording a max value over a
% sampling time.
% Last lab, we used the formal definition of linear interpolation to find
% the corresponding brightness values and colors of the RGB LED. Another
% way we can do this is by using MATLAB's interp1() function!
% https://www.mathworks.com/help/matlab/ref/interp1.html

% HINT: Use the interp1 function
% HINT: setRGB does not accept decimal values. Use the MATLAB round
% function
% HINT: It is good practice to include a short pause in every MATLAB while
% loop which sends data to/from the Arduino. Your PC is much faster than
% your microcontroller.

% For live plotting (uncomment lines below):
% nb.pinMode('A1', 'ainput');
% nb.livePlot('analog', 'A1');
% maxADC = '?'

% For sampling (uncomment lines below):
% % Forcefully squeeze the FSR within 5 seconds to record the max ADC value
% nb.pinMode('A1', 'ainput');
% tic
% vals = [];
% while (toc < 5)
%     vals(length(vals)+1) = nb.analogRead('A1');
% end
% maxADC = max(vals);
% fprintf('max val = %.0f\n', maxADC);

% Solution:
maxVal = 802;

nb.pinMode('A1', 'ainput');
nb.initRGB('D12', 'D11', 'D10');
adcRange = [0, maxVal];
rgbRange = [0, 255];

tic
while(toc < 20) % Run for 20 seconds
    % To smooth out noise:
    %Here is an example of taking 5 readings, then averaging:
    numreads = 5;
    vals = zeros(1,numreads);
    for i = 1:numreads
        vals(i) = nb.analogRead('A1');
    end
    adc = mean(vals);
    
    % Make sure adc is equal to or below maxVal
    if adc > maxVal
        adc = maxVal;
    end

    % Use interp1 to find the intensity value corresponding to adc
    scale = interp1(adcRange, rgbRange, adc);
    brightness = scale / 255;

    % Calculate the corresponding RGB values (change the values being
    % multiplied by brightness to any value between 0 and 255 to shift
    % colors!)
    r = round(brightness * 255);
    g = round(brightness * 0);
    b = round(brightness * 0);

    nb.setRGB(r, g, b);
    pause(0.01);
end
nb.setRGB(0, 0, 0);

%% 4. Measure the rise time
% Measure the approximate rise time in milliseconds between application of
% force and the settling of the FSR resistance value. Rise time is defined
% as time it takes for a value to go from 10 percent of its max value to 90
% percent of its max value.
%
% HINT: If you haven't changed your circuit, your max value from the
% previous section should be used.

% Solution:
nb.pinMode('A1', 'ainput');

% Let's find how many samples we can record over 1 second:
ctr = 0;
tic
while(toc < 1)
    temp = nb.analogRead('A1');
    ctr = ctr + 1;
end
% Thus the period (in seconds) between samples is:
period = 1/ctr;
fprintf('Period between samples is ~%d seconds. Squeeze your FSR hard to find rise time!\n', period);

% Let's find the rise time:
riseLow = 0.1 * maxVal;
riseHigh = 0.9 * maxVal;
ctr = 0;
tic
while(toc < 10) 
    adcVal = nb.analogRead('A1');

    if(adcVal > riseLow)
        while(adcVal < riseHigh)
            adcVal = nb.analogRead('A1');
            ctr = ctr + 1;
        end
        break;
    end
end
% Thus, our rise time is:
riseTime = ctr * period;
fprintf("The rise time is: %d seconds\n", riseTime);

%% 5. Making a FSR "thumb wrestling" game
% Grab a partner and turn your two FSRs into a "thumb wrestling" game using
% a single Arduino and computer. This game should be a race to pinch your
% FSR at a specific, randomly generated force amount for around a full second.
% Turn the LED to either red or blue, depending on who wins first.

% HINT: Have a tolerance for an acceptable +/- from the desired ADC output
% using the "and" operator in the if statement, &&

% HINT: display something like
% fprintf("Target: %.0f | P1: %.0f | P2: %.0f \n",target,p1val,p2val)
% every time in the while loop.

% NOTE: If you prefer to work by yourself on this for whatever reason,
% please ask instruction staff for a second FSR to borrow.

% Solution:
% Setup:
nb.initRGB('D12','D11','D10');
nb.setRGB(0, 0, 0);
plusMinTol = 75; % ADC can fluctuate between 2 * this val centered around target
adcMax = maxVal; % Max ADC value for this setup, based on live read.
adcTarget = randi(adcMax);
lowBnd = adcTarget - plusMinTol;
upBnd = adcTarget + plusMinTol;
p1time = 1;
p2time = 1;
p1Win = 0;
p2Win = 0;

% Let's find how many samples we can record over 1 second:
ctr = 0;
tic
while(toc < 1)
    % To smooth out noise:
    %Here is an example of taking 5 readings, then averaging:
    numreads = 5;
    vals = zeros(1,numreads);
    for i = 1:numreads
        vals(i) = nb.analogRead('A1');
    end
    temp = round(mean(vals));
    ctr = ctr + 1;
end
% Thus the period (in seconds) between smooth samples is:
period = 1/ctr;

% Game Loop:
while(true)
    numreads = 5;
    vals = zeros(1,numreads);
    for i = 1:numreads
        vals(i) = nb.analogRead('A1');
    end
    p1Val = round(mean(vals));

    numreads = 5;
    vals = zeros(1,numreads);
    for i = 1:numreads
        vals(i) = nb.analogRead('A2');
    end
    p2Val = round(mean(vals));
    fprintf("Target: %d | P1: %d | P2: %d\n", adcTarget, p1Val, p2Val);

    if((p1Val >= lowBnd && p1Val <= upBnd) && p1time > 0)
        p1time = p1time - 2*period; % Since 2 reads were done
    elseif(p1time <= 0)
        p1Win = 1;
    else
        p1time = 1;
    end

    if((p2Val >= lowBnd && p2Val <= upBnd) && p2time > 0)
        p2time = p2time - 2*period; % Since 2 reads were done
    elseif(p2time <= 0)
        p2Win = 1;
    else
        p2time = 1;
    end

    if(p1Win || p2Win)
        break
    end
end

% Winner selection
if(p1Win && p2Win)
    fprintf("WINNER: It's a tie! :O\n")
    nb.setRGB(0, 128, 0);
elseif(p1Win)
    fprintf("WINNER: Player 1 wins!\n")
    nb.setRGB(128, 0, 0);
else
    fprintf("WINNER: Player 2 wins!\n")
    nb.setRGB(0, 0, 128);
end
pause(5);
fprintf("Thanks for playing!\n");
nb.setRGB(0, 0, 0);

%% 6. EXTENSION (optional)
% - Turn the target, player 1, and player 2 values into a live plot
%       - There are multiple ways to do this: one is to append the new
%       values every loop to a sample array, then re-plot the whole thing
%       each time.
% - Think of fun variations on this game using any components you have on
%   hand

%% 7. DISCONNECT
%  Clears the workspace and command window, then
%  disconnects from the nanobot, freeing up the serial port.

clc
delete(nb);
clear('nb');
clear all