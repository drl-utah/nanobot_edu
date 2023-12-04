%%%%%%%%%%%%%
% ECE 3610
% LAB 4 -- Scale Factors and Time Constants
%%%%%%%%%%%%%

% TODO: Figure out interp1, add extensions, test with RGB LED

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
nb = nanobot('COM7', 115200, 'serial');

%% 2. Physical setup
% We will be creating a voltage divider with the FSR and a single
% resistor. First, use the provided FSR datasheet <describe location> to
% decide on an appropriate resistor value to use in your voltage divider.
% Assume we will be attempting to measure forces between 20g and 1000g.
% With the resistor value you've chosen, what output from the ADC will you
% see at 20g? 100g? 1000g?
% 
% Note: be careful when inserting the FSR into your breadboard, try to
% grab it from as close to the pins as possible.
%
% Solution:
% As the datasheet uses Rm at +5V, I assume we will use +5V in this lab.
% We are intending to measure between 20-1000g, and we want to make as much
% use of our voltage range as possible while still retaining some linearity
% of control. 30k or 47k are thus decent values for control. Assume 30k for
% now.
%
%% 3. Changing LED intensity with the FSR
% Write code that makes your onboard LED brighter the harder you squeeze
% the FSR. Ideally, your brightness should cover the full scale range of
% your ADC.
%
% HINT: Use the interp1 function
% HINT: setRGB does not accept decimal values. Use the MATLAB round
% function
% HINT: It is good practice to include a short pause in every MATLAB while
% loop which sends data to/from the Arduino. Your PC is much faster than
% your microcontroller.

% Solution:
tic
while(toc < 20) % test for 20 sec
    aData = nb.analogRead(1); % read analog data from pin A1
    colorNorm = round(255 * (aData / 1023)); % 255 is max range we want, 1023 bit res of ADC
    nb.setRGB(colorNorm, colorNorm, colorNorm);
    pause(0.001);
end

%% 4. Measure the rise time
% Measure the approximate rise time in milliseconds between application of
% force and the settling of the FSR resistance value.
%
% HINT: Try doing a live plot of your analog values to get a sense of the
% starting and resting values to check for

% Solution:
% I found that ADC 100 was a good lower threshold for no squeeze, and 800
% was good as max squeeze
msCount = 0;
while(true)
    if(nb.analogRead(1) > 100)
        while(nb.analogRead(1) < 800)
            pause(0.001);
            msCount = msCount + 1;
        end
        break
    end
end
fprintf("rise time was %d ms\n", msCount);

%% 5. Making a FSR "thumb wrestling" game
% Grab a partner and turn your two FSRs into a "thumb wrestling" game using
% a single Arduino and computer. This game should be a race to pinch your
% FSR at a specific, randomly generated force amount for a full second.
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
nb.setRGB(0, 0, 0);
plusMinTol = 50; % ADC can fluctuate between 2 * this val centered around target
adcMax = 900; % Max ADC value for this setup, based on live read.
adcTarget = randi(adcMax);
lowBnd = adcTarget - 50;
upBnd = adcTarget + 50;
p1Ctr = 1000;
p2Ctr = 1000;
p1Win = 0;
p2Win = 0;

% Game Loop:
while(true)
    p1Val = nb.analogRead(1);
    p2Val = nb.analogRead(2);
    fprintf("Target: %d | P1: %d | P2: %d\n", adcTarget, p1Val, p2Val);

    if((p1Val >= lowBnd && p1Val <= upBnd) && p1Ctr ~= 0)
        p1Ctr = p1Ctr - 1;
    elseif(p1Ctr == 0)
        p1Win = 1;
    else
        p1Ctr = 1000;
    end

    if((p2Val >= lowBnd && p2Val <= upBnd) && p2Ctr ~= 0)
        p2Ctr = p2Ctr - 1;
    elseif(p2Ctr == 0)
        p2Win = 1;
    else
        p2Ctr = 1000;
    end

    if(p1Win || p2Win)
        break
    end

    pause(0.001);   % May need to toy with this to get ~1s
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