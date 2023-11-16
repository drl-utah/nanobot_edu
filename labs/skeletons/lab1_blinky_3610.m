%%%%%%%%%%%%%
% ECE 3610
% LAB 1 -- BLINKY
%%%%%%%%%%%%%

%%%%%%%%%%%%%
% Making an LED blink is the "Hello World" of using a microcontroller; it's
% a clear way to draw a connection between code you write and a physical,
% visible output. 
%
% Follow the detailed lab instructions on Canvas to write a program that
% connects to your "nanobot," then blinks the onboard LED at 5Hz with a 
% 50% duty cycle for 3 seconds. For an optional challenge, you can make the
% LED blink out "Hello World" in Morse code. 
%%%%%%%%%%%%%%

%%%%%%%%%%%%%%
% TIP: "RUN SECTION"
% I recommend breaking up your code into distinct sections, as in this lab,
% and running each one individually. For example, you don't need to
% re-connect to your nanobot every time you change a variable! Separate
% sections by %% and a carriage return, click inside the "section" you want
% to run, then press "Run Section" or CTRL+ENTER.
%%%%%%%%%%%%%%

%% 1. CONNECT TO YOUR NANOBOT
%  Remember to replace the first input argument with text corresponding to
%  the correct serial port you found through the Arduino IDE. Note that the
%  clc and clear all will clear your command window and all saved
%  variables!

clc
clear all
nb = nanobot('/dev/cu.usbmodem21101', 115200);

%% 2. CALCULATE YOUR LED PARAMETERS
%  First calculate some of the important values for this program using the
%  desired behavior.

% These are variables you can choose:
blinkTime = ; %time in seconds for the blink program to run
blinkFrequency = ;  %frequency that LED blinks in Hz
dutyCycle = ; %duty cycle of the LED in decimal form (e.g., 20% is 0.2)

% These are variables which should be calculated:
blinkPeriod = ;
onTime = ;
offTime = ; 

%% 3. STEADY BLINK YOUR LED 
%  Use the calculated parameters from the previous section to blink your
%  LED with the desired pattern.

tic
while (toc < blinkTime)
    nb.ledWrite(1)
    pause( )               % How long should the LED stay on?
    nb.ledWrite(0)
    pause( )               % How long should the LED stay off?
end
nb.ledWrite(0)

%% 4. BLINK IN MORSE (optional)
%  Rules of International Morse Code:
%  1. The length of a dot is one unit.
%  2. A dash is three units.
%  3. The space between parts of the same letter is one unit.
%  4. The space between letters is three units.
%  5. The space between words is seven units.
%
%  HINT: Define the length of one unit in seconds, make a matrix with the
%  Morse code for each letter in each word, then iterate through the matrix. 
%  I provided most of the code for this first extension for you below.

unitTime = ; 
words = 2;

%  Fill in the corresponding code for "WORLD" yourself:
letters = ["....",".",".-..",".-..","---"; W O R L D]; 

%  HINT: The trickiest part of this code is getting the integer multiples of
%  "unitTime" correct for the pauses after each part of the letter, each
%  letter, and each word. Think about the TOTAL "units" which will elapse in
%  each case, based on execution of the for loops.

nb.ledWrite(0)
for i = 1:words
    for j = 1:length(letters(i,:))
        letter = letters(i,j);
        for z = 1:length(letter{1})
            if letter{1}(z) == '.'
                nb.ledWrite(1)
                pause(unitTime)
            elseif letter{1}(z) == '-'
                nb.ledWrite(1)
                pause(x * unitTime)          % ?
            end
            nb.ledWrite(0)
            pause(x * unitTime)              % ?
        end
        nb.ledWrite(0)
        pause(x * unitTime)                  % ?
    end
    nb.ledWrite(0)
    pause(x * unitTime)                      % ?
end

%% 5. DISCONNECT
%  Clears the workspace and command window, then
%  disconnects from the nanobot, freeing up the serial port.

clc
clear all
delete(nb);
clear('nb');