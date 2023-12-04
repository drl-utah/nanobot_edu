%%%%%%%%%%%%%
% ECE 3610
% LAB 4 -- Scale Factors and Time Constants
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
nb = nanobot('COM7', 115200, 'serial');

%% 2. Physical setup
%  We will be creating a voltage divider with the FSR and a single
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
%%
%% X. EXTENSION (optional)
%  Brief text description of this section of code.
%
%  HINT: It can be helpful to provide hints like this, if necessary.


%% X. DISCONNECT
%  Clears the workspace and command window, then
%  disconnects from the nanobot, freeing up the serial port.

clc
delete(nb);
clear('nb');
clear all