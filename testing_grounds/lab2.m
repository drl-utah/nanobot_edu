%%%%%%%%%%%%%
% ECE 3610
% LAB # -- SHORTNAME
%%%%%%%%%%%%%

%%%%%%%%%%%%%
% Brief text description of the purpose of this lab.
%
% Brief text description of the objectives of this lab, including
% deliverable(s) and extension(s). 
%%%%%%%%%%%%%%

%% 1. CONNECT TO YOUR NANOBOT
%  Remember to replace the first input argument with text corresponding to
%  the correct serial port you found through the Arduino IDE. Note that the
%  clc and clear all will clear your command window and all saved
%  variables!

clc
clear all
nb = nanobot('/dev/cu.usbmodem21101', 115200);

%% X. LAB CODE (sometimes this should be multiple sections)
%  Brief text description of this section of code.
%
%  HINT: It can be helpful to provide hints like this, if necessary.

%% X. EXTENSION (optional)
%  Brief text description of this section of code.
%
%  HINT: It can be helpful to provide hints like this, if necessary.


%% X. DISCONNECT
%  Clears the workspace and command window, then
%  disconnects from the nanobot, freeing up the serial port.

clc
clear all
delete(nb);
clear('nb');