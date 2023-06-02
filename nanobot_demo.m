clc
clear all

% Create an instance of the nanobot class
nb = nanobot('/dev/cu.usbmodem141101', 115200);

%%
% LIVE PLOT
nb.livePlot('accel');
% nb.livePlot('analog','A0');
%%
% DIGITAL READ

nb.digitalRead('D2')
%%
% ANALOG READ

nb.analogRead('A1')
%%
% DIGITAL WRITE

nb.digitalWrite('D3', 1)
%%
% ACCEL READ

nb.accelRead()

%%
% LED WRITE

tic
while (toc < 3)
    nb.ledWrite(1)
    pause(0.05)
    nb.ledWrite(0)
    pause(0.05)
end
nb.ledWrite(0)