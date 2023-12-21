%%%%%%%%%%%%%
% ECE 3610
% LAB 9 -- Actuators 2: Music with Piezoelectricity
%%%%%%%%%%%%%

% TODO: Update Canvas from M3 to M4, polarity doesn't matter?

%%%%%%%%%%%%%
% Digital sound production is all around us -- I'm listening to music from
% my computer speakers as I write this! Piezoelectric actuators are some 
% of the simplest possible speakers, and their amenability to 
% miniaturization means that they are ubiquitous. Because of their low 
% fidelity, they are mostly just used as annoying buzzers and alarms. Let's
% make some music instead.

% Deliverables:
%   - Play the C major scale with one note per beat at 100BPM in 4/4 time.
%   - Play Hot Cross Buns using your piezo.

% Extensions:
%   - Take arbitrary character input from your keyboard to play a song.
%%%%%%%%%%%%%%

%% 1. CONNECT TO YOUR NANOBOT
%  Remember to replace the first input argument with text corresponding to
%  the correct serial port you found through the Arduino IDE. Note that the
%  clc and clear all will clear your command window and all saved
%  variables!

clc
clear all
nb = nanobot('COM7', 115200, 'serial');

%% 2. Setting the frequencies and periods
% Insert a wire from your piezo into the M4+ port, and the other into the
% M4- port. Make sure you screw down until they are in there firmly.
% 
% Modify the code below to contain the correct frequencies corresponding to
% a C major scale
%
% Here's some info on the C major scale:
% https://en.wikipedia.org/wiki/C_major
%
% You can find information about the needed frequencies here:
% https://ux1.eiu.edu/~cfadd/3050/Adventures/chapter_12/ch12_4.htm
nb.initPiezo('M4');

% Solution:
% Scale frequency array (using equal-tempered frequencies for key-independence)
noteFrequencies = [261.6, 293.7, 329.7, 349.2, 392, 440, 493, 523.3];

% Below is a useful tool for calculating note periods depending on the time
% signature and type of note:
% https://toolstud.io/music/bpm.php
qNote100BPMPeriod = 0.6; 

arrayLen = length(noteFrequencies);

%% 3. Playing an ascending C major scale
% Using your frequency array and your note periods, play an ascending
% C major scale at 100 BPM in 4/4 time.

% Solution:
for note = 1:length(noteFrequencies)
    nb.setPiezo(noteFrequencies(note), qNote100BPMPeriod * 1000);
end
nb.setPiezo(0, 1);

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