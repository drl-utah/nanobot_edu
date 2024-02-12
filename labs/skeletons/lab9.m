%%%%%%%%%%%%%
% ECE 3610
% LAB 9 -- Actuators 2: Music with Piezoelectricity
%%%%%%%%%%%%%

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
%   - Demonstrate the ability to change your BPM dynamically within either
%   of the two deliverables above.

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
nb = nanobot('COM47', 115200, 'serial');


%% Setting up a potentiometer for variable BPM
% For this lab, we're going to implement the ability to change the BPM of
% the song using our potentiometers. Wire up your pot to 3.3V and GND, with
% the wiper connecting to one of your analog pins
% We're going to need the minimum and maximum ADC values of this setup to
% interpolate between our minimum and maximum BPM values. A good range of
% BPM values for most songs is between 40 and 180 BPM.
% Use the live plot code to find your minimum and maximum ADC values.

% nb.pinMode('?', 'ainput');
% nb.livePlot('analog', '?'); % Use me to find your min and max analog values

minADC = '?';
maxADC = '?';
minBPM = '?';
maxBPM = '?';

%% Defining a function for finding quarter note period
% If we want to change our BPM while the song is playing, we need to
% recalculate the period of a quarter note every time we play a note. To
% make the code *slightly* less cumbersome, we've defined a function at the
% bottom of the page (per MATLAB syntax guidelines) to calculate and return
% the period (in ms) of a quarter note. Go define the function, then return
% here.
% Give your function definition a test here once you've filled it out:
% Make sure you put your interpolation bounds in the same positions you
% specified in the function definition
% E.g. qNote = qNoteCalc(nb, minADC, maxADC, minBPM, maxBPM);
% Adjust your potentiometer and re-run this section, are the BPM values
% different?
qNote = qNoteCalc(nb, '?', '?', '?', '?');


%% 2. Setting the frequencies and periods
% Insert a wire from your piezo into the M4+ port, and the other into the
% M4- port. Make sure you screw down until they are in there firmly.
% If your buzzer doesn't make noise when you try to play something on it,
% check that the screw contacts are touching the wires, not the insulated
% jacket. Try power cycling the switch on your motor carrier board
% (disconnecting and reconnecting battery power). If in doubt, you can also
% try reconnecting to your arduino.
% 
% Modify the code below to contain the correct frequencies corresponding to
% a C major scale. We want to use equal-tempered frequencies for
% key-independence.
%
% Here's some info on the C major scale:
% https://en.wikipedia.org/wiki/C_major
%
% You can find information about the needed frequencies here:
% https://ux1.eiu.edu/~cfadd/3050/Adventures/chapter_12/ch12_4.htm

% Solution:
% Scale frequency array (using equal-tempered frequencies for key-independence)
noteFrequencies = ['?', '?', '?', '?', '?', '?', '?', '?'];

% Below is a useful tool for calculating note periods depending on the time
% signature and type of note:
% https://toolstud.io/music/bpm.php

%% 3. Playing an ascending C major scale
% Using your frequency array and your note periods, play an ascending
% C major scale at 100 BPM in 4/4 time.
% Recall, setPiezo() takes the period to play in ms as its second parameter
% (as a whole number), but pause() takes it in seconds.

nb.initPiezo('?');
nb.pinMode('?', 'ainput');

for note = 1:length('?')
    qNote = '?'; % HINT: Your function definition will prove useful here!
    nb.setPiezo(noteFrequencies(note), round(qNote));
    pause(qNote/1000);  % THIS IS IMPORTANT! It delays the code while allowing the piezo to play.
end

%% 4. Playing 'Hot Cross Buns' on a Piezo Speaker
% Play Hot Cross Buns at 120 BPM. Refer to the music sheet on the Canvas
% lab page. I've given you the first note of bar 1 so you can see the structure of
% each note.
% HINT: Using for loops for Bar 2 of the song may help you reduce code
% clutter.

% Solution:
nb.initPiezo ('?');
nb.pinMode('?', 'ainput');

% Bar 1:
% First note (half): 
qNote = qNoteCalc('?', '?', '?', '?', '?');
nb.setPiezo(noteFrequencies(3), round(qNote*2));
pause(qNote*2/1000);

% Bar 2:

% Bar 3:


%% 5. EXTENSION (optional)
%  Take arbitrary character input from your keyboard to play a song.
% HINT: Since MATLAB doesn't have default support for keyboard event
% listening, we could set up a song by getting input from the command line.
% One way of doing this may be to use CELL ARRAYS, with each cell
% containing a two-letter string. The first letter could indicate the note,
% and the second could indicate the timing (quarter, half, whole, etc.)
% HINT: One way of mapping characters to frequency values and periods is to
% use the matlab containers.Map() function.
% HINT: Cell arrays are indexed as follows:
%   - cellArray(i) -> returns the actual cell object at i
%   - cellArray{i} -> returns the object stored in cell i
%   - cellArray{i}(j) -> returns a value at index j in an array stored in cell i

% Here are some useful resources:
% - https://www.mathworks.com/help/matlab/cell-arrays.html
% - https://www.mathworks.com/help/matlab/ref/containers.map.html

%% X. DISCONNECT
%  Clears the workspace and command window, then
%  disconnects from the nanobot, freeing up the serial port.

clc
delete(nb);
clear('nb');
clear all

%% Function Definitions
% This is a function definition. qNoteCalc is the name of the function,
% qNotePer is the stand-in name for the return value. You need to pass all
% values used by the function from outside as parameters. In this case, its
% the nanobot, as well as our interpolation bounds.

% Replace fixmes below with appropriate interpolation bounds. Order of the parameters doesn't
% matter so long as you pass the correct value to the correct parameter
% position.
function qNotePer = qNoteCalc(nb, fixme2, fixme3, fixme4, fixme5) % min and max adc, min and max BPM

    % Using the parameters you pass to the function, do an analog read,
    % then linearly interpolate to find a corresponding bpm value. Then
    % calculate the period (in ms) of a quarter note at that bpm
    adc = '?'; % Take a reading here!
    bpm = '?'; % Use linear interpolation here!
    fprintf("Currently selected BPM: %d\n", bpm);
    % Convert BPM to period (ms)
    qNotePer = '?'; % 60 1-second beats in a minute, each beat is a quarter note, 1 second is 1000 ms
end