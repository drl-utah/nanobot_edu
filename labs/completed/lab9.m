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

% nb.pinMode('A1', 'ainput');
% nb.livePlot('analog', 'A1'); % Use me to find your min and max analog values

minADC = 0;
maxADC = 1023;
minBPM = 40;
maxBPM = 180;

%% Defining a function for finding quarter note period
% If we want to change our BPM while the song is playing, we need to
% recalculate the period of a quarter note every time we play a note. To
% make the code *slightly* less cumbersome, we've defined a function at the
% bottom of the page (per MATLAB syntax guidelines) to calculate and return
% the period (in ms) of a quarter note. Go define the function, then return
% here.

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
noteFrequencies = [261.6, 293.7, 329.7, 349.2, 392, 440, 493, 523.3];

% Below is a useful tool for calculating note periods depending on the time
% signature and type of note:
% https://toolstud.io/music/bpm.php

%% 3. Playing an ascending C major scale
% Using your frequency array and your note periods, play an ascending
% C major scale at 100 BPM in 4/4 time.
% Recall, setPiezo() takes the period to play in ms as its second parameter
% (as a whole number), but pause() takes it in seconds.

% Solution:
nb.initPiezo('M4');
nb.pinMode('A1', 'ainput');

for note = 1:length(noteFrequencies)
    qNote = qNoteCalc(nb, minADC, maxADC, minBPM, maxBPM);
    nb.setPiezo(noteFrequencies(note), round(qNote));
    pause(qNote/1000);  % THIS IS IMPORTANT
end

%% 4. Playing 'Hot Cross Buns' on a Piezo Speaker
% Play Hot Cross Buns at 120 BPM. Refer to the music sheet on the Canvas
% lab page.

% Solution:
nb.initPiezo ('M4');
nb.pinMode('A1', 'ainput');

qNote = qNoteCalc(nb, minADC, maxADC, minBPM, maxBPM);

% Bar 1:
nb.setPiezo(noteFrequencies(3), round(qNote*2));
pause(qNote*2/1000);
qNote = qNoteCalc(nb, minADC, maxADC, minBPM, maxBPM);
nb.setPiezo(noteFrequencies(2), round(qNote*2));
pause(qNote*2/1000);
qNote = qNoteCalc(nb, minADC, maxADC, minBPM, maxBPM);
nb.setPiezo(noteFrequencies(1), round(qNote*4));
pause(qNote*4/1000);

qNote = qNoteCalc(nb, minADC, maxADC, minBPM, maxBPM);
nb.setPiezo(noteFrequencies(3), round(qNote*2));
pause(qNote*2/1000);
qNote = qNoteCalc(nb, minADC, maxADC, minBPM, maxBPM);
nb.setPiezo(noteFrequencies(2), round(qNote*2));
pause(qNote*2/1000);
qNote = qNoteCalc(nb, minADC, maxADC, minBPM, maxBPM);
nb.setPiezo(noteFrequencies(1), round(qNote*4));
pause(qNote*4/1000);


% Bar 2:
for note = 1:4
    qNote = qNoteCalc(nb, minADC, maxADC, minBPM, maxBPM);
    nb.setPiezo(noteFrequencies(1), round(qNote));
    pause(qNote/1000);
end

for note = 1:4
    qNote = qNoteCalc(nb, minADC, maxADC, minBPM, maxBPM);
    nb.setPiezo(noteFrequencies(2), round(qNote));
    pause(qNote/1000);
end

% Bar 3:
qNote = qNoteCalc(nb, minADC, maxADC, minBPM, maxBPM);
nb.setPiezo(noteFrequencies(3), round(qNote*2));
pause(qNote*2/1000);
qNote = qNoteCalc(nb, minADC, maxADC, minBPM, maxBPM);
nb.setPiezo(noteFrequencies(2), round(qNote*2));
pause(qNote*2/1000);
qNote = qNoteCalc(nb, minADC, maxADC, minBPM, maxBPM);
nb.setPiezo(noteFrequencies(1), round(qNote*2));
pause(qNote*2/1000);


%% 5. EXTENSION (optional)
%  Take arbitrary character input from your keyboard to play a song.

% Solution:
nb.initPiezo('M4');

getBPM = "Enter the BPM of the song: ";
bpm = input(getBPM);
% fprintf("Please enter an array of notes according to the instructions below...\n");
% fprintf("The input should be a valid natural note (i.e. not sharp or flat) formatted as 'note'\n");
% fprintf("e.g. 'c' for middle C.\n");
% getNotes = "Enter the array: ";
% noteList = input(getNotes);
% getTimes = "Enter the corresponding types of notes, 1 for whole, 2 for half, 4 for quarter, and so on: ";
% timeList = input(getTimes);
fprintf("Please enter a song as a cell array of two-character strings\n");
fprintf("The first character should be a valid natural note (i.e. not sharp or flat) e.g 'c' for middle C\n");
fprintf("NOTE: use 'o' for the C an octave above middle C.\n");
fprintf("The second character should be a letter indicating the type of note.\n")
fprintf("e.g. 'w' for whole, 'h' for half, 'q' for quarter, 'e' for eighth, and 's' for sixteenths.\n")
getCells = "Enter cell array: ";
songCells = input(getCells);

for i = 1:length(songCells)
    if length(songCells{i}) ~= 2
        fprintf("Invalid note-time pair at cell: %d\n", i);
        exit();
    elseif ~contains("cdefgabo", songCells{i}(1))
        fprintf("Invalid note. '%c' is not a valid natural note\n", songCells{i}(1));
        exit();
    elseif ~contains("whqes", songCells{i}(2))
        fprintf("Invalid beat type. '%s' is not a valid beat length\n", songCells{i}(2));
        exit();
    end
end

% Should have valid contents now
beatPeriod = 60 / bpm;  % in seconds
wPeriod = 4 * beatPeriod;
hPeriod = 2 * beatPeriod;
qPeriod = beatPeriod;
ePeriod = 0.5 * beatPeriod;
sPeriod = 0.25 * beatPeriod;

noteKeys = {'c','d','e','f','g','a','b','o'};
noteVals = [261.6, 293.7, 329.7, 349.2, 392, 440, 493, 523.3];
noteMap = containers.Map(noteKeys, noteVals);
timeKeys = {'w','h','q','e','s'};
timeVals = [wPeriod, hPeriod, qPeriod, ePeriod, sPeriod];
timeMap = containers.Map(timeKeys, timeVals);

% Playing logic
songLen = length(songCells);
for i = 1:songLen
    freq = noteMap(songCells{i}(1));
    dur = timeMap(songCells{i}(2)); % in s
    nb.setPiezo(freq, dur * 1000); % convert to ms
    pause(dur);
end



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

function qNotePer = qNoteCalc(nb, minADC, maxADC, minBPM, maxBPM)
    adc = nb.analogRead('A1');
    bpm = round(interp1([minADC, maxADC], [minBPM, maxBPM], adc));
    fprintf("Currently selected BPM: %d\n", bpm);
    % Convert BPM to period (ms)
    qNotePer = round((60/bpm) * 1000);
end