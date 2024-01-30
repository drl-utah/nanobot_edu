%%%%%%%%%%%%%
% ECE 3610
% LAB 7 -- Sensors Milestone
%%%%%%%%%%%%%

%%%%%%%%%%%%%
% This is the first lab where you will work with the robots used in
% your final project. To start, we will work on line detection and
% interfacing with the IR reflectance sensors on the robot.
% As there are only 20 robots to work with, you will be working in groups to
% accomplish this lab.
%
% Deliverables:
% - Demonstrate functioning code that allows the robot to detect the
% presence of a line of black electrical tape against a white background,
% and determine its location relative to the line. Have the robot change an
% RGB LED's color and brightness depending on which state it's in.
%%%%%%%%%%%%%%

%% 1. CONNECT TO YOUR NANOBOT
%  Remember to replace the first input argument with text corresponding to
%  the correct serial port you found through the Arduino IDE. Note that the
%  clc and clear all will clear your command window and all saved
%  variables!

clc
clear all
nb = nanobot('COM49', 115200, 'serial');

%% 2. Connecting to the reflectance array
% Follow the wiring diagram on Canvas to connect your Arduino to the
% reflectance arrays. Included below is a short key with the connections
% for reference.
%
% Give your reflectance sensor a test using some demo code to verify that
% everything is wired correctly.
% 
% KEY:
% Pololu Array Pin | Arduino Pin
%     1 | V_IN
%     4 | D8 (IR1)
%     9 | D10 (IR2)
%    11 | GND
%    12 | GND
%    14 | +5V
%    18 | D12 (IR4)
%    22 | D11 (IR3)
% JP1-2 | +5V

%Initialize the reflectance sensor with default pins D12, D11, D10, D8
nb.initReflectance();

%Take a single reflectance sensor reading
val = nb.reflectanceRead;

%The sensor values are saved as fields in a structure:
fprintf('one: %.2f, two: %.2f, three: %.2f four: %.2f\n', val.one, val.two, val.three, val.four);

%% 3. How the reflectance array works
% In essence, this reflectance array works using an infrared (IR) LED, a
% phototransistor, and a capacitor to detect how much IR light is able to
% reflect off of the surface beneath the sensor. Prior to a reading, the
% capacitor is charged and stands by. When a reading is taken, the
% capacitor is allowed to discharge at a rate dictated by the
% phototransistor, dependent on the light it recieves. The time it takes
% for the capacitor to discharge to a certain voltage is recorded and
% reported as a measure of the reflectance of the surface below the sensor.
% On your reflectance sensors, this time is measured in microseconds.
% Higher values correspond to less reflective surfaces, and vice versa.

% For more detailed info, https://www.pololu.com/docs/0J13/2 is a good
% resource.

%% 4. Thinking about states
% For basic line following, we can select a value threshold to turn the
% sensors into 2-state sensors (light vs dark). Since we have 4 sensors
% being used on this board, what are the possible combinations of light and
% dark values we could see from the array? 
%
% HINT: You can think about this similarly to different values of a 4-bit
% binary number. E.g. '0110' for light-dark-dark-light (this could mean
% centered on the black electrical tape)
% HINT: Not all the combinations of 1s and 0s make sense for detecting a
% single line, can you identify which ones?

% Put your answers here (commented):
% //////////////////////////////////
% Valid states:
% 0000 - lost
% 1111 - somewhere in the 90 +/- 36 degree arc perpendicular to the line
%
% 0001 - detect line to right, angle unknown within 180 deg
% 0010 - aligned, robot offset left
% 0100 - aligned, robot offset right
% 1000 - detect line to left, angle unknown within 180 deg
%
% 0011 - robot offset left, angle unknown within 180 deg
% 0110 - perfectly aligned and centered
% 1100 - robot offset right, angle unknown within 180 deg
%
% 0111 - robot offset slight left, 180 deg unknown w/ ~45 deg cutout ctr
% 1110 - robot offset slight right, 180 deg unknown w/ ~45 deg cutout ctr
% (cutouts due to sensor spacing making detection within central 45 deg
% impossible due to geometry)
%
% Invalid states (line not detected between two sensor detections [broken line]):
% 0101
% 1010
% 1001
% 1011
% 1101
% //////////////////////////////////

%% Line detection
% In addition to your reflectance array, wire up your RGB LED module so
% that we can see our states expressed through colors in addition to
% command line output. Find a good threshold value that corresponds to
% cutoff between light and dark reflectance values.

nb.initReflectance();
nb.initRGB('D4','D3','D2');
thresh = 1200; % Set this threshold to an appropriate value between light and dark

while (true)
    % Take a reading
    val = nb.reflectanceRead();

    % Check thresholding
    bOne = (val.one >= thresh); % should be true/false
    bTwo = (val.two >= thresh);
    bThree = (val.three >= thresh);
    bFour = (val.four >= thresh);

    % Create the word:
    state = 0b0000;
    if bOne
        state = state + 0b1000;
    end
    if bTwo
        state = state + 0b0100;
    end
    if bThree
        state = state + 0b0010;
    end
    if bFour
        state = state + 0b0001;
    end

    % Cases:
    switch state
        case 0b0110
            fprintf("0110 | Centered!\n"); % 2 HI
            nb.setRGB(0,255,0);
        case 0b0000
            fprintf("0000 | No line detected...\n"); % 0 HI
            nb.setRGB(0,0,0);
        case 0b1111
            fprintf("1111 | Facing the line...\n"); % 4 HI
            nb.setRGB(128,128,128);
        case 0b0001
            fprintf("0001 | Line detected to the right!\n"); % 1 HI
            nb.setRGB(0,0,255);
        case 0b1000
            fprintf("1000 | Line detected to the left!\n"); % 1 HI
            nb.setRGB(255,0,0);
        case 0b0011
            fprintf("0011 | Closer to line on the right!\n"); % 2 HI
            nb.setRGB(0,0,128);
        case 0b1100
            fprintf("1100 | Closer to line on the left!\n"); % 2 HI
            nb.setRGB(128,0,0);
        case 0b0111
            fprintf("0111 | Angled to line on the right!\n"); % 3 HI
            nb.setRGB(0,128,128);
        case 0b1110
            fprintf("1110 | Angled to line on the left!\n"); % 3 HI
            nb.setRGB(128,128,0);
        case 0b0010
            fprintf("0010 | Aligned and offset slight left!\n"); % 1 HI
            nb.setRGB(0,255,128);
        case 0b0100
            fprintf("0100 | Aligned and offset slight right!\n"); % 1 HI
            nb.setRGB(128,255,0);
        otherwise
            fprintf("ERROR: Unexpected case, state is %s...\n", dec2bin(state));
            nb.setRGB(0,0,0);
    end
    pause(0.05); % Slow down to make output comprehensible
end