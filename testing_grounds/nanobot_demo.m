clc
clear all

% Create an instance of the nanobot class
nb = nanobot('COM7', 115200, 'wifi');

%%
% VALID PINS

%The motor carrier uses many of the pins on your Arduino for interfacing
%with the motors, encoders, and IMU. These are the ones that are still
%open for use as general purpose I/O, and safest to avoid conflicts:

%Valid ANALOG pins with motor carrier installed: A2, A3, A6, A7
%Valid DIGITAL pins with motor carrier installed: D12, D11, D10, D8, D7

%Refer to the Arduino Nano 33 IoT PINOUT in order to find these. 

%%
% LIVE PLOT

%This starts a dynamically updating plot of the accelerometer values. Click
%the "stop" button and wait for the figure to close to exit.
nb.livePlot('accel');

%You can also plot an analog read value. Note that I have to initialize the pin as an analog input ("ainput")
%first:
nb.pinMode('A2','ainput');
nb.livePlot('analog','A2');
%%
% DIGITAL READ

%First set the mode of desired pin to digital input 'dinput'
    %fyi: dinput is INPUT_PULLUP, so will return 1 by default
nb.pinMode('D10','dinput');

%Here is an example of taking a single reading:
val = nb.digitalRead('D10');
fprintf('val = %i\n', val)

%%
% ANALOG READ

%Note that I have to initialize the pin as an analog input ("ainput")
%first:
nb.pinMode('A3','ainput');

%Here is an example of taking a single reading:
val = nb.analogRead('A3');

%Here is an example of taking a second of readings, then averaging:
tic
vals = [];
while (toc < 1)
    vals(length(vals)+1) = nb.analogRead('A3');
end
meanval = mean(vals);
fprintf('mean val = %.0f\n', meanval)

%Here is an example of taking 100 readings, then averaging:
numreads = 100;
vals = zeros(1,numreads);
for i = 1:numreads
    vals(i) = nb.analogRead('A3');
end
meanval = mean(vals);
fprintf('mean val = %.0f\n', meanval)

%%
% DIGITAL WRITE

%First set the mode of the desired pin to output
nb.pinMode('A7','output');

%Then write to that pin either high (1) or low (0)
nb.digitalWrite('A7', 1);

%%
% ACCEL READ

%Here is an example of taking a single accelerometer reading:
val = nb.accelRead();

%The axis values are saved as fields in a structure:
fprintf('x: %.2f, y: %.2f, z: %.2f\n', val.x, val.y, val.z);

%Here is an example of taking 10 accelerometer readings, then averaging
%each axis:
numreads = 10;
vals = zeros(3,numreads);
for i = 1:numreads
    val = nb.accelRead();
    vals(1,i) = val.x;
    vals(2,i) = val.y;
    vals(3,i) = val.z;
end
%Note the index, getting every column in a specific row for each axis:
meanx = mean(vals(1,:));
meany = mean(vals(2,:));
meanz = mean(vals(3,:));
fprintf('mean x: %.2f, mean y: %.2f, mean z: %.2f\n', meanx, meany, meanz);
%%
% SET MOTOR

%Set the motor with NUM, DUTY CYCLE (-100:100)
nb.setMotor(1,25)
pause(1)
nb.setMotor(1,0)

%%
% ENCODER READ

%Specify which encoder to read from (e.g., HA1/HB1 or HA2/HB2)
val = nb.encoderRead(1);
fprintf('counts since last read: %i, counts per second: %i\n', val.counts,val.countspersec);

%%
% SET SERVO

%Set the servo with NUM, ANGLE (0:180)
angle = 0;
while (angle <= 180)
    nb.setServo(1,angle)
    angle = angle+20;
    pause(0.25)
end
clear angle

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

%%
% ULTRASONIC DISTANCE

%Initialize the ultrasonic sensor with TRIGPIN, ECHOPIN
nb.initUltrasonic('D8','D7')

%Take a single ultrasonic reading
val = nb.ultrasonicRead();
fprintf('val = %i\n', val)
%%
% PIEZO BUZZER

%Initialize the piezo with the motor pins being used.
%Only 'M3' or 'M4' are valid for piezo use.
nb.initPiezo('M4');

%Set the piezo tone with FREQUENCY [Hz], DURATION [ms]
nb.setPiezo(600,3000);

%%
% REFLECTANCE

%Initialize the reflectance sensor with default pins D12, D11, D10, D8
nb.initReflectance();

%Take a single reflectance sensor reading
val = nb.reflectanceRead;

%The sensor values are saved as fields in a structure:
fprintf('one: %.2f, two: %.2f, three: %.2f four: %.2f\n', val.one, val.two, val.three, val.four);

%%
% CLOSING OUT

% Close and clear the nanobot instance; if you change the name from "nb",
% make sure to update the clear() function here

delete(nb);
clear('nb');