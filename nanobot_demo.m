clc
clear all

% Create an instance of the nanobot class
nb = nanobot('/dev/cu.usbmodem141201', 115200);

%%
% LIVE PLOT

%This starts a dynamically updating plot of the accelerometer values. Click
%the "stop" button and wait for the figure to close to exit.
nb.livePlot('accel');

%You can also plot an analog read value.
nb.livePlot('analog','A0');
%%
% DIGITAL READ

%First set the mode of desired pin to digital input 'dinput'
    %fyi: dinput is INPUT_PULLUP, so will return 1 by default
nb.pinMode('A2','dinput');

%Here is an example of taking a single reading:
val = nb.digitalRead('A2');
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
nb.initUltrasonic('A2','A3')

%Take a single ultrasonic reading
val = nb.ultrasonicRead();
fprintf('val = %i\n', val)
%%
% PIEZO BUZZER

%Initialize the piezo with the pin
nb.initPiezo('A6')

%Set the piezo tone with FREQUENCY [Hz], DURATION [ms]
nb.setPiezo(600,1000)