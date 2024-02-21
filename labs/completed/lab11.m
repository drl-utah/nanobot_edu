%%%%%%%%%%%%%
% ECE 3610
% LAB 11 -- Encoders and PID control of a DC motor
%%%%%%%%%%%%%

%%%%%%%%%%%%%
% In this lab you will work IN TEAMS to tune a PID controller that sets a
% DC motor duty cycle to achieve a desired wheel speed and stabilizes any internal
% turning motion. Low level controllers like this are critical for wheeled
% robot motion, and will be necessary for your project!
%
% Deliverables:
%   - Find your encoder's approximate sampling frequency
%   - Target 10% duty cycle in speed with amplitude oscilations less than
%   2%, and a rise time of less than 500 ms.
%   
% Extensions:
%   - Use an additional PID loop to stabilize the robot's turning using the
%   encoder counts.
%%%%%%%%%%%%%%

%% 1. CONNECT TO YOUR NANOBOT
%  Remember to replace the first input argument with text corresponding to
%  the correct serial port you found through the Arduino IDE. Note that the
%  clc and clear all will clear your command window and all saved
%  variables!

clc
clear all
nb = nanobot('COM50', 115200, 'serial');

%% 2. Approximating the encoder's sampling frequency
% To gain an approximation of the encoder's sampling frequency, we want to
% figure out how many times it can sample over a period of time (say, a second)
% without experiencing aliasing artifacts. Beyond a certain motor speed, we
% will not recieve accurate counts. Experiment around to try and find the
% encoder's approximate sampling frequency.

tic
runtime = 1;
% 14% duty cycle seems to be the lowest that doesn't cause aliasing
% wraparound with the encoder.
nb.setMotor(1,10); % orig 10
nb.setMotor(2,10);
pause(1); % Allow motor to reach constant speed
val = nb.encoderRead(1);
pause(runtime);
val = nb.encoderRead(1);

fprintf('counts since last read: %i, counts per second: %i\n', val.counts,val.countspersec);
nb.setMotor(1,0);
nb.setMotor(2,0);

% Did val.counts samples in runtime sec, calculate sample freq:
sampleFreq = abs(val.counts / runtime);


% disp(vals) %hint: if this is how many samples you got in runtime, then what is the sampling frequency?

% Answer: It seems that the highest value I can read in counts over 1
% second is around 3k at a duty cycle of 14%, so the sample frequency must be at least that
% much. The sign of the duty cycle corresponds to the direction in which
% the motor spins.

val.counts = 0;
val.countspersec = 0;

%% Creating a PID controller
% In this section, we will be using a PID controller to approach a given
% duty cycle value smoothly over time. You will need to tune your PID
% parameters in order to get your motor signal output 10% duty cycle with
% amplitude oscilations less than 2%, and a rise time of less than 500 ms.

% PID terms for speed
KpS = 1; % Proportional gain
KiS = 0.0; % Integral gain
KdS = 0.001; % Derivative gain

% EXTENSION
% PID terms to maintain straight line
% KpT = 0.00;
% KiT = 0.0; 
% KdT = 0.000;

curSpeed = 0;
endSpeed = 10;

% Initialize PID variables
prevErrorS = 0;
integralS = 0;

% EXTENSION
%prevErrorT = 0;
%integralT = 0;


speeds1 = 0;
speeds2 = 0;
times = 0;

prevTime = 0;

runTime = 3;

tic
pause(0.03)
while toc < runTime

    dt = toc - prevTime;
    prevTime = toc;

    errorS = endSpeed - curSpeed;

    integralS = integralS + errorS * dt;

    derivativeS = (errorS - prevErrorS) / dt;

    pidS = KpS * errorS + KiS * integralS + KdS * derivativeS;

    curSpeed = curSpeed + pidS;


    % EXTENSION
    % leftCountsPerSec = nb.encoderRead(2).counts;
    % rightCountsPerSec = -nb.encoderRead(1).counts; % Negative due to flipped encoder

    % Calculate the velocity error
    % errorT = leftCountsPerSec - rightCountsPerSec;
    % integralT = integralT + errorT * dt;
    % derivativeT = (errorT - prevErrorT)/dt;

    % Apply PID control to adjust motor speeds
    %correction = KpT * errorT + KiT * integralT + KdT * derivativeT;



    % Adjust motor speeds to maintain straight motion
    motor2New = curSpeed; %- correction; EXTENSION
    motor1New = curSpeed; %+ correction; EXTENSION
    nb.setMotor(2, motor2New);
    nb.setMotor(1, motor1New);

    prevErrorS = errorS;
    % prevErrorT = errorT; % EXTENSION

    speeds1(end+1) = motor1New;
    speeds2(end+1) = motor2New;
    times(end+1) = toc;
end
pause(0.1);
nb.setMotor(1,0);
nb.setMotor(2,0);

clf
plot(times, speeds1)
hold on
plot(times, speeds2)
yline(endSpeed,'-','Target')
ylim([0, endSpeed+10])
xlim ([0 runTime])
hold off

%% Stop motors (if running)
nb.setMotor(1,0);
nb.setMotor(2,0);


%% X. DISCONNECT
%  Clears the workspace and command window, then
%  disconnects from the nanobot, freeing up the serial port.

clc
delete(nb);
clear('nb');
clear all