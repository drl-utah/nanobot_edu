%%%%%%%%%%%%%
% ECE 3610
% LAB 11 -- Encoders and PID control of a DC motor
%%%%%%%%%%%%%

%%%%%%%%%%%%%
% In this lab you will work IN TEAMS to tune a PID controller that sets a
% DC motor duty cycle to achieve a desired wheel RPM. Low level controllers
% like this are critical for wheeled robot motion, and will be necessary 
% for your project!
%
% Deliverables:
%   - Target 100RPM with a rise time under 400ms, average steady state
%   error < 20rpm, and oscillations < ~10rpm.
%%%%%%%%%%%%%%

%% 1. CONNECT TO YOUR NANOBOT
%  Remember to replace the first input argument with text corresponding to
%  the correct serial port you found through the Arduino IDE. Note that the
%  clc and clear all will clear your command window and all saved
%  variables!

clc
clear all
nb = nanobot('COM7', 115200, 'serial');

%% 2. Approximating the encoder's sampling frequency
% To gain an approximation of the encoder's sampling frequency, we want to
% figure out how many times it can sample over a period of time (say, a second)
% without experiencing aliasing artifacts. Beyond a certain motor speed, we
% will not recieve accurate counts

tic
runtime = 1;
% 14% duty cycle seems to be the lowest that doesn't cause aliasing
% wraparound with the encoder.
nb.setMotor(3,10); % orig 10
pause(1); % Allow motor to reach constant speed
val = nb.encoderRead(1);
pause(runtime);
val = nb.encoderRead(1);

fprintf('counts since last read: %i, counts per second: %i\n', val.counts,val.countspersec);
nb.setMotor(3,0);

% Did val.counts samples in runtime sec, calculate sample freq:
sampleFreq = abs(val.counts / runtime);


% disp(vals) %hint: if this is how many samples you got in runtime, then what is the sampling frequency?

% Answer: It seems that the highest value I can read in counts over 1
% second is around 3.3k at a duty cycle of 14%, so the sample frequency must be at least that
% much. The sign of the duty cycle corresponds to the direction in which
% the motor spins.

val.counts = 0;
val.countspersec = 0;

%% PID Tuning to match a desired RPM
pause(0.5);
dSample = 1 / 3300;
vals = 0;
oldval = 0;
val = 0;
motorval = 0;

tic
vals = nb.encoderRead(1);
rpm_targ = 100;  %The goal RPM

rpms = 0;
times = 0;

error_sum = 0;
last_error = 0;

kp = 0.1;         %proportional gain
ki = 0.023;        %derivative gain
kd = 0.013;         %integral gain

runtime = 2;
pause(0.03);
while toc < runtime
    vals = nb.encoderRead(1); %this gets the encoder reading since the last time it was called
   
    times(end+1) = toc;
    %Calculate RPM from count_delta here:
        %hint 1: use the time between samples estimated from the first
        %section
        %hint 2: there are ~720 counts per rotation
    rpm =  (vals.counts / (times(end) - times(end - 1))) * (60 / 720);  %YOU WILL NEED TO EDIT THIS
    
    rpms(end+1) = rpm;
    
    
    error = rpm_targ - rpm;
    error_sum = error_sum + error;
    error_delta = last_error - error;
    last_error = error;

    %Write the code for your controller output here, using the gain
    %variables and the three errors computed above:
    control = (kp * error) + (ki * error_sum) + (kd * error_delta); %YOU WILL NEED TO EDIT THIS
    
    %Caps the motor duty cycle at +/- 50
    if control > 50
        control = 50;
    end 
    if control < -50
        control = -50;
    end
   
    nb.setMotor(3,round(control));
end

pause(0.1);
nb.setMotor(3,0);

clf
hold on
plot(times,rpms)
yline(rpm_targ,'-','Target')
ylim([0, rpm_targ+50])
xlim ([0 runtime])
hold off

% The control constants seem to work well for values between 50 and 200
% RPM.

%% X. DISCONNECT
%  Clears the workspace and command window, then
%  disconnects from the nanobot, freeing up the serial port.

clc
delete(nb);
clear('nb');
clear all