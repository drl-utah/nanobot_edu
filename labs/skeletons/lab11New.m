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
%   - Target 100 RPM with a rise time under 400ms, average steady state
%   error < 20rpm, and oscillations < ~10rpm.
%%%%%%%%%%%%%%

%% 1. CONNECT TO YOUR NANOBOT
%  Remember to replace the first input argument with text corresponding to
%  the correct serial port you found through the Arduino IDE. Note that the
%  clc and clear all will clear your command window and all saved
%  variables!

clc
clear all
nb = nanobot('COM47', 115200, 'serial');

%% 2. Approximating the encoder's sampling frequency
% To gain an approximation of the encoder's sampling frequency, we want to
% figure out how many times it can sample over a period of time (say, a second)
% without experiencing aliasing artifacts. Beyond a certain motor speed, we
% will not recieve accurate counts. Experiment around to try and find the
% encoder's approximate sampling frequency.

% HINT: try recording how many counts you get in one second at different
% duty cycles. Beware, if you go too high, your motor will spin too fast
% for your encoder to read accurately, and you will see your counts per
% second decrease as you increase the duty cycle. If you go even faster,
% your encoder may not even register any counts! 
% HINT: Start at a low duty cycle

tic
runtime = '?';

maxEncoderDuty = '?'; % Set your duty cycle
nb.setMotor('?',maxEncoderDuty);
pause(1); % Allow motor to reach constant speed
val = nb.encoderRead(1); % Initial read
pause(runtime);
val = nb.encoderRead(1); % Read again to get counts since last read
nb.setMotor('?',0);

fprintf('counts since last read: %i, counts per 1/100th second: %i\n', val.counts,val.countspersec);

% Did val.counts samples in runtime sec, calculate sample freq:
sampleFreq = '?'; % how many counts did you get over your runtime?

val.counts = 0;
val.countspersec = 0;

%% PID Tuning to match a desired RPM

% Modify these (Tip: tune proportional first, then integral or derivative)
kp = '?';         %proportional gain
ki = '?';           %integral gain
kd = '?';         %derivative gain

rpm_targ = 100;  %The goal RPM

integral = 0;
prevError = 0;

vals = 0;

% For graphing
rpms = 0;
times = 0;

% Change me to change how long the program runs
runtime = 3;

vals = nb.encoderRead(1);
tic
pause(0.03); % Small delay to avoid initial case dt blowing up
while toc < runtime
    vals = nb.encoderRead(1); % Get the counts since the last time we called
   
    times(end+1) = toc; % Collect for graphing

    dt = times(end) - times(end - 1); % Compute the time difference between the last loop and this one.

    %Calculate RPM:
    % Hint 1: You can get your counts per second either by dividing your
    % counts since last read by dt, or use the val.countspersec
    % estimation multiplied by 100.

    % Hint 2: There are about 1440 encoder counts per revolution of the wheel 
    
    % Hint 3: Remember, we want this result in revolutions per minute (RPM)
    rpm = '?';
    
    rpms(end+1) = rpm; % Collect for graphing
    
    
    error = '?'; % Difference between our target and current rpm 
    integral = '?'; % Running sum adding the error over the time step
    derivative = '?'; % Rate of change estimate using the difference
                                         % between the current and prior error over the
                                         % time between them
                                         
    prevError = error;

    %Write the code for your controller output here, using the gain
    %variables and the three errors computed above:
    control = '?'; % Put the error terms and coefficients together
                                                                  % into one control signal!
    
    % Caps the motor duty cycle at +/- max encoder duty cycle to prevent
    % aliasing
    if control > maxEncoderDuty
        control = maxEncoderDuty;
    end 
    if control < -maxEncoderDuty
        control = -maxEncoderDuty;
    end
   
    nb.setMotor('?', control); % Send the control signal to the motor.
end

nb.setMotor('?',0);

% Graphing stuff
clf
hold on
plot(times,rpms)
yline(rpm_targ,'-','Target')
ylim([0, rpm_targ+50])
xlim ([0 runtime])
hold off

%% X. DISCONNECT
%  Clears the workspace and command window, then
%  disconnects from the nanobot, freeing up the serial port.

clc
delete(nb);
clear('nb');
clear all