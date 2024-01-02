clear; clc; close all; instrreset;

%% Connect to Device
r = MKR_MotorCarrier;

%% CALCULATE THE SAMPLE RATE
vals = 0;

tic
runtime = 1;
r.motor(3,10);

while toc < runtime
    val = r.readEncoderPose;
    vals = vals + 1;
end

r.motor(3,0);
disp(vals) %hint: if this is how many samples you got in runtime, then what is the sampling frequency?

%% DETERMINE ENCODER READING DIRECTION
tic
runtime = 1;
r.motor(3,10);
while toc < runtime
    val = r.readEncoderPose
end
tic
r.motor(3,-10);
while toc < runtime
    val = r.readEncoderPose
end
r.motor(3,0);

%% PID CONTROL FOR RPM
r.resetEncoder(1);
pause(0.5);
vals = 0;
oldval = 0;
val = 0;
motorval = 0;

tic
rpm_targ = 100;  %The goal RPM

rpms = 0;
times = 0;

error_sum = 0;
last_error = 0;

kp = 0;         %proportional gain
kd = 0;        %derivative gain
ki = 0;         %integral gain

runtime = 2;

while toc < runtime
    count_delta = r.readEncoderPose; %this gets the encoder reading since the last time it was called
    
    %Calculate RPM from count_delta here:
        %hint 1: use the time between samples estimated from the first
        %section
        %hint 2: there are ~720 counts per rotation
    rpm =  0;  %YOU WILL NEED TO EDIT THIS

    rpms(end+1) = rpm;
    times(end+1) = toc;
    
    error = rpm_targ - rpm;
    error_sum = error_sum + error;
    error_delta = last_error - error;
    last_error = error;

    %Write the code for your controller output here, using the gain
    %variables and the three errors computed above:
    control = 0; %YOU WILL NEED TO EDIT THIS
    
    %Caps the motor duty cycle at +/- 50
    if control > 50
        control = 50;
    end 
    if control < -50
        control = -50;
    end
   
    r.motor(3,round(control));
end

pause(0.1)
r.motor(3,0)

clf
hold on
plot(times,rpms)
yline(rpm_targ,'-','Target')
ylim([0 rpm_targ+50])
xlim ([0 runtime])
hold off