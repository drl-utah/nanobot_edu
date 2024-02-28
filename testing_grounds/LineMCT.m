clc
clear all

% Create an instance of the nanobot class
nb = nanobot('COM50', 115200, 'wifi');

%%
nb.initReflectance();

% REFLECTANCE
minReflectance  = [103,95,86,112]; % Replace these with actual minimum values observed during calibration
% maxReflectance  = [1225,1225,1225,1225]; % Replace these with actual maximum values observed during calibration
kp=1.5;
kd=0.0;
ki=0;
basespeed=10;
maxspeed=20;
%Initialize the reflectance sensor with default pins D12, D11, D10, D8

di=[0, 1.1, 2.2, 3.3];
d0=1.65;
dt=0.1;
prevError = 0;
prevTime = 0;
integral = 0;
%Take a single reflectance sensor reading
tic
pause(0.03);
while true
    dt = toc - prevTime;
    prevTime = toc;
    sensorVals = nb.reflectanceRead();
    calibratedVals =   struct('one', 0, 'two', 0, 'three', 0, 'four', 0);
    % Calibrate sensor readings
    calibratedVals.one = (sensorVals.one - minReflectance(1));
    calibratedVals.two = (sensorVals.two - minReflectance(2));
    calibratedVals.three = (sensorVals.three - minReflectance(3));
    calibratedVals.four = (sensorVals.four - minReflectance(4));
    d = (calibratedVals.one * di(1) + calibratedVals.two * di(2) + calibratedVals.three * di(3) + calibratedVals.four * di(4)) / (calibratedVals.one+calibratedVals.two+calibratedVals.three+calibratedVals.four);
    if (calibratedVals.one == 0 && calibratedVals.two == 0 && calibratedVals.three == 0 && calibratedVals.four == 0)
        d = 0;
    end
    error= d0-d;
    
    integral = integral + error * dt;
    derivative = (error - prevError) / dt;
    Lmspeed=basespeed - (kp*error+ki*integral+kd*derivative);
    Rmspeed=basespeed + (kp*error+ki*integral+kd*derivative);
    %nb.setMotors(Lmspeed,Rmspeed);
    previouserror = error;
    fprintf('one: %.2f, two: %.2f, three: %.2f four: %.2f, error: %.2f, LS: %0.2f, RS: %0.2f \n', calibratedVals.one, calibratedVals.two, calibratedVals.three, calibratedVals.four,error,Lmspeed,Rmspeed);
    if(Rmspeed>maxspeed)
        Rmspeed=maxspeed;
    end
    if(Rmspeed<(-1*maxspeed))
        Rmspeed=-(maxspeed);
    end
    if(Lmspeed>maxspeed)
        Lmspeed=maxspeed;
    end
    if(Lmspeed<-(maxspeed))
        Lmspeed=-(maxspeed);
    end
    if(Lmspeed>0 && Lmspeed<8)
        Lmspeed =8;
    end
    if(Lmspeed<0 && Lmspeed>-8)
        Lmspeed =-8;
    end
   
    % STATE CHECKING:
    if (sensorVals.one < 200 & ...
            sensorVals.two < 200 & ...
            sensorVals.three < 200 & ...
            sensorVals.four < 200)
        % ALL WHITE CONDITION
        nb.setMotor(1, 0);
        nb.setMotor(2, 0);
        break;
    elseif (sensorVals.one > 1000 & ...
            sensorVals.two > 1000 & ...
            sensorVals.three > 1000 & ...
            sensorVals.four > 1000)
        % ALL BLACK CONDITION
        nb.setMotor(1, 0);
        nb.setMotor(2, 0);
        break;
    else

        nb.setMotor(1,Rmspeed);
        nb.setMotor(2, 1.2 * Lmspeed);
    end

end

%%
nb.setMotors(0,0);