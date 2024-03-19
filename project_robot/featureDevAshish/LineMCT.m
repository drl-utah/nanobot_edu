clc
clear all

% Create an instance of the nanobot class
nb = nanobot('COM32', 115200, 'serial');
nb.initReflectance();

% REFLECTANCE
minReflectance  = [300,230,200,200,230,330]; % Replace these with actual minimum values observed during calibration
% maxReflectance  = [1225,1225,1225,1225]; % Replace these with actual maximum values observed during calibration
kp=1.4;
kd=-0.003;
ki=0.000001;
basespeed=9;
%Initialize the reflectance sensor with default pins D12, D11, D10, D8

di=[0, 1.4, 2.5, 3.6, 4.7, 6.1];
d0=3.05;
dt=0.1;
prevError = 0;
integral = 0;
calibratedVals =   struct('one', 0, 'two', 0, 'three', 0, 'four', 0,'five',0,'six',0);
    calibratedVals.one = 0;
    calibratedVals.two = 0;
    calibratedVals.three = 0;
    calibratedVals.four =0;
    calibratedVals.five =0;
    calibratedVals.six =0;
%Take a single reflectance sensor reading
while ~(calibratedVals.one>300 && calibratedVals.two>300 && calibratedVals.three>300 && calibratedVals.four>300 && calibratedVals.five>300 && calibratedVals.six>30)
    tic;
    sensorVals = nb.reflectanceRead();
    
    % Calibrate sensor readings
    calibratedVals.one = (sensorVals.one - minReflectance(1));
    calibratedVals.two = (sensorVals.two - minReflectance(2));
    calibratedVals.three = (sensorVals.three - minReflectance(3));
    calibratedVals.four = (sensorVals.four - minReflectance(4));
    calibratedVals.five = (sensorVals.five - minReflectance(5));
    calibratedVals.six = (sensorVals.six - minReflectance(6));
    d = (calibratedVals.one * di(1) + calibratedVals.two * di(2) + calibratedVals.three * di(3) + calibratedVals.four * di(4) + calibratedVals.five * di(5) + calibratedVals.six * di(6))/ (calibratedVals.one+calibratedVals.two+calibratedVals.three+calibratedVals.four+calibratedVals.five+calibratedVals.six);
    error= d0-d;
    
    integral = integral + error * dt;
    derivative = (error - prevError) / dt;
    Lmspeed=basespeed - (kp*error+ki*integral+kd*derivative);
    Rmspeed=basespeed + (kp*error+ki*integral+kd*derivative);
    %nb.setMotors(Lmspeed,Rmspeed);
    previouserror = error;
    fprintf('one: %.2f, two: %.2f, three: %.2f four: %.2f, five: %.2f, six: %.2f, error: %.2f, LS: %.2f, RS: %.2f \n', calibratedVals.one, calibratedVals.two, calibratedVals.three, calibratedVals.four,calibratedVals.five,calibratedVals.six,error,Lmspeed,Rmspeed);
    % if(Rmspeed>basespeed+5)
    %     Rmspeed=basespeed+5;
    % end
    % if(Rmspeed<-(basespeed+5))
    %     Rmspeed=-(basespeed+5);
    % end
    % if(Lmspeed>basespeed+5)
    %     Lmspeed=basespeed+5;
    % end
    % if(Lmspeed<-(basespeed+5))
    %     Lmspeed=-(basespeed+5);
    % end
    % if(Lmspeed>0 && Lmspeed<8)
    %     Lmspeed =8;
    % end
    % if(Lmspeed<0 && Lmspeed>-8)
    %     Lmspeed =-8;
    % end
   
    nb.setMotor(1,Rmspeed);
    nb.setMotor(2, Lmspeed);
    dt=toc;
end
nb.setMotors(0,0);