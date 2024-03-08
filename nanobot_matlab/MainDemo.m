%% Clean up
clear; clc; close all;
tic;
% Create an instance of the nanobot class
nb = nanobot('COM31', 115200, 'serial');

%% main code
% Intialize all sensors and motors
initallSensors(nb);
botdirection =0;
walloffset =0;
color ='';
angleDeviation =30;
odoDist=20;
linecompletionflag =false;
wallcompletionflag = false;
odometrycompletionflag = false;
disp("Choose a Sequence...");
disp("1. Line following");
disp("2. Odometry");
disp("3. Wall Following");
available_Sequences = containers.Map;
available_Sequences("A") = [1, 3, 2];
available_Sequences("B") = [3, 1, 2];

disp("Sequence Dictionaries:");
disp("A: " + num2str(available_Sequences("A")));
disp("B: " + num2str(available_Sequences("B")));

disp("Choose sequence id A or B:");
chosen_sequence = input("Enter sequence ID (A or B): ", 's');

if isKey(available_Sequences, chosen_sequence)
    selected_sequence = available_Sequences(chosen_sequence);
    disp("You have chosen Sequence " + chosen_sequence);
    disp("Sequence Steps: " + num2str(selected_sequence));
    
    % Add code to execute the chosen sequence based on selected_sequence
    
else
    disp("Invalid sequence ID. Please choose A or B.");
end

for i=1:length(selected_sequence)
    currenttask = selected_sequence(i);
    if(currenttask == 1)
        if(botdirection == 0)
            disp("Turn left till line detected...(middle 2 sensors black)");
            turnLefttillline(nb);
            botdirection =1;
            disp("Following Line.........");
            lineFollowing(nb,currenttask,botdirection);
            
            disp("Detected all black returnflagline = true robot about turn 180 deg,botdirection=-1 and foloow back line");
            botdirection =-1;
            linecompletionflag = true;
            disp("Following Line........ all black linecompletionflag = true ");
        end
        if(botdirection == 1)
            disp("GO Straight");
            goStraighttillline(nb);
            disp("Following Line.........");
            lineFollowing(nb,currenttask,botdirection);
            disp("Detected all black returnflagline = true robot about turn 180 deg,botdirection=-1 and foloow back line");
            botdirection =-1;
            linecompletionflag = true;
            disp("Following Line........ all black linecompletionflag = true ");
        end
    elseif(currenttask == 2)
        if(botdirection ==-1)
            disp("Turn Left till middle two sensors read black");
            turnLefttillline(nb);
            botdirection =0;
            disp("follow line till all black read, read color");
            lineFollowing(nb,currenttask,botdirection);
           
            disp("Perform odometry based on color")
            performOdometrybycolor(nb,angleDeviation,odoDist);
            odometrycompletionflag =true;
            
        end
        if(botdirection == 1)
            disp("Turn Right till middle two sensors read black");
            turnRighttillline(nb);
            botdirection =0;
            disp("follow line till all black read, read color");
            lineFollowing(nb,currenttask,botdirection);
            disp("Perform odometry based on color")
            performOdometrybycolor(nb,angleDeviation,odoDist);
            odometrycompletionflag =true;
        end

    else
         if(botdirection == 0)
            disp("Turn Right till line detected...(middle 2 sensors black)");
            turnRighttillline(nb);
            disp("Following line till front sensor is < threshold.........");
            botdirection = -1;
            lineFollowing(nb,currenttask,botdirection);
            disp("Turn Right and follow wall");
            turnrighttillcurvediverges(nb);
            [~,walloffset] = getUSvalues(nb);
            disp("Following wall........ all black wallcompletionflag = true, Turn right till middle two sensors black ");
            wallFollowing(nb,botdirection,walloffset);
            botdirection = 1;
            wallcompletionflag = true;
            disp("follow line ... all black wallcompleteflag = true" );

         end

         if(botdirection == -1)
            disp("Go Straight...");
            goStraighttillline(nb);
            disp("Following line till front sensor is < threshold.........");
            lineFollowing(nb,currenttask,botdirection);
            disp("Turn Right and follow wall with left offset distance of 7cm");
            turnrighttillcurvediverges(nb);
            [~,walloffset] = getUSvalues(nb);
            disp("Following wall........ all black wallcompletionflag = true, Turn right till middle two sensors black ");
            wallFollowing(nb,botdirection,walloffset);

            botdirection = 1;
            wallcompletionflag=true;
            disp("follow line ... all black wallcompleteflag = true" );
        end
        
    end
    if(linecompletionflag && wallcompletionflag && odometrycompletionflag)
        disp("Task Done!!!!!");
        fprintf("Elapsed Time is: %0.2f", toc);

    end

end
%%
function turnrighttillcurvediverges(nb)
    prevlu=0;
            while true
                nb.setMotor(2,10);
                nb.setMotor(1,0);
                [fu,lu]=getUSvalues(nb);
                if(fu>lu && lu<10 && prevlu<lu)
                    fprintf("FU: %0.2f, LU: %0.2f",fu,lu);
                    setMotorstozero(nb);
                    break;
                end
                prevlu =lu;
            end
end

function turnLefttillline(nb)
tic;
            while true
                setMotorstoleft(nb);
                if (checkonLine(nb) && toc>0.4)
                    setMotorstozero(nb);
                    break;
                end
            end
end

function turnRighttillline(nb)
tic;
            while true
                setMotorstoright(nb);
                if (checkonLine(nb) && toc>0.4)
                    setMotorstozero(nb);
                    break;
                end
            end
end

function goStraighttillline(nb)
tic;
            while true
                setMotorstostraight(nb);
                if (toc>0.1)
                    setMotorstozero(nb);
                    break;
                end
            end
end

function [l,lm,rm,r] = getIRvalues(nb)
    minReflectance  = [200,180,180,200];
    sensorVals = nb.reflectanceRead();
    % Calibrate sensor readings
    l = (sensorVals.one - minReflectance(1));
    lm = (sensorVals.two - minReflectance(2));
    rm = (sensorVals.three - minReflectance(3));
    r = (sensorVals.four - minReflectance(4));
end

function [fu,lu] = getUSvalues(nb)
    fu=nb.ultrasonicRead1()*0.01715; % Read front distance
    lu=nb.ultrasonicRead2()*0.01715; % Read front distance
end

function [r,g,b]= getColorvalues(nb)
    %Take a single RGB color sensor reading
    values = nb.colorRead();
    
    %The sensor values are saved as fields in a structure:
    r = values.red;
    g = values.green;
    b = values.blue;
end

function setMotorstostraight(nb)
    nb.setMotor(2,10);
    nb.setMotor(1,10);
end

function setMotorstoleft(nb)
    nb.setMotor(2,0);
    nb.setMotor(1,10);
end


function setMotorstoright(nb)
    nb.setMotor(2,10);
    nb.setMotor(1,0);
end


function setMotorstozero(nb)
    nb.setMotor(2,0);
    nb.setMotor(1,0);
end


function aboutTurn(nb)
    nb.setMotor(2,-10);
    nb.setMotor(1,9);
    while true
        if (checkonLine(nb))
            setMotorstozero(nb);
            break;
        end
    end
end


function initallSensors(nb)
    % Initialize sensors and motors
    nb.initReflectance(); % IR Sensor
    nb.initColor(); % Color sensor
    nb.initUltrasonic1('D2','D3'); % front sensor
    nb.initUltrasonic2('D4','D5'); % left sensor
end
function onLine = checkonLine(nb)
    [l,lm,rm,r] = getIRvalues(nb);
    disp("Check for on Line");
    if(l<300 && lm>300 && rm>300 && r<300)
        onLine = true;
    else
        onLine = false;
    end
end

function onBar = allDark(l,lm,rm,r)
if (l>300 && lm>300 && rm>300 && r>300)
    onBar = true;
else
    onBar = false;
end
end

function wallFollowing(nb,botdirection,walloffset)
Kp = 0.5; % Proportional gain - adjust based on testing
Ki = 0.0001; % Integral gain - adjust based on testing
Kd = 0.0005; % Derivative gain - adjust based on testing
 prevError = 0;
 integral = 0;
 dt = 0.1;
 task =3;
 while true
     tic;
      [l,lm,rm,r]=getIRvalues(nb);
     [~,leftDist]=getUSvalues(nb);
    if(leftDist>10)

        setMotorstozero(nb);
    end
    if (botdirection ==-1 && allDark(l,lm,rm,r))
        setMotorstozero(nb);
        turnRighttillline(nb);
        botdirection =1;
        lineFollowing(nb,task,botdirection)
        break;
    end
    error= leftDist - walloffset;
    integral = integral + error * dt;
    derivative = (error - prevError) / dt;

    % Calculate control signal
    controlSignal = Kp * error + Ki * integral + Kd * derivative;

    % Update previous error
    prevError = error;
        % Adjust motor speeds based on control signal
     baseSpeed = 10; % Base speed for both motors
    speedM1 = baseSpeed + controlSignal; % Adjust right motor speed
    speedM2 = baseSpeed - controlSignal; % Adjust left motor speed

    % Apply the speeds to the motors
    if (speedM1>0 && speedM1>10)
    speedM1 = 10;
    end
    if (speedM2>0 && speedM2>10)
    speedM2 = 10;
    end
    if (speedM1<0 && speedM1<-10)
    speedM1 = -10;
    end
    if (speedM2<0 && speedM2<-10)
    speedM2 = -10;
    end
    nb.setMotor(1, speedM1); % Set right motor speed
    nb.setMotor(2, speedM2);% Set left motor speed
    %pause(dt);
    dt=toc;
 end
% wall Following Code include the below condition
                % if (l>300 && lm>300 && rm>300 && r>300)
                %     setMotorstozero(nb);
                 %    turnRighttillline(nb);
                % end
    
end
function lineFollowing(nb,task,botdirection)
   % minReflectance  = [145,145,145,145]; % Replace these with actual minimum values observed during calibration
% maxReflectance  = [1225,1225,1225,1225]; % Replace these with actual maximum values observed during calibration
kp=1.8;
kd=0.001;
ki=0;
basespeed=10;
%Initialize the reflectance sensor with default pins D12, D11, D10, D8

di=[0, 1.1, 2.2, 3.3];
d0=1.65;
dt=0.1;
prevError = 0;
integral = 0;

  
%Take a single reflectance sensor reading
while true
    tic;
    [l,lm,rm,r]=getIRvalues(nb);
    [fu,~]=getUSvalues(nb);
    if (task ==1 && botdirection ==1 && allDark(l,lm,rm,r))
        aboutTurn(nb);
        botdirection = -1;
        continue;
    end
    if (task ==1 && botdirection ~=1 && allDark(l,lm,rm,r))
        setMotorstozero(nb);
        
        break;
    end
    if(task==3 && botdirection ==1 && allDark(l,lm,rm,r))
        setMotorstozero(nb);
        break;

    end
    if(task==3 && botdirection ==-1 && fu <15)
        setMotorstozero(nb);
        break;
    end
    if(allDark(l,lm,rm,r) && botdirection==0)
        setMotorstozero(nb);
        break;
    end

    d = (l * di(1) + lm * di(2) + rm * di(3) + r * di(4)) / (l+lm+rm+r);
    error= d0-d;
    
    integral = integral + error * dt;
    derivative = (error - prevError) / dt;
    Lmspeed=basespeed - (kp*error+ki*integral+kd*derivative);
    Rmspeed=basespeed + (kp*error+ki*integral+kd*derivative);
    %nb.setMotors(Lmspeed,Rmspeed);
    prevError = error;
    fprintf('one: %.2f, two: %.2f, three: %.2f four: %.2f, error: %.2f, LS: %0.2f, RS: %0.2f \n', l, lm, rm, r,error,Lmspeed,Rmspeed);
    if(Rmspeed>basespeed+5)
        Rmspeed=basespeed+5;
    end
    if(Rmspeed<-(basespeed+5))
        Rmspeed=-(basespeed+5);
    end
    if(Lmspeed>basespeed+5)
        Lmspeed=basespeed+5;
    end
    if(Lmspeed<-(basespeed+5))
        Lmspeed=-(basespeed+5);
    end
    nb.setMotor(1,(Rmspeed-1));
    nb.setMotor(2, Lmspeed);
    dt=toc;
end

% Line Following Code include the below condition
                % if (l>300 && lm>300 && rm>300 && r>300)
                %     setMotorstozero(nb);
                %     aboutTurn(nb);
                % end
end

function performOdometrybycolor(nb,angleDeviation,odoDist)
    disp("Inside Odometry");

     [r,g,b]= getColorvalues(nb);
            if(r>g &&r>b)
                disp("Color is Red");
                disp("Color is red turn Left and travel");
                performOdometry(nb,(-angleDeviation),odoDist);
            end
            if(b>g &&b>r)
                disp("Color is Blue");
                performOdometry(nb,0,odoDist);
            end
            if(g>r &&g>b)
                disp("Color is Green");
                performOdometry(nb,angleDeviation,odoDist);
            end

end

function performOdometry(nb,angleDeviation,odoDist)
    disp("Inside Odometry");
    %Odometry code
    wheelDiameter = 0.07; % Diameter of the wheel in meters
    wheelBase = 0.14; % Distance between the wheels in meters
    encoderCountsPerRevolution = 1440; % Encoder counts per wheel revolution
    wheelCircumference = pi * wheelDiameter;
    fraction = abs(angleDeviation)/90;
    requiredCounts = encoderCountsPerRevolution*fraction;
    Kp = 0.01; % Proportional gain
    Ki = 0.0; % Integral gain
    Kd = 0.003; % Derivative gain
    b=13;
    prevError = 0;
    integral = 0;
    dt = 0.1; % Time step in seconds
    totalLeftEncoderCounts = nb.encoderRead(2).counts;
    totalLeftEncoderCounts=0;
    totalRightEncoderCounts = nb.encoderRead(1).counts;
    totalRightEncoderCounts=0;

    if(angleDeviation>0)
        disp("Color is green turn rigght and travel");
        while totalLeftEncoderCounts<requiredCounts
            tic;
            deltaLeftEncoderCounts = nb.encoderRead(2).counts;
            totalLeftEncoderCounts = totalLeftEncoderCounts + deltaLeftEncoderCounts;
            error = requiredCounts -totalLeftEncoderCounts;
            integral = integral + error * dt;
            derivative = (error - prevError) / dt;
            controlSignal = Kp * error + Ki * integral + Kd * derivative;
            prevError = error;
            if(error<1)
                nb.setMotor(2,0);
            break;
            end
            if(controlSignal>b)
                controlSignal=b;
            end
            if (controlSignal < 10)
                controlSignal=10;
            end
            fprintf('Total Left Enc counts: %0.2f === Control Signal: %0.2f \n', totalLeftEncoderCounts, controlSignal);
            nb.setMotor(2, controlSignal);
            dt=toc;
         end



    elseif(angleDeviation ==0)
        disp("Color is Blue straight travel");
    else
        disp("Color is red turn Left and travel");
        while totalRightEncoderCounts<requiredCounts
            tic;
            deltaRightEncoderCounts = -nb.encoderRead(1).counts;
            totalRightEncoderCounts = totalRightEncoderCounts + deltaRightEncoderCounts;
            error = requiredCounts -totalRightEncoderCounts;
            integral = integral + error * dt;
            derivative = (error - prevError) / dt;
            controlSignal = Kp * error + Ki * integral + Kd * derivative;
            prevError = error;
            if(error<1)
                nb.setMotor(1,0);
            break;
            end
            if(controlSignal>b)
                controlSignal=b;
            end
            if (controlSignal < 10)
                controlSignal=10;
            end
            fprintf('Total Right Enc counts: %0.2f === Control Signal: %0.2f \n', totalRightEncoderCounts, controlSignal);
            nb.setMotor(1, controlSignal);
            dt=toc;
        end
    end
end