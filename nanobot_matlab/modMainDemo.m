%% Clean up
clear; clc; close all;
% Create an instance of the nanobot class
nb = nanobot('COM49', 115200, 'wifi');

%% main code
% Intialize all sensors and motors
tic;
duty = 9; % base speed of motors for methods with customizability
sf = 1.2; % scale factor for motor 2's power
initallSensors(nb);
botdirection =0;
walloffset =0;
color ='';
angleDeviation =30;
odoDist=20;
linecompletionflag =false;
wallcompletionflag = false;
odometrycompletionflag = false;
disp("Please center your robot's array on the start line ");
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

disp("Moving to center self on intersection...");
% [ll,l,lm,rm,r,rr]=getRCvalues(nb);
% while(~(allDark(ll,l,lm,rm,r,rr)))
%     [ll,l,lm,rm,r,rr]=getRCvalues(nb);
%     nb.setMotor(1, 9);
%     nb.setMotor(2, sf*9);
% end
lineFollowing(nb);
nb.setMotor(1, 0);
nb.setMotor(2, 0);
moveToBar(nb, sf);

for i=1:length(selected_sequence)
    currenttask = selected_sequence(i);
    if(currenttask == 1) % Line following
        if(botdirection == 0) % Start
            disp("Turn left till line detected...(middle 2 sensors black)");
            turnTillLine(nb, duty, sf, 'l'); % Turn left
            botdirection =1;
            disp("Following Line.........");
            lineFollowing(nb);
            
            disp("Detected all black returnflagline = true robot about turn 180 deg,botdirection=-1 and follow back line");
            botdirection =-1;
            linecompletionflag = true;
            disp("Following Line........ all black linecompletionflag = true ");
            turnTillLine(nb, duty, sf, 'l'); % Turn left at end
            lineFollowing(nb); % At end, we're facing wall
            moveToBar(nb, sf);
        end
        if(botdirection == 1) % Second action, came from wall following
            disp("GO Straight");
            goStraighttillline(nb);
            disp("Following Line.........");
            lineFollowing(nb);
            disp("Detected all black returnflagline = true robot about turn 180 deg,botdirection=-1 and foloow back line");
            botdirection =-1;
            linecompletionflag = true;
            disp("Following Line........ all black linecompletionflag = true ");
        end
    elseif(currenttask == 2) % Odometry
        if(botdirection ==-1) % Third action, came from line following
            disp("Turn Left till middle two sensors read black");
            turnLefttillline(nb);
            botdirection =0;
            disp("follow line till all black read, read color");
            lineFollowing(nb);
           
            disp("Perform odometry based on color")
            performOdometrybycolor(nb,angleDeviation,odoDist);
            odometrycompletionflag =true;
            
        end
        if(botdirection == 1) % Third action, came from wall following
            disp("Turn Right till middle two sensors read black");
            turnRighttillline(nb);
            botdirection =0;
            disp("follow line till all black read, read color");
            lineFollowing(nb);
            disp("Perform odometry based on color")
            performOdometrybycolor(nb,angleDeviation,odoDist);
            odometrycompletionflag =true;
        end

    else % Wall Following
         if(botdirection == 0) % Start
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

         if(botdirection == -1) % Second action, came from line follow
            disp("Go Straight...");
            disp("Following line till front sensor is < threshold.........");
            lineFollowing(nb); 
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
nb.setMotor(1, 0);
nb.setMotor(2, 0);

%%
function kickMotors(nb, duty, sf, dir)
    % Gives an extremely short high duty-cycle pulse to the motors to
    % overcome static friction of the gearbox, allowing operation of motors
    % at lower duty cycles.
    switch dir
        case 's'
            dir1 = 1;
            dir2 = 1;
        case 'l'
            dir1 = 1;
            dir2 = -1;
        case 'r'
            dir1 = -1;
            dir2 = 1;
        otherwise
            dir1 = 0;
            dir2 = 0;
    end
    nb.setMotor(1, dir1 * duty);
    nb.setMotor(2, dir2 * sf * duty);
    nb.setMotor(1, 0);
    nb.setMotor(2, 0);
end


function turnTillLine(nb, duty, sf, dir)
    % Turn in place till a line is detected, init reflectance prior
    rotDir = 1; % correspond to default left
    stopDir = 'r';
    if dir == 'r'
        rotDir = -1;
        stopDir = 'l';
    end

    kickMotors(nb, duty*1.25, sf, dir);
    tic;
    while(true)
        % % Stall checking
        % val1 = nb.encoderRead(1);
        % val2 = nb.encoderRead(2);
        % if((val1.countspersec == 0) || (val2.countspersec == 0))
        %     kickMotors(nb, duty*1.25, sf, dir);
        % end
        nb.setMotor(1, rotDir * duty);
        nb.setMotor(2, sf * rotDir * -duty);
        vals = nb.reflectanceRead();
        if(vals.one<300 && vals.two<300 && (vals.three>500 || vals.four>500) && vals.five<300 && vals.six<300)
            if(toc < 0.35)
                continue;
            end
            fprintf("Match!\n");
            kickMotors(nb, duty*1.5, sf, stopDir);
            attemptCenter(nb, sf);
            break;
        end
    end

    nb.setMotor(1, 0);
    nb.setMotor(2, 0);
end

function attemptCenter(nb, sf)
    vals = nb.reflectanceRead();
    error = -1*vals.one - 1*vals.two - 1*vals.three + 1*vals.four + 1*vals.five + 1*vals.six;
    while(abs(error) > 300)
        vals = nb.reflectanceRead();
        error = -3*vals.one - 2*vals.two - 1*vals.three + 1*vals.four + 2*vals.five + 3*vals.six;
        if error < 0
            kickMotors(nb, 10, sf, 'l');
        else
            kickMotors(nb, 10, sf, 'r');
        end
        pause(0.05);
    end
end

function moveToBar(nb, sf)
    % For use after an all-black condition is recognized, to move forward
    % to center the rotation about the line.
    nb.setMotor(1, 9);
    nb.setMotor(2, sf*9);
    pause(0.5); % Needs to be tuned
    nb.setMotor(1, 0);
    nb.setMotor(2, 0);
end

function turnOffLine(nb, sf, time, dir)
    % To get off the line so turnToLine doesn't immediately react
    rotDir = -1;
    if dir == 'l'
        rotDir = 1;
    end
    nb.setMotor(1, rotDir*9);
    nb.setMotor(2, rotDir*sf*-9);
    pause(time); % Needs to be tuned
    nb.setMotor(1, 0);
    nb.setMotor(2, 0);
end

function initallSensors(nb)
    % Initialize sensors and motors
    nb.initReflectance(); % IR Sensor
    nb.initColor(); % Color sensor
    nb.initUltrasonic1('D2','D3'); % front sensor
    nb.initUltrasonic2('D4','D5'); % left sensor
end

function [r,g,b]= getColorvalues(nb)
    %Take a single RGB color sensor reading
    values = nb.colorRead();
    
    %The sensor values are saved as fields in a structure:
    r = values.red;
    g = values.green;
    b = values.blue;
end

function [fu,lu] = getUSvalues(nb)
    fu=nb.ultrasonicRead1()*0.01715; % Read front distance
    lu=nb.ultrasonicRead2()*0.01715; % Read front distance
end

function [ll,l,lm,rm,r,rr] = getRCvalues(nb)
    % minReflectance  = [200,180,180,200]; %200,180,180,200
    minReflectance  = [125,100,90,90,100,135]; %200,180,180,200
    sensorVals = nb.reflectanceRead();
    % Calibrate sensor readings
    % ll = (sensorVals.one - minReflectance(1));
    % l = (sensorVals.two - minReflectance(2));
    % lm = (sensorVals.three - minReflectance(3));
    % rm = (sensorVals.four - minReflectance(4));
    % r = (sensorVals.five - minReflectance(5));
    % rr = (sensorVals.six - minReflectance(6));

    ll = (sensorVals.one);
    l = (sensorVals.two);
    lm = (sensorVals.three);
    rm = (sensorVals.four);
    r = (sensorVals.five);
    rr = (sensorVals.six);

end

function onBar = allDark(ll,l,lm,rm,r,rr)
    if (ll>300 && l>300 && lm>300 && rm>300 && r>300 && rr>300)
        onBar = true;
    else
        onBar = false;
    end
end

function lineFollowing(nb)

    % Note: initialize relflectance array before calling this function
    % vals = nb.reflectanceRead(); TESTING
    
    % Motor offset factor:
    mOffScale = 1.2;
    
    % TUNING:
    % Tip: when tuning kd, it must be the opposite sign of kp to damp
    kp = 0.0006; % Was 0.0006
    ki = 0.0;
    kd = -0.00015; % was -0.00015
    
    % Basic initialization
    vals = 0;
    prevError = 0;
    prevTime = 0;
    integral = 0;
    derivative = 0;
    
    % Determine a threshold to detect when white is present on all sensors
    whiteThresh = 200; % Max value detected for all white
    
    % The base duty cycle "speed" you wish to travel down the line with
    % (recommended values are 9 or 10)
    motor1BaseSpeed = 9;
    motor2BaseSpeed = 9;
    
    tic
    % It can be helpful to initialize your motors to a fixed higher duty cycle
    % for a very brief moment, just to overcome the gearbox force of static
    % friction so that lower duty cycles don't stall out at the start.
    % (recommendation: 10, with mOffScale if needed)
    nb.setMotor(1, 10);
    nb.setMotor(2, mOffScale*10);
    pause(0.03);
    while (true)  % Adjust me if you want to stop your line following earlier, or let it run longer.
    
        % % STATE CHECKING:
        % [ll,l,lm,rm,r,rr]=getRCvalues(nb);
        % [fu,~]=getUSvalues(nb);
        % if (allDark(ll,l,lm,rm,r,rr) || fu < 15)
        %     break;
        % end

        % TIME STEP
        dt = toc - prevTime;
        fprintf("%0.5f\n",dt);
        prevTime = toc;
    
        vals = nb.reflectanceRead();
        if(vals.one>300 && vals.two>300 && vals.three>300 && vals.four>300 && vals.five>300 && vals.six>300)
            % All black condition
            break;
        end

    
        % ///// UNCOMMENT IF YOU WANT TO USE CALIBRATED SENSOR VALUES
        % calibratedVals =   struct('one', 0, 'two', 0, 'three', 0, 'four', 0, 'five', 0, 'six', 0);
        % % Calibrate sensor readings
        % calibratedVals.one = (vals.one - minReflectance(1));
        % calibratedVals.two = (vals.two - minReflectance(2));
        % calibratedVals.three = (vals.three - minReflectance(3));
        % calibratedVals.four = (vals.four - minReflectance(4));
        % calibratedVals.five = (vals.five - minReflectance(5));
        % calibratedVals.six = (vals.six - minReflectance(6));
        % error = -4*calibratedVals.one - 2*calibratedVals.two - 1*calibratedVals.three + 1*calibratedVals.four + 2*calibratedVals.five + 4*calibratedVals.six;
        
    
        % ///// COMMENT OUT LINE BELOW IF USING CALIBRATED SENSOR VALUES
        error = -3*vals.one - 2*vals.two - 1*vals.three + 1*vals.four + 2*vals.five + 3*vals.six;
    
        % Calculate P, I, and D terms
        integral = integral + error * dt;
    
        derivative = (prevError - error)/dt;
    
        % Set PID
        control = kp*error + ki*integral + kd*derivative;
    
        % STATE CHECKING - stops robot if all sensors read white (lost tracking):
        if (vals.one < whiteThresh && ...
                vals.two < whiteThresh && ...
                vals.three < whiteThresh && ...
                vals.four < whiteThresh && ...
                vals.five < whiteThresh && ...
                vals.six < whiteThresh)
            % Stop the motors and exit the while loop
            nb.setMotor(1, 0);
            nb.setMotor(2, 0);
            break;
        else
            % LINE DETECTED:
            m1Duty = motor1BaseSpeed - control;
            m2Duty = motor2BaseSpeed + mOffScale * control;
    
            nb.setMotor(1, m1Duty);
            nb.setMotor(2, m2Duty);
        end
    
        prevError = error;
    end
    nb.setMotor(1, 0);
    nb.setMotor(2, 0);

end

function turnByAngle(nb, deg)
    % A function that turns the robot by a specific number of degrees.
    % Positive corresponds to counterclockwise (left) rotation

    wheelDiameter = 0.07; % Diameter of the wheel in meters
    wheelBase = 0.1425; % Distance between the wheels in meters
    encoderCountsPerRevolution = 1440; % Encoder counts per wheel revolution
    wheelCircumference = pi * wheelDiameter;

    if deg > 0
        deg = 0.95 * deg;
    end
    
    arcLen = pi * wheelBase * (deg / 360); % Distance the wheels need to travel

    numRevs = -arcLen / wheelCircumference;

    encTarget = round(numRevs * encoderCountsPerRevolution);
    encCount = 0; % No progress made

    % Motor offset factor:
    mOffScale = 1.2;

    kp = 0.006;
    ki = 0;
    kd = -0.00015;

    vals = 0;
    prevError = 0;
    prevTime = 0;
    integral = 0;
    derivative = 0;
    
    val2 = nb.encoderRead(2);
    tic
    while (true)

        % TIME STEP
        dt = toc - prevTime;
        prevTime = toc;
    
        val2 = nb.encoderRead(2);
        encCount = encCount + (val2.counts);
        fprintf("encTarget: %0.2f, encCounts: %0.2f\n", encTarget, encCount);
              
    
        error = encTarget - encCount;
        fprintf("error: %0.2f\n", error);

        if(abs(error) < 25)
            break;
        end
    
        % Calculate P, I, and D terms
        integral = integral + error * dt;
    
        derivative = (prevError - error)/dt;
    
        % Set PID
        control = kp*error + ki*integral + kd*derivative;

        if control < 0
            if control > -9
                control = -9;
            elseif control < -14
                control = -14;
            end
        else
            if(control > 14)
                control = 14;
            elseif(control < 9)
                control = 9;
            end
        end

        % LINE DETECTED:
        m1Duty = -control;
        m2Duty = mOffScale * control;
        
        nb.setMotor(1, m1Duty);
        nb.setMotor(2, m2Duty);
        
        
        prevError = error;
        end
    nb.setMotor(1, 0);
    nb.setMotor(2, 0);

end

function wallFollowing(nb,botdirection,walloffset)
    Kp = 1.0; % Proportional gain - adjust based on testing
    Ki = 0.000; % Integral gain - adjust based on testing
    Kd = 0.000; % Derivative gain - adjust based on testing
    prevError = 0;
    prevTime = 0;
    integral = 0;
    dt = 0.1;
    task =3;

    times = 0;
    dist = 0;
    errors = 0;
    tic
    while true
        dt = toc - prevTime;
        fprintf("%0.4f\n", dt);
        prevTime = toc;
        times(end+1) = prevTime;

        [ll,l,lm,rm,r,rr]=getRCvalues(nb);
        [~,leftDist]=getUSvalues(nb);
        dist(end+1) = leftDist;
        if (botdirection ==-1 && allDark(ll,l,lm,rm,r,rr))
            setMotorstozero(nb);
            turnRighttillline(nb);
            botdirection =1;
            lineFollowing(nb,task,botdirection)
            break;
        end
        
        error= walloffset - leftDist;
        errors(end+1) = error;

        if(leftDist>15)
            setMotorstozero(nb);
            break;
        end
        integral = integral + error * dt;
        derivative = (error - prevError) / dt;

        % Calculate control signal
        controlSignal = Kp * error + Ki * integral + Kd * derivative;

        % Adjust motor speeds based on control signal
        baseSpeed = 10; % Base speed for both motors
        speedM1 = baseSpeed - controlSignal; % Adjust right motor speed
        speedM2 = 1.2*(baseSpeed + controlSignal); % Adjust left motor speed

        % Apply the speeds to the motors
        % if (speedM1>0 && speedM1>10)
        % speedM1 = 10;
        % end
        % if (speedM2>0 && speedM2>10)
        % speedM2 = 10;
        % end
        % if (speedM1<0 && speedM1<-10)
        % speedM1 = -10;
        % end
        % if (speedM2<0 && speedM2<-10)
        % speedM2 = -10;
        % end
        if controlSignal < 0
            if speedM1 > 12
                speedM1 = 12;
            end
            if speedM2 < 9
                speedM2 = 9;
            end
        else
            if speedM2 > 12
                speedM2 = 12;
            end
            if speedM1 < 9
                speedM1 = 9;
            end
        end

        if((error < prevError) && (speedM1 > speedM2))
            temp = speedM1;
            speedM1 = speedM2;
            speedM2 = temp;
        end
        nb.setMotor(1, speedM1); % Set right motor speed
        nb.setMotor(2, speedM2);% Set left motor speed
        % Update previous error
        prevError = error;
        %pause(dt);
    end
    % wall Following Code include the below condition
                % if (l>300 && lm>300 && rm>300 && r>300)
                %     setMotorstozero(nb);
                 %    turnRighttillline(nb);
                % end

    clf
    hold on
    plot(times,errors)
    % yline(walloffset,'-','Target')
    % ylim([0, walloffset+5])
    hold off
end

function setMotorstozero(nb)
    nb.setMotor(2,0);
    nb.setMotor(1,0);
end