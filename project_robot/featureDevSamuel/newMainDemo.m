%% Clean up
clear; clc; close all;
% Create an instance of the nanobot class
nb = nanobot('COM49', 115200, 'wifi');

%% Main Program

% Program config variables
scaleFactor = 1.1;

% Initialize sensors
initallSensors(nb);

while(1)
    action = getInputAndClassify();

    switch action
        case 'LLine'
            turnTillLine(nb, 7, scaleFactor, 'l');
        case 'RLine'
            turnTillLine(nb, 7, scaleFactor, 'r');
        case 'atmCtr'
            attemptCenter(nb, scaleFactor);
        case 'm2Bar'
            moveToBar(nb, scaleFactor);
        case 'lineFollow'
            lineFollowing(nb, scaleFactor);
        case 'wallFollow'
            % Do something
            % Approach forward with US reads until below threshold
            % Turn right till curve diverges (no left due to asymmetry)
            % Read wall offset
            % Wall follow with wall offset
            % Read all black
            % Either advance and turn-in-place by 90 deg, or use one
            %   wheel
        case 'odometry'
            % Perform odometry by color
            % Implement return functionality
        case 'end'
            break;
        otherwise
            fprintf("Input not recognized, try again...\n")
            continue;
    end
end

%%
nb.setMotor(1, 10);
nb.setMotor(2, 1.1 * 10);

%%
nb.setMotor(1, 0);
nb.setMotor(2, 0);

%%

function [output] = getInputAndClassify()
    output = input(['Enter the action you would like to perform:\n' ...
        'LLine, RLine, atmCtr, m2Bar, lineFollow, wallFollow, odometry, end\n'], "s"); % str for now
end

function hyperPWM(nb, hpwm, T, sf, dir)
    % Turn in place till a line is detected, init reflectance prior
    rotDir = 1; % correspond to default left
    if dir == 'r'
        rotDir = -1;
    end
    nb.setMotor(1, rotDir * hpwm);
    nb.setMotor(2, sf * rotDir * -hpwm);
    pause(T);
end

function [numTurns, dir] = getNumTurnsTill(prevNode, targetNode)
    % Returns number of times turntillline must be called to align the
    % robot from the prevNode path to the targetNode path, along with the
    % direction of least turns
    % prevNode - Node object representing where you came from
    % targetNode - Node object that is a recursive left or right sibling of
    %           prevNode

    if(prevNode ~= 0)
        % Init counters
        lcounter = 1; % Start at 1 since we're gonna turn anyway
        rcounter = 1; % ^
    
        searchNode = prevNode;
    
        % Check left turn first
        while(searchNode.Lsib ~= targetNode)
            searchNode = searchNode.Lsib;
            lcounter = lcounter + 1;
        end
    
        % Check right turn
        while(searchNode.Rsib ~= targetNode)
            searchNode = searchNode.Rsib;
            rcounter = rcounter + 1;
        end
    
        % Check size for optimal direction return
        if(lcounter < rcounter)
            numTurns = lcounter;
            dir = 'l';
        else
            numTurns = rcounter;
            dir = 'r';
        end
    else
        % Prev node was 0, set numTurns to 0 and dir to 's'
        numTurns = 0;
        dir = 's';
    end
end

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

    kickMotors(nb, duty*2, sf, dir);
    prevTime = 0;
    tic;
    while(true)
        % % Stall checking
        % val1 = nb.encoderRead(1);
        % val2 = nb.encoderRead(2);
        % fprintf("e1: %d e2: %d\n", val1.countspersec, val2.countspersec);
        % if((val1.countspersec == 0) || (val2.countspersec == 0))
        %     kickMotors(nb, duty*1.25, sf, dir);
        % end
        % dt = toc - prevTime;
        % prevTime = toc;
        % fprintf("%0.3f\n", dt);
        if(rotDir == 1)
            nb.setMotor(1, 1.2 * rotDir * duty);
            nb.setMotor(2, 1.1 * rotDir * -duty);
        else
            nb.setMotor(1, 1.2 * rotDir * duty);
            nb.setMotor(2, rotDir * -duty);
        end
        vals = nb.reflectanceRead();
        %fprintf('one: %.2f, two: %.2f, three: %.2f four: %.2f five: %.2f six: %.2f\n', vals.one, vals.two, vals.three, vals.four, vals.five, vals.six);

        if(vals.one<300 && (vals.two>500 || vals.three>500 || vals.four>500 || vals.five>500) && vals.six<300)
            if(toc < 1)
                hyperPWM(nb, 9, 0.01, sf, dir);
                continue;
            end
            fprintf("Match!\n");
            kickMotors(nb, duty*1.25, sf, stopDir);
            attemptCenter(nb, sf);
            break;
        end
        hyperPWM(nb, 9, 0.01, sf, dir); % 0.02 additional seconds of 15% duty cycle
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
    pause(0.7); % Needs to be tuned
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

function lineFollowing(nb, sf)

    % Note: initialize relflectance array before calling this function
    % vals = nb.reflectanceRead(); TESTING
    
    % Motor offset factor:
    mOffScale = sf;
    
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
    whiteThresh = 300; % Max value detected for all white
    blackThresh = 500; % Min value detected for all black
    
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
        prevTime = toc;
    
        vals = nb.reflectanceRead();
        fprintf('one: %.2f, two: %.2f, three: %.2f four: %.2f five: %.2f six: %.2f\n', vals.one, vals.two, vals.three, vals.four, vals.five, vals.six);

        blackCount = 0;
        if(vals.one > blackThresh)
            blackCount = blackCount + 1;
        end
        if(vals.two > blackThresh)
            blackCount = blackCount + 1;
        end
        if(vals.three > blackThresh)
            blackCount = blackCount + 1;
        end
        if(vals.four > blackThresh)
            blackCount = blackCount + 1;
        end
        if(vals.five > blackThresh)
            blackCount = blackCount + 1;
        end
        if(vals.six > blackThresh)
            blackCount = blackCount + 1;
        end

        if(vals.one>blackThresh && vals.two>blackThresh && vals.three>blackThresh && vals.four>blackThresh && vals.five>blackThresh && vals.six>blackThresh)
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
            fprintf("Here...\n");
            break;
        else
            % LINE DETECTED:
            m1Duty = motor1BaseSpeed - control;
            m2Duty = motor2BaseSpeed + mOffScale * control;
    
            if(blackCount >= 3)
                nb.setMotor(1, m1Duty * 0.7); % Slow down, may be approaching black line
                nb.setMotor(2, m2Duty * 0.7);
            else
                nb.setMotor(1, m1Duty);
                nb.setMotor(2, m2Duty);
            end
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
Kp = 0.5; % Proportional gain - adjust based on testing
Ki = 0.0001; % Integral gain - adjust based on testing
Kd = 0.0005; % Derivative gain - adjust based on testing
 prevError = 0;
 integral = 0;
 dt = 0.1;
 task =3;
 while true
     tic;
      [ll,l,lm,rm,r,rr]=getIRvalues(nb);
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

function performOdometrybycolor(nb,angleDeviation,odoDist)
    disp("Inside Odometry");
    
    [r,g,b]= getColorvalues(nb);
    if(r>g &&r>b)
        color="r";
        disp("Color is Red");
        disp("Color is red turn Left and travel");
        performOdometry(nb,(-angleDeviation),odoDist);
        goStraightbalanced(nb,color);
        
    end
    if(g>b &&g>r)
        color="g";
        disp("Color is Green");
        performOdometry(nb,0,odoDist);
        goStraightbalanced(nb,color);
    end
    if(b>r &&b>g)
        color="b";
        disp("Color is Blue");
        performOdometry(nb,angleDeviation,odoDist);
        goStraightbalanced(nb,color);
    end
end

function goStraightbalanced(nb,color)
    % wheel Speed Balance
    % Set the desired speed
    desiredSpeed = 10;
    % PID Controller parameters (need to be tuned for your robot)
    Kp = 0.01; % Proportional gain
    Ki = 0.0;  % Integral gain
    Kd = 0.003; % Derivative gain
    
    % Initialize PID variables
    prevError = 0;
    integral = 0;
    dt = 0.1; % Time step in seconds
    tf=tic;
    while true
        tic;
        [r,g,b]=getColorvalues(nb);
        if(r>g && r>b && r>110 && color =="r")
            setMotorstozero(nb);
            break;
        end
        
        if(b>g && b>r && b>110 && color == "b")
            setMotorstozero(nb);
            break;
        end
        
        tfc=toc(tf);
    
        if(tfc>2)
            nb.setMotors(0,0);
            break;
        end
        % Read the encoder counts for both motors
        leftCountsPerSec = nb.encoderRead(2).countspersec;
        rightCountsPerSec = -nb.encoderRead(1).countspersec;
    
        % Calculate the velocity error
        error = leftCountsPerSec - rightCountsPerSec;
    
        % Apply PID control to adjust motor speeds
        correction = Kp * error + Ki * integral + Kd * (error - prevError) / dt;
        
        % Adjust motor speeds to maintain straight motion
        nb.setMotor(2, desiredSpeed - correction);
        nb.setMotor(1, desiredSpeed + correction);
        fprintf('Time: %0.2f === Mot2: %0.2f === Mot1: %0.2f\n', tfc,desiredSpeed - correction,desiredSpeed + correction);
        % Update PID variables
        prevError = error;
        integral = integral + error * dt;
    
        % Pause for the specified time step
        dt=toc;
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
        disp("Color is Blue turn right and travel");
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
        disp("Color is Green straight travel");
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