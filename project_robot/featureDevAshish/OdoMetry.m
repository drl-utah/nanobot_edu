%%
% Clear the workspace and command window
clc
clear all
% Create an instance of the nanobot class (assuming you have this class defined)
% Replace 'COM31' with the actual COM port your robot is connected to
nb = nanobot('COM31', 115200, 'serial');
%%
%%
% Left wheel individual odometry
% Robot and wheel specifications
wheelDiameter = 0.07; % Diameter of the wheel in meters
wheelBase = 0.14; % Distance between the wheels in meters
encoderCountsPerRevolution = 1440; % Encoder counts per wheel revolution
wheelCircumference = pi * wheelDiameter;
requiredCounts = encoderCountsPerRevolution*1/3;
% PID Controller parameters - These need to be tuned for your robot
Kp = 0.01; % Proportional gain
Ki = 0.0; % Integral gain
Kd = 0.003; % Derivative gain
b=13;
% Initialize PID variables
prevError = 0;
integral = 0;
dt = 0.1; % Time step in seconds
totalLeftEncoderCounts = nb.encoderRead(2).counts;
totalLeftEncoderCounts=0;
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
%%
%%
% Right wheel indvidual odometry
wheelDiameter = 0.07; % Diameter of the wheel in meters
wheelBase = 0.14; % Distance between the wheels in meters
encoderCountsPerRevolution = 1440; % Encoder counts per wheel revolution
wheelCircumference = pi * wheelDiameter;
requiredCounts = encoderCountsPerRevolution*4/3;
% PID Controller parameters - These need to be tuned for your robot
Kp = 0.01; % Proportional gain
Ki = 0.0; % Integral gain
Kd = 0.003; % Derivative gain
b=13;
% Initialize PID variables
prevError = 0;
integral = 0;
dt = 0.1; % Time step in seconds
totalRightEncoderCounts = nb.encoderRead(1).counts;
totalRightEncoderCounts=0;
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

%%

%%
% wheel Speed Balance
% Robot and wheel specifications
wheelDiameter = 0.07; % Diameter of the wheel in meters
encoderCountsPerRevolution = 1440; % Encoder counts per wheel revolution
requiredCounts = encoderCountsPerRevolution * 4 / 3;

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

%%