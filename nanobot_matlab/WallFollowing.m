% Clear the workspace and command window
clc
clear all

% Create an instance of the nanobot class (assuming you have this class defined)
% Replace 'COM31' with the actual COM port your robot is connected to
nb = nanobot('COM31', 115200, 'serial');

% Initialize ultrasonic sensors
% Assuming front sensor is Ultrasonic1 and left sensor is Ultrasonic2
 nb.initUltrasonic1('D2','D3'); % Initialize front ultrasonic sensor
 nb.initUltrasonic2('D4','D5'); % Initialize left ultrasonic sensor

% % Threshold distance to consider a wall in front
frontWallThreshold = 10; % Adjust based on your requirement

% Desired distance from the wall (adjust based on your requirement)
desiredDist = 5; % Desired distance from the wall in centimeters


% PID Controller parameters for wall following
Kp = 0.5; % Proportional gain - adjust based on testing
Ki = 0.0001; % Integral gain - adjust based on testing
Kd = 0.0005; % Derivative gain - adjust based on testing


% Initialize PID variables
 prevError = 0;
 integral = 0;
 dt = 0.05; % Time interval for derivative and integral calculations, adjust as needed
% 
% 
% Main loop
while true
    % Read ultrasonic sensor values
    frontDist = nb.ultrasonicRead1()*0.01715; % Read front distance
    frontDist = 0;
    leftDist = nb.ultrasonicRead2()*0.01715; % Read left distance

    % Check for walls in front and to the left
    % if frontDist < frontWallThreshold
    %     % Wall detected in front, turn right
    %     nb.setMotor(1, -10); % Reverse right motor to turn right
    %     nb.setMotor(2, 10); % Keep left motor forward
    %     pause(1); % Adjust pause time based on how much you need to turn
    %     continue; % Skip the rest of the loop and check distances again
    % end
    if(leftDist>10)
    nb.setMotor(1,0);
    nb.setMotor(2,0);
    end

    % Following the wall on the left
    error = leftDist - desiredDist;

    % Update integral and derivative
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
    if (speedM1<0 && speedM1<10)
    speedM1 = -10;
    end
    if (speedM2<0 && speedM2<10)
    speedM2 = -10;
    end
    nb.setMotor(1, speedM1); % Set right motor speed
    nb.setMotor(2, speedM2);% Set left motor speed

    % Print distances for debugging
    fprintf('FD = %0.2f, LD = %0.2f, rightSPD: %0.2f, leftSPD: %0.2f \n', frontDist, leftDist,speedM1,speedM2);

    % Optional: Add a delay to match the dt interval
    pause(dt);
end