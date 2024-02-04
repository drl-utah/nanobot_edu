% Clear the workspace and command window
clc
clear all

% Create an instance of the nanobot class
nb = nanobot('COM8', 115200, 'serial');

% Initialize the ultrasonic sensors
nb.initUltrasonic1('D2', 'D3'); % Front sensor
nb.initUltrasonic2('D4', 'D5'); % Left sensor

% Desired distance from the left wall (in the same units used by your ultrasonic sensors, e.g., cm)
desiredDistance = 8; % Adjust based on your requirements

% PID Controller parameters for wall following
Kp = 1; % Proportional gain - adjust based on testing
Ki = 0; % Integral gain - adjust based on testing
Kd = 0; % Derivative gain - adjust based on testing

% Initialize PID variables
prevError = 0;
integral = 0;
dt = 0.05; % Time interval for derivative and integral calculations, adjust as needed

% Main loop
while true
    % Take a single ultrasonic reading from each sensor
    front = nb.ultrasonicRead1();
    left = nb.ultrasonicRead2();
    
    % Print distances for debugging
    fprintf('Front dist = %i   Left dist = %i\n', front, left);
    
    % Check for obstacles in front and adjust behavior
    if front < desiredDistance
        % Obstacle detected in front, stop or turn
        nb.setMotor(1, 0); % Stop right motor
        nb.setMotor(2, 0); % Stop left motor
        % Add logic here to handle turning if necessary
        continue; % Skip the rest of the loop
    end
    
    % Calculate error for wall following (left sensor)
    error = left - desiredDistance;
    
    % Update integral and derivative for PID
    integral = integral + error * dt;
    derivative = (error - prevError) / dt;
    
    % Calculate control signal for PID
    controlSignal = Kp * error + Ki * integral + Kd * derivative;
    
    % Update previous error
    prevError = error;
    
    % Set motor speeds based on the control signal
    baseSpeed = 50; % Base speed for both motors
    speedM1 = baseSpeed + controlSignal; % Adjust right motor speed
    speedM2 = baseSpeed - controlSignal; % Adjust left motor speed
    
    % Apply the speeds to the motors
    nb.setMotor(1, speedM1); % Set right motor speed
    nb.setMotor(2, speedM2); % Set left motor speed
    
    % Optional: Add a delay to match the dt interval
    pause(dt);
end
