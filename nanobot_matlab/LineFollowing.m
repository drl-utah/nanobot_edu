% Clear the workspace and command window
clc
clear all

% Create an instance of the nanobot class (assuming you have this class defined)
% Replace 'COM31' with the actual COM port your robot is connected to
nb = nanobot('COM31', 115200, 'serial');

% Initialize the reflectance sensors (assuming 4 sensors)
nb.initReflectance();

% Calibrate sensors
% You should manually move the robot over the line and off the line to get max/min values
minReflectance  = [170,170,170,170]; % Replace these with actual minimum values observed during calibration
maxReflectance  = [1225,1225,1225,1225]; % Replace these with actual maximum values observed during calibration

% PID Controller parameters
Kp = 1; % Proportional gain - adjust based on testing
Ki = 0.005; % Integral gain - adjust based on testing
Kd = 0.0000001; % Derivative gain - adjust based on testing

% Initialize PID variables
prevError = 0;
integral = 0;
dt = 0.05; % Time interval for derivative and integral calculations, adjust as needed

% Main loop
while true
    % Read sensor values
    sensorVals = nb.reflectanceRead();
      calibratedVals =   struct('one', 0, 'two', 0, 'three', 0, 'four', 0);
    % Calibrate sensor readings
    calibratedVals.one = (sensorVals.one - minReflectance(1)) / (maxReflectance(1) - minReflectance(1));
    calibratedVals.two = (sensorVals.two - minReflectance(2)) / (maxReflectance(2) - minReflectance(2));
    calibratedVals.three = (sensorVals.three - minReflectance(3)) / (maxReflectance(3) - minReflectance(3));
    calibratedVals.four = (sensorVals.four - minReflectance(4)) / (maxReflectance(4) - minReflectance(4));
 
    % Calculate error with the new sensor orientation
    error = -2*calibratedVals.four - calibratedVals.three + calibratedVals.two + 2*calibratedVals.one;
    
    % Update integral and derivative
    integral = integral + error * dt;
    derivative = (error - prevError) / dt;
    
    % Calculate control signal
    controlSignal = Kp * error + Ki * integral + Kd * derivative;
    
    % Update previous error
    prevError = error;
    
    % Set motor speeds based on the control signal
    baseSpeed = 10; % Base speed for both motors
    speedM1 = baseSpeed + controlSignal; % Adjust right motor speed (IR1 is now rightmost)
    speedM2 = baseSpeed - controlSignal; % Adjust left motor speed (IR4 is now leftmost)
    
    % Apply the speeds to the motors
    nb.setMotor(1, speedM1); % Set right motor speed
    nb.setMotor(2, speedM2); % Set left motor speed
    
    % Optional: Add a delay to match the dt interval
    pause(dt);
end
