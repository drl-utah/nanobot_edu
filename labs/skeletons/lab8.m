%%%%%%%%%%%%%
% ECE 3610
% LAB 8 -- Actuators 1: DC Motors and Encoders
%%%%%%%%%%%%%

%%%%%%%%%%%%%
% Feedback between sensor inputs and actuator outputs is at the core of a
% huge number of robotic and cyberphysical system applications. In this
% lab, we will explore two different methods of sensorimotor feedback with
% naive control implementations. We will build on some of the underlying
% skills later, but for now we will largely be able to use code and
% concepts from earlier labs.
%
% Deliverables:
%   - An IMU tilt sensor that controls the speed of a DC motor
%   rotation, and reverses direction depending on tilt direction
%   - A flex sensor that sets the angle of a servo motor to match the
%   bend angle.
%
% Extensions:
%   - Use the IMU tilt sensor to control the speed of the DC motor using
%   one axis, and change the angle of the servo motor using the other.
%   (Speed and steering control!)
%%%%%%%%%%%%%%

%% 1. CONNECT TO YOUR NANOBOT
%  Remember to replace the first input argument with text corresponding to
%  the correct serial port you found through the Arduino IDE. Note that the
%  clc and clear all will clear your command window and all saved
%  variables!

clc
clear all
nb = nanobot('COM47', 115200, 'serial');

%% 2. DC motor speed based on IMU Tilt
%  Using readings from your onboard IMU, control the speed of your DC motor
%  based on the tilt angle of the board. Note, there may be multiple
%  approaches to map the tilt to a PWM value.
%   Connect your motor to one of the motor ports on your motor carrier
%   using the screw-down connectors, and make sure to take note of which
%   motor pins it corresponds to.
tic

while (toc < 20) % ADJUST ME if you want to have longer/shorter trials
    %Here is an example of taking 5 accelerometer readings, then averaging
    %each axis:
    numreads = 5;
    vals = zeros(3,numreads);
    for i = 1:numreads
        val = nb.accelRead();
        vals(1,i) = val.x;
        vals(2,i) = val.y;
        vals(3,i) = val.z;
    end
    %Note the index, getting every column in a specific row for each axis:
    meanx = mean(vals(1,:));
    meany = mean(vals(2,:));
    meanz = mean(vals(3,:));
    
    % Implement your tilt to motor PWM code below:
    % HINT: Think about the value ranges of your IMU data and setMotor()'s
    % accepted PWM values
    % NOTE: Depending on your implementation, you may find it useful to
    % restrict your expected motor speed range. +/-100% duty cycle can be a
    % bit aggressive on the motor and will drain your battery quickly.


end
nb.setMotor('?', 0);

%% Run me if you manually interrupted the above code to reset the motor speed to zero
nb.setMotor('?', 0);

%% 3. Servo angle matching flex sensor
% For this section, we will try to match the bend angle of our flex sensor
% with our servo's wiper. Keep in mind, the resistance of the flex sensor 
% is likely to only change significantly between 0 and 90 degrees. Hook up
% your servo to one of the servo ports (with the brown wire aligned to
% ground, the white stripe on the underside of the board), and set up a
% voltage divider with your flex sensor and 10k resistor. Connect it to one
% of your analog read pins, and use the value to determine the
% corresponding flex sensor and servo angles.
tic

while (toc < 30)
    % Find average value over 10 samples to get a steady signal
    numreads = 10;
    vals = zeros(1,numreads);
    for i = 1:numreads
        vals(i) = nb.analogRead('?');
    end
    meanval = mean(vals);

    % Determine what range of values correspond to bend angles between 0
    % and 90 degrees:

    % Knowing that your servo angle range should go between 0 and 90,
    % convert your analog value into a servo angle, and set the
    % corresponding servo motor.

end
nb.setServo('?', 0);

%% Run me to reset servo angle to zero if manually interrupted
nb.setServo('?',0);

%% 4. EXTENSION (optional)
%  Use the IMU to control motor speed and the servo angle at the same
%  time!

%% X. DISCONNECT
%  Clears the workspace and command window, then
%  disconnects from the nanobot, freeing up the serial port.

clc
delete(nb);
clear('nb');
clear all