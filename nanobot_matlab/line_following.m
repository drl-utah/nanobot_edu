%% INIT

clc
clear all

% Create an instance of the nanobot class
nb = nanobot('COM50', 115200, 'wifi');

%% LINE FOLLOWING

nb.initReflectance();

% TUNING:
kp = 0.0007;
ki = 0.00035;
kd = 0.00035;

% dt option:
% dt = 0.05;

vals = 0;
prevError = 0;
errorSum = 0;
timeDif = 0;
whiteThresh = 300;
blackThresh = 800;

motor1BaseSpeed = 10;
motor2BaseSpeed = 10;

tic
while (1)
    vals = nb.reflectanceRead();
    %fprintf('one: %.2f, two: %.2f, three: %.2f four: %.2f\n', vals.one, vals.two, vals.three, vals.four);
    % timeDif = toc - timeDif; % For calculating i and d

    % Calculate offset error (p):
    error = -2*vals.one - vals.two + vals.three + 2*vals.four;

    % Calculate P, I, and D terms
    errorSum = error + errorSum;

    errorDiff = prevError - error;

    % Set PID
    control = kp*error + ki*errorSum + kd*errorDiff;
    
    % Cap for sanity
    if control > motor1BaseSpeed
        control = motor1BaseSpeed;
    elseif control < -motor1BaseSpeed
        control = -motor1BaseSpeed;
    end

    % STATE CHECKING:
    if (vals.one < whiteThresh & ...
            vals.two < whiteThresh & ...
            vals.three < whiteThresh & ...
            vals.four < whiteThresh)
        % ALL WHITE CONDITION
        nb.setMotor(1, 0);
        nb.setMotor(2, 0);
    elseif (vals.one > blackThresh & ...
            vals.two > blackThresh & ...
            vals.three > blackThresh & ...
            vals.four > blackThresh)
        % ALL BLACK CONDITION
        nb.setMotor(1, 0);
        nb.setMotor(2, 0);
    else
        % LINE DETECTED:
        % Set motor wheel speed:
        nb.setMotor(1, round(motor1BaseSpeed - control));
        nb.setMotor(2, round(motor2BaseSpeed + control));
    end

    prevError = error;
end
%%
% clear motors
nb.setMotor(1, 0);
nb.setMotor(2, 0);


%% Clear all;
clc
delete(nb);
clear all