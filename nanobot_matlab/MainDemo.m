%% Clean up
clear; clc; close all;
tic;
% Create an instance of the nanobot class
nb = nanobot('COM31', 115200, 'serial');

%% main code
% Intialize all sensors and motors
%initSensorsAndMotors(nb);
botdirection =0;
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
            disp("Following Line.........");
            lineFollowing(nb);
            
            disp("Detected all black returnflagline = true robot about turn 180 deg,botdirection=-1 and foloow back line");
            botdirection =-1;
            linecompletionflag = true;
            disp("Following Line........ all black linecompletionflag = true ");
        end
        if(botdirection == 1)
            disp("GO Straight");
            goStraighttillline(nb);
            disp("Following Line.........");
            lineFollowing(nb);
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
            lineFollowing(nb);
           
            disp("Perform odometry based on color")
            performOdometrybycolor(nb);
            odometrycompletionflag =true;
            
        end
        if(botdirection == 1)
            disp("Turn Right till middle two sensors read black");
            turnRighttillline(nb);
            botdirection =0;
            disp("follow line till all black read, read color");
            lineFollowing(nb);
            disp("Perform odometry based on color")
            performOdometrybycolor(nb);
            odometrycompletionflag =true;
        end

    else
         if(botdirection == 0)
            disp("Turn Right till line detected...(middle 2 sensors black)");
            turnRighttillline(nb);
            disp("Following line till front sensor is < 7cm.........");
            lineFollowing(nb);
            disp("Turn Right and follow wall with left offset distance of 7cm");
            turnrighttillcurvediverges(nb);
            disp("Following wall........ all black wallcompletionflag = true, Turn right till middle two sensors black ");
            wallFollowing(nb);
            botdirection = 1;
            wallcompletionflag = true;
            disp("follow line ... all black wallcompleteflag = true" );

         end

         if(botdirection == -1)
            disp("Go Straight...");
            goStraighttillline(nb);
            disp("Following line till front sensor is < 7cm.........");
            lineFollowing(nb);
            disp("Turn Right and follow wall with left offset distance of 7cm");
            turnrighttillcurvediverges(nb);
            disp("Following wall........ all black wallcompletionflag = true, Turn right till middle two sensors black ");
            wallFollowing(nb);
            botdirection = 1;
            wallcompletionflag=true;
            disp("follow line ... all black wallcompleteflag = true" );
        end
        
    end
    if(linecompletionflag && wallcompletionflag && odometrycompletionflag)
        disp("Task Done!!!!!");
        disp("Elapsed Time is: %0.2f", toc);

    end

end
%%
function turnrighttillcurvediverges(nb)
    prevfu=0;
            while true
                nb.setMotor(2,10);
                nb.setMotor(1,0);
                [fu,~]=getUSvalues(nb);
                if(fu>prevfu && prevfu~=0)
                    setMotorstozero(nb);
                    break;
                end
                prevfu =fu;
            end
end

function turnLefttillline(nb)
            while true
                setMotorstoleft(nb);
                if (checkonLine(nb))
                    setMotorstozero(nb);
                    break;
                end
            end
end

function turnRighttillline(nb)
            while true
                setMotorstoright(nb);
                if (checkonLine(nb))
                    setMotorstozero(nb);
                    break;
                end
            end
end

function goStraighttillline(nb)
            while true
                setMotorstostraight(nb);
                if (checkonLine(nb))
                    setMotorstozero(nb);
                    break;
                end
            end
end

function [l,lm,rm,r] = getIRvalues(nb)
    minReflectance  = [200,200,200,200];
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
    nb.setMotor(1,10);
    while true
        if (checkonLine)
            setMotorstozero(nb);
            break;
        end
    end
end


function initSensorsAndMotors(nb)
    % Initialize sensors and motors
    nb.initReflectance(); % IR Sensor
    nb.initColor(); % Color sensor
    nb.initUltrasonic1('D2','D3'); % front sensor
    nb.initUltrasonic2('D4','D5'); % left sensor
    nb.setMotors(0,0);
end
function onLine = checkonLine(nb)
    [l,lm,rm,r] = getIRvalues(nb);
    if(l<300 && lm>300 && rm>300 && r<300)
        onLine = true;
    else
        onLine = false;
    end
end
function wallFollowing(nb)
% wall Following Code include the below condition
                % if (l>300 && lm>300 && rm>300 && r>300)
                %     setMotorstozero(nb);
                 %    turnRighttillline(nb);
                % end
    
end
function lineFollowing(nb)
% Line Following Code include the below condition
                % if (l>300 && lm>300 && rm>300 && r>300)
                %     setMotorstozero(nb);
                %     aboutTurn(nb);
                % end
end

function performOdometrybycolor(nb)
     [r,g,b]= getColorvalues(nb);
     if(r>g &&r>b)
                performOdometry(nb,-angleDeviation,odoDist);
            end
            if(b>g &&b>r)
                performOdometry(nb,0,odoDist);
            end
            if(g>r &&g>b)
                performOdometry(nb,angleDeviation,odoDist);
            end

end

function performOdometry(nb,angle,dist)
%Odometry code
end