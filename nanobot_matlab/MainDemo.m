%% Clean up
clear; clc; close all;
tic;
% Create an instance of the nanobot class
nb = nanobot('COM31', 115200, 'serial');

%% main code
% Intialize all sensors and motors
%initSensorsAndMotors(nb);
disp("Choose a Sequence...");
disp("1. Line following");
disp("2. Odometry");
disp("3. Wall Following");
botdirection=0;
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
            disp("Following Line.........");
            disp("Detected all black returnflagline = true robot about turn 180 deg,botdirection=-1 and foloow back line");
            botdirection =-1;
            disp("Following Line........ all black linecompletionflag = true ");
        end
        if(botdirection == 1)
            disp("GO Straight");
            disp("Following Line.........");
            disp("Detected all black returnflagline = true robot about turn 180 deg,botdirection=-1 and foloow back line");
            botdirection =-1;
            disp("Following Line........ all black linecompletionflag = true ");
        end
    elseif(currenttask == 2)
        if(botdirection ==-1)
            disp("Turn Left till middle two sensors read black");
            botdirection =0;
            disp("follow line till all black read, read color");
            disp("Perform odometry based on color")
        end
        if(botdirection == 1)
            disp("Turn Right till middle two sensors read black");
            botdirection =0;
            disp("follow line till all black read, read color");
            disp("Perform odometry based on color")
        end

    else
         if(botdirection == 0)
            disp("Turn Right till line detected...(middle 2 sensors black)");
            disp("Following line till front sensor is < 7cm.........");
            disp("Turn Right and follow wall with left offset distance of 7cm");
            disp("Following wall........ all black wallcompletionflag = true, Turn right till middle two sensors black ");
            botdirection = 1;
            disp("follow line ... all black wallcompleteflag = true" );

         end
         if(botdirection == -1)
            disp("Go Straight...");
            disp("Following line till front sensor is < 7cm.........");
            disp("Turn Right and follow wall with left offset distance of 7cm");
            disp("Following wall........ all black wallcompletionflag = true, Turn right till middle two sensors black ");
            botdirection = 1;
            disp("follow line ... all black wallcompleteflag = true" );

        end
        
    end


end
%%

function initSensorsAndMotors(nb)
    % Initialize sensors and motors
    nb.initReflectance(); % IR Sensor
    nb.initColor(); % Color sensor
    nb.initUltrasonic1('D2','D3'); % front sensor
    nb.initUltrasonic2('D4','D5'); % left sensor
    nb.setMotors(0,0);
end