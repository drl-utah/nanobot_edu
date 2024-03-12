%% Intelligence Lab 2: Intro to Machine Learning & Feature Generation

clear; clc; close all; %initialization
r = nanobot('COM47', 115200, 'serial'); %connect to MKR
r.ledWrite(0);

%% Specify initial parameters:
trialCount = 4; % specify how many times you will make each gesture
digits = [0, 1]; % specify which digits you will be gesturing
numreads = 100; % about 1.5 seconds (on serial)
%% Collect Multiple Gesture
gestureCount = length(digits); % determine the number of gestures
data = cell(gestureCount, trialCount+1); % preallocate cell array
for i = 1:gestureCount
    data{i,1} = digits(i);
end

countdown("Beginning in", 3); %display a countdown to prep user to move accelerometer 

for a = 1:gestureCount % iterate through all the gestures
    b = 1; %index for trials
    while b <= trialCount % iterate through all the trials
        fprintf("Draw a %d (%d of %d)",digits(a), b, trialCount); % Displays the prompt in the command window
        r.ledWrite(1);                % Begin recording data

        % Gesture is performed during the segement below
        for i = 1:numreads
            val = r.accelRead();
            vals(1,i) = val.x;
            vals(2,i) = val.y;
            vals(3,i) = val.z;
        end

        r.ledWrite(0);          % Set the LED to red          % Set the LED to red
        try
            data{a,b+1} = [vals(1,:);vals(2,:);vals(3,:)]; % Truncate the data
            b = b + 1;
        catch
            disp("Data capture failed. Trying again in 3 seconds")
            pause(3);
        end
        clc; % clear the command line
        pause(1); % wait one second
    end
end

pause(1); % wait a second
clc; % clear command line

if menu("Would you like to save the dataset you just recorded?", "Yes", "No") == 1
    t = clock;
    filename = sprintf("%d%d%d_%d%d%d_TrainingSet_%dGestures%dTrials",t(1),t(2),t(3),t(4),t(5),round(t(6)),height(data),width(data)-1);
    save(filename, "data");
end

%% Calculate 3 features for each image (one per accelerometer axis)
gestureCount = height(data);
trialCount = width(data)-1;
Features = zeros(gestureCount, trialCount, 3); % 3 because the accelerometer sends 3 axes of data
for a = 1:gestureCount %iterate through all gestures
    for b = 1:trialCount %iterate through all trials
        singleLetter = data{a,b}; %get the individual gesture data

        %%%%%%%%%% CALCULATE FEATURES (YOUR CODE GOES HERE) %%%%%%%%%%%%%%%

        Features(a,b,:) = rand(3,1);

        %%%%%%%%%%%%%%%%%%%%%%%% END OF YOUR CODE %%%%%%%%%%%%%%%%%%%%%%%%%
    end
end

%% Plot features
figure(); hold on; grid on; % create plot
for a = 1:gestureCount
    scatter3(Features(a,:,1), Features(a,:,2), Features(a,:,3), 'filled'); %x, y, z values from features 1, 2, and 3 respectively
end

%% Perform linear discriminate analysis
digits = [data{:,1}];
TrainingFeatures = reshape(Features,[trialCount*gestureCount,3]); %reshape data so that it's #observations by #features
TrainingLabels = repmat(digits, [1, trialCount]); %assign appropriate label to each observation (i.e., 0 or 1)
LDA = fitcdiscr(TrainingFeatures,TrainingLabels); %perform LDA

%% Plot features & LDA
figure(); hold on; grid on; % create plot
for a = 1:gestureCount
    scatter3(Features(a,:,1), Features(a,:,2), Features(a,:,3), 'filled'); %x, y, z values from features 1, 2, and 3 respectively
end
limits = [xlim ylim zlim];
K = LDA.Coeffs(1,2).Const;
L = LDA.Coeffs(1,2).Linear;
f = @(x1,x2,x3) K + L(1)*x1 + L(2)*x2 + L(3)*x3;
h2 = fimplicit3(f, limits);

%% Run-Time Predictions (copy of lab 1 code with determiation replaced with LDA predictions)

% make sure NN exists
if(~exist('LDA'))
    error("You have not yet performed LDA! Be sure you run this section AFTER you have performed LDA.");
end

% collect gesture
clear r singleLetter
r = nanobot('COM5', 115200, 'serial'); %connect to MKR
numreads = 100; % about 1.5 seconds (on serial)
r.ledWrite(0);
pause(.5);
countdown("Beginning in", 3);
disp("Make A Gesture!");
r.ledWrite(1); % Begin recording data

% Gesture is performed during the segement below
for i = 1:numreads
    val = r.accelRead();
    vals(1,i) = val.x;
    vals(2,i) = val.y;
    vals(3,i) = val.z;
end

r.ledWrite(0); % Set the LED to red

singleLetter = [vals(1,:);vals(2,:);vals(3,:)];

% put accelerometer data into LDA input form
LDAinput = zeros(1,3);

%%%%%%%%%%%%%%% CALCULATE FEATURE AGAIN (YOUR CODE GOES HERE) %%%%%%%%%%%%%

LDAinput(1,:) = rand(3,1); % REPLACE

%%%%%%%%%%%%%%%%%%%%%%%%%%% END OF YOUR CODE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Prediction based on NN
LDAprediction = predict(LDA,LDAinput);

% Plot with label
figure(); plot(singleLetter', 'LineWidth', 1.5); %plot accelerometer traces
legend('X','Y','Z'); ylabel('Acceleration'); xlabel('Time') %label axes
title("Classification:", string(LDAprediction)); %title plot with the label