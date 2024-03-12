%% Intelligence Lab 3: Perceptron Classification

clear; clc; close all; %initialization
r = nanobot('COM5', 115200, 'serial'); %connect to MKR
r.ledWrite(0);

%% Specify initial parameters:
trialCount = 5; % specify how many times you will make each gesture
digits = 0:1; % specify which digits you will be gesturing
numreads = 100; % about 1.5 seconds (on serial)
vals = zeros(3,numreads);
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

%% Calculate features for each image
% You may also load in a saved Training set and run from this section
% onward.
gestureCount = height(data);
trialCount = width(data)-1;
Features = zeros(gestureCount, trialCount, 3); % 3 because the accelerometer sends 3 axes of data
for a = 1:gestureCount %iterate through all gestures
    for b = 1:trialCount %iterate through all trials
        singleLetter = data{a,b+1}; %get the individual gesture data   
        
        Features(a,b,:) = rand(3,1); % YOU CAN MODIFY THIS LINE
        
    end
end

%% Plot features
figure(); hold on; grid on; % create plot
for a = 1:gestureCount
    scatter3(Features(a,:,1), Features(a,:,2), Features(a,:,3), 'filled');
end

%% Store Data as "Images" for Neural Network
TrainingFeatures = zeros(3,1,1,gestureCount*trialCount); %features are stored as a stack of 3D images (4D array)
labels = zeros(1,gestureCount*trialCount); %labels are stored as a simple number (1D array)

k=1; %simpler counter
for a = 1:gestureCount %iterate through gestures
    for b = 1:trialCount %iterate through trials
        TrainingFeatures(:,:,:,k) = Features(a,b,:); %put each feature into image stack
        labels(k) = data{a,1}; %put each label into label stack
        k = k + 1; %increment
    end
end
labels = categorical(labels); %convert labels into categorical

%% Split Training and Testing Data
selection = ones(1,gestureCount*trialCount); %allocate logical array
selectionIndices = []; %initialization
for b = 1:gestureCount %save 1/4 of the data for testing
    selectionIndices = [selectionIndices,  round(linspace(1,trialCount,round(trialCount/4))) + (trialCount*(b-1))];
end
selection(selectionIndices) = 0; %set logical to zero

%training data
xTrain = TrainingFeatures(:,:,:,logical(selection)); %get subset (3/4) of features to train on
yTrain = labels(logical(selection)); %get subset (3/4) of labels to train on
%testing data
xTest = TrainingFeatures(:,:,:,~logical(selection)); % get subset (1/4) of features to test on
yTest = labels(~logical(selection)); %get subset (1/4) of labels to test on

%% Define Neural Network

[inputsize1,inputsize2,~] = size(TrainingFeatures); %input size is defined by features
numClasses = length(unique(labels)); %output size (classes) is defined by number of unique labels

%%%%%%%%%%%%%%%%%%%%% YOU SHOULD MODIFY THESE PARAMETERS %%%%%%%%%%%%%%%%%%%

learnRate = .00000000001; %how quickly network makes changes and learns
maxEpoch = 10; %how long the network learns

%%%%%%%%%%%%%%%%%%%%%%% END OF YOUR MODIFICATIONS %%%%%%%%%%%%%%%%%%%%%%

layers= [ ... %NN architecture for a simple perceptron
    imageInputLayer([inputsize1,inputsize2,1])
    fullyConnectedLayer(numClasses)
    softmaxLayer
    classificationLayer
    ];

options = trainingOptions('sgdm','InitialLearnRate', learnRate, 'MaxEpochs', maxEpoch,...
    'Shuffle','every-epoch','Plots','training-progress', 'ValidationData',{xTest,yTest}); %options for NN

%% Train Neural Network

[myNeuralNetwork,info] = trainNetwork(xTrain,yTrain,layers,options); %output is the trained NN

%% Test Neural Network

t = 1:length(info.TrainingAccuracy);
figure();
subplot(2,2,1);
plot(info.TrainingAccuracy,'LineWidth',2,'Color',"#0072BD");
hold on;
plot(t(~isnan(info.ValidationAccuracy)),info.ValidationAccuracy(~isnan(info.ValidationAccuracy)),'--k','LineWidth',2,'Marker','o');
title("Training Accuracy")
legend("Training Accuracy","Validation Accuracy");
xlabel("Iterations");
ylabel("Accuracy (%)");

subplot(2,2,3);
plot(info.TrainingLoss,'LineWidth',2,'Color',"#D95319");
hold on;
plot(t(~isnan(info.ValidationLoss)),info.ValidationLoss(~isnan(info.ValidationLoss)),'--k','LineWidth',2,'Marker','o');
title("Training Loss")
legend("Training Loss","Validation Loss");
xlabel("Iterations");
ylabel("RMSE");

predictions = classify(myNeuralNetwork, xTest)'; %classify testing data using NN
disp("The Neural Network Predicted:"); disp(predictions); %display predictions
disp("Correct Answers"); disp(yTest); % display correct answers
subplot(2,2,[2,4]); confusionchart(yTest,predictions); % plot a confusion matrix
title("Confusion Matrix")

%% View Neural Network

figure(); plot(myNeuralNetwork); % visualize network connections
disp(myNeuralNetwork.Layers); % view layers
disp(myNeuralNetwork.Layers(2)); % view fully connect layer
disp(myNeuralNetwork.Layers(2).Weights); % view weights for each layer
disp(myNeuralNetwork.Layers(2).Bias); % view offset for each layer

%% Run-Time Predictions (copy of lab 1 code with determiation replaced with nn code)

% make sure NN exists
if(~exist('myNeuralNetwork'))
    error("You have not yet created your neural network! Be sure you run this section AFTER your neural network is created.");
end

% collect gesture
clear r;
r = nanobot('COM5', 115200, 'serial'); %connect to MKR
r.ledWrite(0);
pause(.5);
countdown("Beginning in", 3);
disp("Make A Gesture!");
r.ledWrite(1); % Begin recording data
numreads = 100; % about 1.5 seconds (on serial)

% Gesture is performed during the segement below
for i = 1:numreads
    val = r.accelRead();
    vals(1,i) = val.x;
    vals(2,i) = val.y;
    vals(3,i) = val.z;
end

r.ledWrite(0); % Set the LED to red

singleLetter = [vals(1,:);vals(2,:);vals(3,:)];

% put accelerometer data into NN input form
xTestLive = zeros(3,1,1,1);

xTestLive(:,:,:,1) = rand(3,1); % YOU CAN MODIFY THIS LINE 

% Prediction based on NN
prediction = classify(myNeuralNetwork,xTestLive);

% Plot with label
figure(); plot(singleLetter', 'LineWidth', 1.5); %plot accelerometer traces
legend('X','Y','Z'); ylabel('Acceleration'); xlabel('Time') %label axes
title("Classification:", string(prediction)); %title plot with the label