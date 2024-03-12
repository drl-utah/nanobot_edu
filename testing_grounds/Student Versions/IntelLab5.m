%% Intelligence Lab 5: Data Augmentation and Software Engineering 2.0

clear; clc; close all; %initialization

%% Load in Merged Training Sets
file = uigetfile('.mat');
load(file);

[h, w] = size(data);
gestureCount = h;
trialCount = w-1;
digits = [data{:,1}];

%% Data Augmentation
dataaug = data;
for a = 1:gestureCount
    c = 2;
    for b = 1:trialCount
        temp = data{a,b+1};
        % augment by flipped axes
        dataaug{a,trialCount+c} = temp([1,3,2],:);
        dataaug{a,trialCount+c+1} = temp([2,1,3],:);
        dataaug{a,trialCount+c+2} = temp([2,3,1],:);
        dataaug{a,trialCount+c+3} = temp([3,2,1],:);
        dataaug{a,trialCount+c+4} = temp([3,1,2],:);
        % augment by inverting axes
        dataaug{a,trialCount+c+5} = bsxfun(@times,temp,[-1;1;1]);
        dataaug{a,trialCount+c+6} = bsxfun(@times,temp,[1;-1;1]);
        dataaug{a,trialCount+c+7} = bsxfun(@times,temp,[1;1;-1]);
        dataaug{a,trialCount+c+8} = bsxfun(@times,temp,[1;-1;-1]);
        dataaug{a,trialCount+c+9} = bsxfun(@times,temp,[-1;-1;1]);
        dataaug{a,trialCount+c+10} = bsxfun(@times,temp,[-1;-1;-1]);
        % augment by shifting time
        dataaug{a,trialCount+c+11} = circshift(temp,5,2);
        dataaug{a,trialCount+c+12} = circshift(temp,10,2);
        dataaug{a,trialCount+c+13} = circshift(temp,15,2);
        dataaug{a,trialCount+c+14} = circshift(temp,20,2);
        dataaug{a,trialCount+c+15} = circshift(temp,25,2);
        % augment by skewing time
        dataaug{a,trialCount+c+16} = imresize(temp(:,5:end-5), [3 100], 'bilinear');
        dataaug{a,trialCount+c+17} = imresize(temp(:,10:end-10), [3 100], 'bilinear');
        dataaug{a,trialCount+c+18} = imresize(temp(:,15:end-15), [3 100], 'bilinear');
        dataaug{a,trialCount+c+19} = imresize(temp(:,20:end-20), [3 100], 'bilinear');
        dataaug{a,trialCount+c+20} = imresize(temp(:,25:end-25), [3 100], 'bilinear');
        % augment by amplifying movements
        dataaug{a,trialCount+c+21} = temp*1.1;
        dataaug{a,trialCount+c+22} = temp*1.2;
        dataaug{a,trialCount+c+23} = temp*1.3;
        dataaug{a,trialCount+c+24} = temp*1.4;
        dataaug{a,trialCount+c+25} = temp*1.5;
        % augment by adding noise
        dataaug{a,trialCount+c+26} = awgn(temp,30,'measured');
        dataaug{a,trialCount+c+27} = awgn(temp,25,'measured');
        dataaug{a,trialCount+c+28} = awgn(temp,20,'measured');
        dataaug{a,trialCount+c+29} = awgn(temp,18,'measured');
        dataaug{a,trialCount+c+30} = awgn(temp,15,'measured');
        c = c + 31;
    end
end
figure();
ax(1) = nexttile; plot(data{1,2}', 'LineWidth', 1.5); title("Original Data");
ax(2) = nexttile; ax2 = plot(dataaug{1,24}', 'LineWidth', 1.5); title("Swapped Axis");
ax(3) = nexttile; ax3 = plot(dataaug{1,30}', 'LineWidth', 1.5); title("Inverted Axis");
ax(4) = nexttile; ax4 = plot(dataaug{1,33}', 'LineWidth', 1.5); title("Time Shift");
ax(5) = nexttile; ax5 = plot(dataaug{1,40}', 'LineWidth', 1.5); title("Time Skew");
ax(6) = nexttile; ax6 = plot(dataaug{1,45}', 'LineWidth', 1.5); title("Amplitude Amplification");
ax(7) = nexttile; ax7 = plot(dataaug{1,50}', 'LineWidth', 1.5); title("Gaussian Noise");
linkaxes(ax, 'xy');
sgtitle("Example of Data Augmentation")
% shuffle order
justData = dataaug(:,2:end);
for a = 1:gestureCount
    order = randperm(length(justData));
    justData(a,order) = justData(a,:);
end
dataaug(:,2:end) = justData;
data = dataaug;
%% Store Data as "Images" for Neural Network
gestureCount = height(data);
trialCount = width(data)-1;

TrainingFeatures = zeros(3,100,1,gestureCount*trialCount); %features are stored as a stack of 3D images (4D array)
labels = zeros(1,gestureCount*trialCount); %labels are stored as a simple number (1D array)

k=1; %simpler counter
for a = 1:gestureCount %iterate through gestures
    for b = 1:trialCount %iterate through trials
        TrainingFeatures(:,:,:,k) = data{a,b}; %put data into image stack
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

%%%%%%%%%%%%%%%%%%%%%% YOUR MODIFICATIONS GO HERE %%%%%%%%%%%%%%%%%%%%%%%%%%

learnRate = 0.000000001; %how quickly network makes changes and learns
maxEpoch = 10; %how long the network learns

layers= [ ... %NN architecture for a conv net
    imageInputLayer([inputsize1,inputsize2,1])
    convolution2dLayer([1,1],20) %size of convolution (e.g., 3x10 means across all accelerometers and 10 time steps)
    batchNormalizationLayer
    reluLayer
    convolution2dLayer([1,1],20) %size of convolution (e.g., 3x10 means across all accelerometers and 10 time steps)
    batchNormalizationLayer
    reluLayer
    fullyConnectedLayer(20)
    batchNormalizationLayer
    dropoutLayer(.75) %rate of dropout (e.g., 0.01 = 1%). 
    fullyConnectedLayer(numClasses)
    softmaxLayer
    classificationLayer
    ];

%%%%%%%%%%%%%%%%%%%%%%%%% END OF YOUR MODIFICATIONS %%%%%%%%%%%%%%%%%%%%%

options = trainingOptions('sgdm','InitialLearnRate', learnRate, 'MaxEpochs', maxEpoch,...
    'Shuffle','every-epoch','Plots','training-progress', 'ValidationData',{xTest,yTest}); %options for NN

%% Train Neural Network

[myNeuralNetwork, info] = trainNetwork(xTrain,yTrain,layers,options); %output is the trained NN

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

rtdata = [vals(1,:);vals(2,:);vals(3,:)];

% put accelerometer data into NN input form
xTestLive = zeros(3,100,1,1);
xTestLive(:,:,1,1) = rtdata;

% Prediction based on NN
prediction = classify(myNeuralNetwork,xTestLive);

% Plot with label
figure(); plot(rtdata', 'LineWidth', 1.5); %plot accelerometer traces
legend('X','Y','Z'); ylabel('Acceleration'); xlabel('Time') %label axes
title("Classification:", string(prediction)); %title plot with the label