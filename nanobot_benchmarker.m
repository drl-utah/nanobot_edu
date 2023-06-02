clc
clear all

% Create an instance of the nanobot class
nb = nanobot('/dev/cu.usbmodem141101', 115200);

% Set the number of iterations for the benchmark
numIterations = 100;

% Benchmark digitalRead operation
start = tic;
for i = 1:numIterations
    nb.digitalRead('D2'); % Replace with the desired pin number
end
elapsedTime = toc(start);
digitalReadFreq = numIterations / elapsedTime;

% Benchmark analogRead operation
start = tic;
for i = 1:numIterations
    nb.analogRead('A1'); % Replace with the desired pin number
end
elapsedTime = toc(start);
analogReadFreq = numIterations / elapsedTime;

% Benchmark digitalWrite operation
start = tic;
for i = 1:numIterations
    nb.digitalWrite('D3', 1); % Replace with the desired pin number and value
end
elapsedTime = toc(start);
digitalWriteFreq = numIterations / elapsedTime;

% % Benchmark digitalWrite operation
start = tic;
for i = 1:numIterations
    vals = nb.accelRead();
end
elapsedTime = toc(start);
accelReadFreq = numIterations / elapsedTime;

% Display the results
disp(['Digital Read Frequency: ', num2str(digitalReadFreq), ' Hz']);
disp(['Analog Read Frequency: ', num2str(analogReadFreq), ' Hz']);
disp(['Digital Write Frequency: ', num2str(digitalWriteFreq), ' Hz']);
disp(['Accelerometer Read Frequency: ', num2str(accelReadFreq), ' Hz']);

% Delete the nanobot instance
delete(nb);
