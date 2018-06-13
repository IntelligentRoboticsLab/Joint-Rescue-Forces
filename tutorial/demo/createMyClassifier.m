% Source: 
% https://archive.ics.uci.edu/ml/datasets/Occupancy+Detection+
% Citation:
% Accurate occupancy detection of an office room from light, temperature, humidity and CO2 measurements using statistical learning models. Luis M. Candanedo, Véronique Feldheim. Energy and Buildings. Volume 112, 15 January 2016, Pages 28-39.

%% Read file
clc
addpath('occupancy_data');
T_train1 = readtable('datatraining.txt', ...
              'ReadVariableNames',true);
T_train1 = T_train1(:,3:end); % Remove first 2 columns
T_train2 = readtable('datatest.txt', ...
              'ReadVariableNames',true);
T_train2 = T_train2(:,3:end); % Remove first 2 columns
% Concatenate all data
trainData = [T_train1;T_train2];

%% Train the model
% classificationLearner
[myClassifier,valAccuracy] = trainClassifier(trainData); % Calls auto-generated function
disp(['Validation accuracy: ' num2str(valAccuracy*100) '%']);

%% Load 2nd test data set and predict on new data
testData = readtable('datatest2.txt', ...
              'ReadVariableNames',true);
testData = testData(:,3:end); % Remove first 2 columns

prediction = myClassifier.predictFcn(testData);
testAccuracy = nnz(prediction==testData{:,'Occupancy'})/size(testData,1);
disp(['Test accuracy: ' num2str(testAccuracy*100) '%']);

%% Save the classifier to a MAT-file
save myClassifier myClassifier