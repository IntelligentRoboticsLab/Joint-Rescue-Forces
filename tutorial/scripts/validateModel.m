%% Load validation table
validationData = importRescueData('validation.csv');
hp_bins = [0 1 3000 7000 10000];
bin_names = {'Dead', 'Critical', 'Injured', 'Stable'};
validationData.hp_class = discretize(validationData.eHP, hp_bins, 'categorical', bin_names);

%% Do prediction on all the data
load targetSelectorModel
predictions = targetSelectorModel.predictFcn(validationData);

%% Use prediction and ground truth to compute validation accuracy
numCorrect = nnz(predictions == validationData.hp_class);
validationAccuracy = numCorrect/size(validationData,1);
fprintf('Validation accuracy: %.2f%%\n', validationAccuracy*100);