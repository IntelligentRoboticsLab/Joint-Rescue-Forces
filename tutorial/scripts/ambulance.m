trainingData = importRescueData('training.csv');
validationData = importRescueData('validation.csv');

TData = trainingData(:, [2:5 9]);
VData = validationData(:, [2:5 9]);

hp_bins = [0 1 3000 7000 10000];
bin_names = {'Dead', 'Critical', 'Injured', 'Stable'};
TData.hp_class = discretize(TData.eHP, hp_bins, 'categorical', bin_names);