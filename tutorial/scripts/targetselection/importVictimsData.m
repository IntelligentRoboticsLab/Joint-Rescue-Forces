trainingData = importRescueData('training.csv');
validationData = importRescueData('validation.csv');

TData = trainingData(:, 2:end);
VData = validationData(:, 2:end);

hp_bins = [0 1 3000 7000 10000];
bin_names = {'Dead', 'Critical', 'Injured', 'Stable'};
TData.hp_class = discretize(TData.eHP, hp_bins, 'categorical', bin_names);