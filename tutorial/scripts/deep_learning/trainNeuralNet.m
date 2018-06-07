%% Load the data
trainingData = importRescueData('training.csv');

%% Create neural network training input and output
X = [trainingData.sTime, trainingData.sDist, ...
     trainingData.sHP  , trainingData.sDamage]';
 
mode = 1;
switch mode
    case 1    % Binary classifier (alive or dead)
        Y = (trainingData.eHP > 0)';
    case 2    % Multi-class classifier
        hp_bins = [0 1 3000 7000 10000];
        bin_names = {'Dead', 'Critical', 'Injured', 'Stable'};
        Yclass = discretize(trainingData.eHP', hp_bins);
        Y = zeros(numel(hp_bins),numel(trainingData.eHP));
        for idx = 1:numel(hp_bins)
           Y(idx,Yclass==idx) = 1; 
        end
    case 3    % Regression (HP prediction)
        Y = trainingData.eHP';
end

%% Build and train a network
net = feedforwardnet([30 15]);
[net,tr] = train(net,X,Y);

%% Return training and validation accuracy

% Predict on training data
Ypred = sim(net,X);
    
% Predict on validation data
valData = importRescueData('validation.csv');
Xval = [valData.sTime, valData.sDist, ...
        valData.sHP  , valData.sDamage]';
Ypredval = sim(net,Xval);
switch mode
    case 1
        Yval = (valData.eHP > 0)';
    case 2
        Yvalclass = discretize(valData.eHP', hp_bins);
        Yval = zeros(numel(hp_bins),numel(valData.eHP));
        for idx = 1:numel(hp_bins)
           Yval(idx,Yvalclass==idx) = 1; 
        end
    case 3
        Yval = valData.eHP';
end

% Calculate metrics
switch mode
    case 1
        % Classification accuracy based on number of mismatches
        trainingAccuracy = 100 * nnz(Y==(Ypred>0.5))/numel(Y)
        validationAccuracy = 100 * nnz(Yval==(Ypredval>0.5))/numel(Yval)
    case 2
        [~,Ypredclass] = max(Ypred);
        trainingAccuracy = 100 * (1 - (nnz(Yclass~=Ypredclass)/2)/size(Y,2))
        [~,Ypredvalclass] = max(Ypredval);
        validationAccuracy = 100 * (1 - (nnz(Yvalclass~=Ypredvalclass)/2)/size(Yval,2))
    case 3
        % Regression error expressed as root mean square error
        % Normalized for a max HP value of 10000
        maxHP = 10000;
        trainingRMSE = mean(sqrt(((Y-Ypred)/maxHP).^2))
        validationRMSE = mean(sqrt(((Yval-Ypredval)/maxHP).^2))
end
