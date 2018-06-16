%% Neural network example using old Neural Network functionality

%% Create neural network training input and output
mode = 1;
trainingData = importRescueData('training.csv');
[X,Y,Yclass] = createNeuralNetData(trainingData,mode);

%% Build and train a network
net = feedforwardnet([30 15]);
[net,tr] = train(net,X,Y);

%% Return training and validation accuracy

% Predict on training data
Ypred = sim(net,X);
    
% Predict on validation data
valData = importRescueData('validation.csv');
[Xval,Yval,Yvalclass] = createNeuralNetData(valData,mode);
Ypredval = sim(net,Xval);

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
