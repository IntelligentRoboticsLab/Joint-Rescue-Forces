%% Load the data
trainingData = importRescueData('training.csv');

%% Create neural network training input and output
X = trainingData{:,{'sTime','sDist','sHP','sDamage'}};
reshapedSize = [1 1 size(X,2) size(X,1)];
X = reshape(X,reshapedSize);
mode = 1;
switch mode
    case 1
        Y = categorical(trainingData.eHP > 0,[0 1],{'Dead','Alive'});
        numClasses = 2;
    case 2
        hp_bins     = [0 1 3000 7000 10000];
        bin_names   = {'Dead', 'Critical', 'Injured', 'Stable'};
        numClasses  = numel(bin_names);
        Y = discretize(trainingData.eHP, hp_bins, 'categorical', bin_names);
    case 3
        Y = trainingData.eHP/10000;
end

%% Build and train a network
hiddenLayers1 = [fullyConnectedLayer(30); ...         
                 batchNormalizationLayer; ...
                 reluLayer];
hiddenLayers2 = [fullyConnectedLayer(15); ...         
                 batchNormalizationLayer; ...
                 reluLayer];
switch mode
    case {1,2}
        outputLayers = [fullyConnectedLayer(numClasses); ... 
                        softmaxLayer; ... 
                        classificationLayer];
    case 3
        outputLayers = [fullyConnectedLayer(1); ...
                        regressionLayer];
end
layers = [imageInputLayer(reshapedSize(1:3)); ...
          hiddenLayers1; ...
          hiddenLayers2; ...
          dropoutLayer(0.75); ...
          outputLayers];
numBatches = 1;
options = trainingOptions('sgdm', ...
                          'InitialLearnRate',1e-3, ...
                          'MaxEpochs',150, ...
                          'MiniBatchSize',ceil(reshapedSize(end)/numBatches), ...
                          'Plots','training-progress');
[net,trainInfo] = trainNetwork(X,Y,layers,options);

%% Return training and validation accuracy
valData = importRescueData('validation.csv');
Xval = valData{:,{'sTime','sDist','sHP','sDamage'}};
reshapedSize = [1 1 size(Xval,2) size(Xval,1)];
Xval = reshape(Xval,reshapedSize);
switch mode
    case 1
        Yval = categorical(valData.eHP > 0,[0 1],{'Dead','Alive'});
    case 2
        Yval = discretize(valData.eHP, hp_bins, 'categorical', bin_names);
    case 3
        Yval = valData.eHP/10000;
end

switch mode
    case {1,2}
        Ypred = classify(net,X);
        Ypredval = classify(net,Xval);
        % Calculate percentage of matching classifications
        trainingAccuracy    = 100 * nnz(Y==Ypred)/numel(Y)
        validationAccuracy  = 100 * nnz(Yval==Ypredval)/numel(Yval)
    case 3
        Ypred = predict(net,X);
        Ypredval = predict(net,Xval);
        % Regression error expressed as root mean square error
        % Normalized for a max HP value of 10000
        trainingRMSE = mean(sqrt((Y-Ypred).^2))
        validationRMSE = mean(sqrt((Yval-Ypredval).^2))
end
