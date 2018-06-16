%% Neural network example using new Deep Learning functionality

%% Create neural network training input and output
mode = 3;
trainingData = importRescueData('training.csv');
[X,Y,numClasses] = createDeepNetData(trainingData,mode);

%% Build and train a network
hiddenLayers1 = [fullyConnectedLayer(50); ...         
                 batchNormalizationLayer; ...
                 reluLayer];
hiddenLayers2 = [fullyConnectedLayer(25); ...         
                 batchNormalizationLayer; ...
                 reluLayer];
switch mode
    case {1,2}
        classWeights = numel(Y) ./ countcats(Y)';
        outputLayers = [fullyConnectedLayer(numClasses); ... 
                        softmaxLayer; ... 
                        weightedClassificationLayer(classWeights)];
    case 3
        outputLayers = [fullyConnectedLayer(1); ...
                        regressionLayer];
end
layers = [imageInputLayer(reshapedSize(1:3)); ...
          hiddenLayers1; ...
          hiddenLayers2; ...
          dropoutLayer(0.2); ...
          outputLayers];
numBatches = 2;
options = trainingOptions('sgdm', ...
                          'InitialLearnRate',5e-3,...
                          'MaxEpochs',500, ...
                          'MiniBatchSize',ceil(size(X,4)/numBatches), ...
                          'Plots','training-progress');
[net,trainInfo] = trainNetwork(X,Y,layers,options);

%% Return training and validation accuracy
valData = importRescueData('validation.csv');
[Xval,Yval,numClasses] = createDeepNetData(valData,mode);

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
