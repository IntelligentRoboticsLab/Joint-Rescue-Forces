function output = netPredictDeep(X,mode)

persistent singleClassNet multiClassNet regressionNet
if isempty(singleClassNet) || isempty(multiClassNet) || isempty(regressionNet)
    load neuralNetsDeep;
end

inp = reshape(X,[1 1 numel(X)]);

switch mode
    case 1      % Binary Classification
        output = classify(singleClassNet,inp);
    case 2      % Multi-Class Classification
        output = classify(multiClassNet,inp);
    case 3      % Regression
        output = double(predict(regressionNet,inp))*10000;
end

end

