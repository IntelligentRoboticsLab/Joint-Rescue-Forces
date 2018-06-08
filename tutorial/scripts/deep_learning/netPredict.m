function output = netPredict(X,mode)

persistent singleClassNet multiClassNet regressionNet
if isempty(singleClassNet) || isempty(multiClassNet) || isempty(regressionNet)
    load neuralNets;
end

switch mode
    case 1  % Binary classification
        Y = sim(singleClassNet,X);
        output = (Y > 0.5);
    case 2  % Multiclass classification
        Y = sim(multiClassNet,X);
        [~,output] = max(Y);
    case 3  % Regression
        output = sim(regressionNet,X);
end

end

