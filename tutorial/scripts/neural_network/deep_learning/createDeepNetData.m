function [X,Y,numClasses] = createDeepNetData(tableData,mode)
% Creates X (predictors) and Y (response), as well as outputs number of
% classes, from a RoboCupRescue data table

X = tableData{:,{'sTime','sDist','sHP','sDamage'}};

% Normalize and reshape the predictors
inputWeights = [100 1000000 10000 100];
X = (X - inputWeights/2)./inputWeights;
reshapedSize = [1 1 size(X,2) size(X,1)];
X = reshape(X,reshapedSize);

% Create the response and number of classes based on learning mode
switch mode
    case 1  % Binary classification
        Y = categorical(tableData.eHP > 0,[0 1],{'Dead','Alive'});
        numClasses = 2;
    case 2  % Multiclass classification
        hp_bins     = [0 1 3000 7000 10000];
        bin_names   = {'Dead', 'Critical', 'Injured', 'Stable'};
        numClasses  = numel(bin_names);
        Y = discretize(tableData.eHP, hp_bins, 'categorical', bin_names);
        Y = categorical(Y,'ordinal',false);
    case 3
        % Regression
        Y = tableData.eHP/10000;
        numClasses = 0;
end

end

