function [X,Y,Yclass] = createNeuralNetData(tableData,mode)
% Creates X (predictors) and Y (response), as well as outputs number of
% classes, from a RoboCupRescue data table

X = tableData{:,{'sTime','sDist','sHP','sDamage'}}';

Yclass = [];    % Only used in mode 2

switch mode
    case 1    % Binary classifier (alive or dead)
        Y = (tableData.eHP > 0)';
    case 2    % Multi-class classifier
        hp_bins = [0 1 3000 7000 10000];
        % bin_names = {'Dead', 'Critical', 'Injured', 'Stable'};
        Yclass = discretize(tableData.eHP', hp_bins);
        Y = zeros(numel(hp_bins),numel(tableData.eHP));
        for idx = 1:numel(hp_bins)
           Y(idx,Yclass==idx) = 1; 
        end
    case 3    % Regression (HP prediction)
        Y = tableData.eHP';
end

end