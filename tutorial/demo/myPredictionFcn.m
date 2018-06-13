function response = myPrediction(classifier,predictors)
% Function that predicts given the classifier provided

    response = logical(predict(classifier.ClassificationKNN,predictors));

end