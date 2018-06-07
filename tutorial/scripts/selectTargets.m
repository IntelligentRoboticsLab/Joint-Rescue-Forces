function predictions = selectTargets(time,dist,hp,damage)

persistent targetSelectorModel
if isempty(targetSelectorModel)
   load targetSelectorModel targetSelectorModel
end

predictors = table(time,dist,hp,damage, ...
     'VariableNames',{'sTime','sDist','sHP','sDamage'});

predictions = int32(targetSelectorModel.predictFcn(predictors));

end

