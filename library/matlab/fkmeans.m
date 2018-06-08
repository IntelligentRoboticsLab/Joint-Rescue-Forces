function [idx,C] = fkmeans(data, k, distanceMetric, maxIter)
  [idx,C] = kmeans(data, k, 'distance', distanceMetric, 'maxIter', maxIter)
end