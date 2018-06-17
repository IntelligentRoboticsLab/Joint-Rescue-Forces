% this is the third step of the MATLAB code of the pathplan part of the Workshop
% https://github.com/IntelligentRoboticsLab/Joint-Rescue-Forces/wiki

load ../../data/kobe_graph.mat;
from = find(G.Nodes.ID=="36373");
target = [find(G.Nodes.ID=="33976") find(G.Nodes.ID=="35378") find(G.Nodes.ID=="34146")];
[TR,D] = shortestpathtree(G,from,target,'OutputForm','cell')

