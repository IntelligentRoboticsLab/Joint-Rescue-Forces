% this is the first step of the MATLAB code of the pathplan part of the Workshop
% https://github.com/IntelligentRoboticsLab/Joint-Rescue-Forces/wiki

figure(1);
load ../../data/testmap_graph.mat;
plot(G,'NodeLabel',cellstr(G.Nodes.ID));
