% this is the second step of the MATLAB code of the pathplan part of the Workshop
% https://github.com/IntelligentRoboticsLab/Joint-Rescue-Forces/wiki

figure(2);
load ../../data/testmap_graph.mat;
plot(G,'xdata',G.Nodes.X,'ydata',G.Nodes.Y,'NodeLabel',cellstr(G.Nodes.ID));
