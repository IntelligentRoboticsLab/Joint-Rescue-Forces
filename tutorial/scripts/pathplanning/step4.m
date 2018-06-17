% this is the fourth step of the MATLAB code of the pathplan part of the Workshop
% https://github.com/IntelligentRoboticsLab/Joint-Rescue-Forces/wiki

figure(3);
load ../../data/kobe_graph.mat;
from = find(G.Nodes.ID=="36373");
target = [find(G.Nodes.ID=="33976") find(G.Nodes.ID=="35378") find(G.Nodes.ID=="34146")];
[TR,D] = shortestpathtree(G,from,target,'OutputForm','cell');
[PG,M] = graphToPGraph(G);
PG.plot();
short_path = TR{2};
medium_path = TR{1};
longest_path = TR{3};
PG.highlight_path(short_path, 'EdgeThickness', 3, 'EdgeColor', 'cyan');
PG.highlight_path(medium_path, 'EdgeThickness', 3, 'EdgeColor', 'cyan');
PG.highlight_path(longest_path, 'EdgeThickness', 3, 'EdgeColor', 'cyan');
PG.highlight_node(short_path(end), 'NodeFaceColor', 'red');
PG.highlight_node(medium_path(end), 'NodeFaceColor', 'red');
PG.highlight_node(longest_path(end), 'NodeFaceColor', 'red');
PG.highlight_node(short_path(1), 'NodeFaceColor', 'green')

