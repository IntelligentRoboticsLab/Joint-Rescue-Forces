% this is the sixth step of the MATLAB code of the pathplan part of the Workshop
% https://github.com/IntelligentRoboticsLab/Joint-Rescue-Forces/wiki

figure(5);
load ../../data/kobe_pgraph.mat;
[original_path,C] = PG.shortestpath(M(char("36373")),M(char("33976")),'algorithm', 'Astar', 'timer');
neighbours = PG.neighbours(909)
edges = PG.edges(909)
old_cost = PG.cost(edges)
new_cost = 4 .* PG.cost(edges)
PG.setcost(edges,new_cost);
PG.plot();
PG.highlight_edge(edges(1), 'NodeFaceColor', 'black', 'EdgeThickness', 90);

[astar_path,C] = PG.shortestpath(M(char("36373")),M(char("33976")),'algorithm', 'Astar', 'timer');
PG.highlight_path(original_path, 'EdgeThickness', 3, 'EdgeColor', 'cyan');
PG.highlight_path(astar_path, 'EdgeThickness', 3, 'EdgeColor', 'red');
PG.highlight_node(astar_path(end), 'NodeFaceColor', 'red');
PG.highlight_node(astar_path(1), 'NodeFaceColor', 'green');

