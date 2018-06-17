% this is the fifth step of the MATLAB code of the pathplan part of the Workshop
% https://github.com/IntelligentRoboticsLab/Joint-Rescue-Forces/wiki

figure(4);
load ../../data/kobe_pgraph.mat;
PG.plot();
[depth_path,C] = PG.shortestpath(M(char("36373")),M(char("33976")),'algorithm', 'depth-first', 'timer');
[astar_path,C] = PG.shortestpath(M(char("36373")),M(char("33976")),'algorithm', 'Astar', 'timer');
[dijkstra_path,C] = PG.shortestpath(M(char("36373")),M(char("33976")),'algorithm', 'Dijkstra', 'timer');
PG.highlight_path(depth_path, 'EdgeThickness', 3, 'EdgeColor', 'cyan');
PG.highlight_path(astar_path, 'EdgeThickness', 3, 'EdgeColor', 'cyan');
PG.highlight_path(dijkstra_path, 'EdgeThickness', 3, 'EdgeColor', 'cyan');
PG.highlight_node(depth_path(end), 'NodeFaceColor', 'red');
PG.highlight_node(astar_path(end), 'NodeFaceColor', 'red');
PG.highlight_node(dijkstra_path(end), 'NodeFaceColor', 'red');
PG.highlight_node(dijkstra_path(1), 'NodeFaceColor', 'green');

