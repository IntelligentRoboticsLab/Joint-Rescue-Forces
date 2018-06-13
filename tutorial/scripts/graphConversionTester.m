filename = '../data/kobe_graph';
load(filename)

PG = graphToPGraph(G);

figure(1) 
plot(G,'xdata',G.Nodes.X,'ydata',G.Nodes.Y); %, ... 
       %'NodeLabel',cellstr(G.Nodes.ID));
title('MATLAB Graph');

figure(2)
plot(PG,'NodeSize',3);
title('RVC Toolbox PGraph');
