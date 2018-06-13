function [PG,M] = graphToPGraph(G)
% Converts MATLAB graph to RVC Toolbox Graph

PG = PGraph;

% Add nodes to the PGraph
nNodes = size(G.Nodes,1);
for idx = 1:nNodes
   PG.add_node([G.Nodes.X(idx), G.Nodes.Y(idx)]);   % Set the XY coords
   PG.setvdata(idx,G.Nodes.ID(idx));                % Set the node ID
end

% Add edges to the PGraph
nEdges = size(G.Edges,1);
for idx = 1:nEdges
   edgeIndices = G.Edges.EndNodes(idx,:);
   PG.add_edge(edgeIndices(1),edgeIndices(2),G.Edges.Weight(idx));
end

% Create a Map
M = containers.Map(cellstr(G.Nodes.ID),1:nNodes);

end

