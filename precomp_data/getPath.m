function short_path = getPath(from,targets)

persistent G
if isempty(G)
  load graph.mat G
  disp 'graph is loaded'
end

%from
%targets

% Compute a path for each target
nTargets = numel(targets);
target_idx = zeros(1,nTargets);
for i = 1:nTargets
   target_idx(i) = find(G.Nodes.ID==targets(i));
end
from_idx = find(G.Nodes.ID==from);
[TR,D] = shortestpathtree(G,from_idx,target_idx,'OutputForm','cell');

% Sort the path from shortest to longest
[~,I] = sort(D);
short_path_idx = TR{I(1)};
short_path = G.Nodes.ID(short_path_idx);

end

