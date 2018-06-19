%% Import the building data and plot the building map
buildings = importBuildingsData('data/unsupervised/buildings.csv');
close all
figure
plot(buildings.x,buildings.y,'bo')
title('Building Map'); xlabel('X position'); ylabel('Y position');

%% Perform k-Means clustering and plot the results
numMeans = 5;
[indices,centroids] = kmeans([buildings.x buildings.y],numMeans);

figure
hold on
gscatter(buildings.x,buildings.y,indices)
plot(centroids(:,1),centroids(:,2),'kx','MarkerSize',10,'LineWidth',2)
hLegend = legend;
hLegend.String{end} = 'centroids';
title('k-Means Clustered Map'); xlabel('X position'); ylabel('Y position');

%% Perform k-Medoids clustering and plot the results
numMedoids = 5;
[indices,centroids] = kmedoids([buildings.x buildings.y],numMedoids);

figure
hold on
gscatter(buildings.x,buildings.y,indices)
plot(centroids(:,1),centroids(:,2),'kx','MarkerSize',10,'LineWidth',2)
hLegend = legend;
hLegend.String{end} = 'centroids';
title('k-Medoids Clustered Map'); xlabel('X position'); ylabel('Y position');