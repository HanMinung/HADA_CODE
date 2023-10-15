clear

pt = readmatrix('../visualization.csv');

X = pt(:,1);
Y = pt(:,2);
Z = pt(:,3);

plot3(X,Y,Z, 'bo');
grid minor;


