i= 0;
filename = strcat(num2str(i), '.pcd');
filename = strcat('./IX/cloud',filename);
Point_Sensor=loadpcd(filename)';

Point_Sensor(:, 2) = -Point_Sensor(:, 2); 

% for i = 1 : 10 : size(Point_Sensor, 1)
%     scatter3(Point_Sensor(i , 1), Point_Sensor(i , 3), -Point_Sensor(i , 2));
% end
figure
scatter3(-Point_Sensor(: , 1), Point_Sensor(: , 3), Point_Sensor(: , 2))

%%
for i = 1:1:1
    filename = strcat(num2str(i), '.txt');
    filename = strcat('./GridTest/GridMap',filename);
    Grid=load(filename)';
    
        for i=1:1:400
            for j=1:1:400
                GridMap(i, j) = Grid(i, j) ;
            end
        end
    X=1:1:400;
    Y=1:1:400;
    figure
    mesh(X, Y, GridMap,'Facecolor','non');
end
%%
for i = 1:1:8
    filename = strcat(num2str(i), '.txt');
    filename = strcat('./GridTest/LastGridMap',filename);
    Grid=load(filename)';
    
        for i=1:1:400
            for j=1:1:400
                GridMap(i, j) = Grid(401 - i, j) ;
            end
        end
    X=1:1:400;
    Y=1:1:400;
    figure
    mesh(X, Y, GridMap,'Facecolor','non');
end

%%
a = [1, ]