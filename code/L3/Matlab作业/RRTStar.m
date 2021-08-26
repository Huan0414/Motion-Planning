%***************************************
%Author: Chaoqun Wang
%Date: 2019-10-15
%***************************************
%% ?????
clc
clear all; close all;
x_I=1; y_I=1;           
x_G=700; y_G=700;      
Thr=50;                 
Delta = 120;              
%% ?????
T.v(1).x = x_I;         
T.v(1).y = y_I; 
T.v(1).xPrev = x_I;     
T.v(1).yPrev = y_I;
T.v(1).dist  = 0;      % used to save the distance from start node to the current node 
T.v(1).indPrev = 0;     

Nodes = [x_I,y_I];     % only save the coordinates of nodes in the tree
%% ??????????
figure(1);
ImpRgb=imread('newmap.png');
Imp=rgb2gray(ImpRgb);
imshow(Imp)
xL=size(Imp,2);
yL=size(Imp,1);
hold on
plot(x_I, y_I, 'ro', 'MarkerSize',10, 'MarkerFaceColor','r');
plot(x_G, y_G, 'go', 'MarkerSize',10, 'MarkerFaceColor','g');
count=1;
path_min = inf;
bFind = false;
closeNode =[];
pathExist = false;
for iter = 1:3000
    updatePlot = false;
    
    x_rand=[];
    x_rand(1) = randi([0, xL],1);
    x_rand(2) = randi([0, yL],1);
    
    x_near=[];
    dis_rand = (Nodes(:,1) - x_rand(1)).*(Nodes(:,1) - x_rand(1)) ...
           + (Nodes(:,2) - x_rand(2)).*(Nodes(:,2) - x_rand(2));
    [dis_min, index] = min(dis_rand);
    x_near(1) = Nodes(index, 1);
    x_near(2) = Nodes(index, 2);
    
    x_new=[];
    dis_min = sqrt(dis_min);
    delta_x = (x_rand(1) - x_near(1)) * Delta/dis_min;
    delta_y = (x_rand(2) - x_near(2)) * Delta/dis_min;
    x_new(1) = x_near(1) + delta_x;
    x_new(2) = x_near(2) + delta_y;
    
    %check collision-free
    if ~collisionChecking(x_near,x_new,Imp) 
       continue;
    end
    count=count+1;
    plot(x_new(1), x_new(2),'ro', 'MarkerSize',3, 'MarkerFaceColor','r');
    % Find other near nodes
    x_Prev = [];
    dis_new =  (Nodes(:,1) - x_new(1)).*(Nodes(:,1) - x_new(1)) ...
           + (Nodes(:,2) - x_new(2)).*(Nodes(:,2) - x_new(2));
    nearNodesIndex = find(floor(dis_new) <= Delta*Delta);
    
    if length(nearNodesIndex) > 1         
        % check collision
        for i = 1:length(nearNodesIndex)
            nodeNear = [Nodes(nearNodesIndex(i),1), Nodes(nearNodesIndex(i),2)];
            if ~collisionChecking(nodeNear,x_new,Imp)               
                nearNodesIndex(i) = 0;
            end
        end
        nearNodesIndex(nearNodesIndex == 0) = [];
        % Select a parent with least path cost
        if length(nearNodesIndex) >= 1
            PrevIndex = nearNodesIndex(1);
            x_Prev(1) = Nodes(PrevIndex, 1);
            x_Prev(2) = Nodes(PrevIndex, 2);
            for j=1:length(nearNodesIndex)-1
                if T.v(PrevIndex).dist + sqrt(dis_new(PrevIndex)) > T.v(j+1).dist + sqrt(dis_new(j+1))                                                       
                    PrevIndex = nearNodesIndex(j+1);
                    x_Prev(1) = Nodes(PrevIndex, 1);
                    x_Prev(2) = Nodes(PrevIndex, 2);
                end
            end     
        end
    else % take x_near as the parent node
        x_Prev(1) = x_near(1);
        x_Prev(2) = x_near(2);
        PrevIndex = index; 
    end
       
    % Add the new node into the tree 
    T.v(count).x = x_new(1);         
    T.v(count).y = x_new(2); 
    T.v(count).xPrev = x_Prev(1);   
    T.v(count).yPrev = x_Prev(2);
    T.v(count).dist  = T.v(PrevIndex).dist + sqrt(dis_new(PrevIndex));          
    T.v(count).indPrev = PrevIndex;    
    Nodes = [Nodes; x_new(1), x_new(2)];
    plot([x_Prev(1),x_new(1)],[x_Prev(2),x_new(2)], 'r','LineWidth',1);
    
    % Rewire
    if length(nearNodesIndex) > 1
        for k = 1:length(nearNodesIndex)
            if T.v(nearNodesIndex(k)).dist > T.v(count).dist + sqrt(dis_new(nearNodesIndex(k)))                                           
               % remove old edge
               plot([T.v(nearNodesIndex(k)).x,T.v(nearNodesIndex(k)).xPrev],[T.v(nearNodesIndex(k)).y,T.v(nearNodesIndex(k)).yPrev], 'w','LineWidth',1);
               % change the parent to the new node
               T.v(nearNodesIndex(k)).xPrev = x_new(1);
               T.v(nearNodesIndex(k)).yPrev = x_new(2);
               T.v(nearNodesIndex(k)).dist  = T.v(count).dist + sqrt(dis_new(nearNodesIndex(k)));
               T.v(nearNodesIndex(k)).indPrev = count;
               % plot new edge
               plot([T.v(nearNodesIndex(k)).x,x_new(1)],[T.v(nearNodesIndex(k)).y,x_new(2)], 'g','LineWidth',1);
             end
        end
    end
    
    % update the path if a shorter path is found
    if (abs(pdist([x_new(1),x_new(2);x_G,y_G])) <= Thr)
        bFind = true;
        if (T.v(count).dist + abs(pdist([x_new(1),x_new(2);x_G,y_G])) < path_min) 
            path_min = T.v(count).dist + abs(pdist([x_new(1),x_new(2);x_G,y_G]));
            closeNode(1) = x_new(1);
            closeNode(2) = x_new(2);
            closeNodeIndex = PrevIndex;
            updatePlot = true;
        end
    end
    
    % regularly update the path
    if mod(iter,10) == 0
        updatePlot = true;
    end
    
    if bFind && updatePlot
        if pathExist
            for j = 2:length(path.pos)
                plot([path.pos(j).x; path.pos(j-1).x;], [path.pos(j).y; path.pos(j-1).y], 'w', 'Linewidth', 2);
            end
            path.pos = [];
        end
            path.pos(1).x = x_G; path.pos(1).y = y_G;
%             path.pos(2).x = T.v(end).x; path.pos(2).y = T.v(end).y;
            path.pos(2).x = closeNode(1); path.pos(2).y = closeNode(2);
%             pathIndex = T.v(end).indPrev;
            pathIndex = closeNodeIndex;
            j=0;
            while 1
                path.pos(j+3).x = T.v(pathIndex).x;
                path.pos(j+3).y = T.v(pathIndex).y;
                pathIndex = T.v(pathIndex).indPrev;
                if pathIndex == 1
                    break
                end
                j=j+1;
            end 
            path.pos(end+1).x = x_I; path.pos(end).y = y_I; 
            for j = 2:length(path.pos)
                plot([path.pos(j).x; path.pos(j-1).x;], [path.pos(j).y; path.pos(j-1).y], 'b', 'Linewidth', 2);
            end
            pathExist = true;
    end
    pause(0.02); 
end
