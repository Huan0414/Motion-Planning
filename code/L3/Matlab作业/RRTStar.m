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
rng(5);
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
lastNode =[];
pathExist = false;

for iter = 1:3000
    updatePlot = false;
    
    x_rand=[];
    x_rand(1) = randi([0, xL],1);
    x_rand(2) = randi([0, yL],1);
    
    x_near=[];
    % distance with the rand node
    dis_rand = (Nodes(:,1) - x_rand(1)).*(Nodes(:,1) - x_rand(1)) ...
           + (Nodes(:,2) - x_rand(2)).*(Nodes(:,2) - x_rand(2));
    [dis_min, index_min] = min(dis_rand);
    x_near(1) = Nodes(index_min, 1);
    x_near(2) = Nodes(index_min, 2);
    
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
    
    % plot the new node
    plot(x_new(1), x_new(2), 'ro', 'MarkerSize',3, 'MarkerFaceColor','r');
    
    % Find other near nodes and parent node
    x_Prev = [];
    dis_near =  (Nodes(:,1) - x_new(1)).*(Nodes(:,1) - x_new(1)) ...
           + (Nodes(:,2) - x_new(2)).*(Nodes(:,2) - x_new(2));
    
    % All nodes within the range
    nearNodesIndex = find(floor(dis_near) <= Delta*Delta);
    
    if length(nearNodesIndex) > 1         
        % check collision of all near nodes
        for i = 1:length(nearNodesIndex)
            nodeNear = [Nodes(nearNodesIndex(i),1), Nodes(nearNodesIndex(i),2)];
            if ~collisionChecking(nodeNear,x_new,Imp)               
                nearNodesIndex(i) = 0;
            end
        end
        % remove the nodes lead to collision
        nearNodesIndex(nearNodesIndex == 0) = [];
        % Select a parent node with the least path cost
        if length(nearNodesIndex) >= 1
            PrevIndex = nearNodesIndex(1);
            x_Prev(1) = Nodes(PrevIndex, 1);
            x_Prev(2) = Nodes(PrevIndex, 2);
            for j=1:length(nearNodesIndex)-1
                if T.v(PrevIndex).dist + sqrt(dis_near(PrevIndex)) > T.v(j+1).dist + sqrt(dis_near(nearNodesIndex(j+1)))                                                       
                    PrevIndex = nearNodesIndex(j+1);
                    x_Prev(1) = Nodes(PrevIndex, 1);
                    x_Prev(2) = Nodes(PrevIndex, 2);
                end
            end     
        end
    else % take x_near as the parent node
        x_Prev(1) = x_near(1);
        x_Prev(2) = x_near(2);
        PrevIndex = index_min; 
    end
       
    % Add the new node into the tree 
    count=count+1;
    T.v(count).x = x_new(1);         
    T.v(count).y = x_new(2); 
    T.v(count).xPrev = x_Prev(1);   
    T.v(count).yPrev = x_Prev(2);
    T.v(count).dist  = T.v(PrevIndex).dist + sqrt(dis_near(PrevIndex));          
    T.v(count).indPrev = PrevIndex;    
    Nodes = [Nodes; x_new(1), x_new(2)];
    plot([x_Prev(1),x_new(1)],[x_Prev(2),x_new(2)], 'r','LineWidth',1);
    
    % Rewire
    if length(nearNodesIndex) > 1
        for k = 1:length(nearNodesIndex)
            if T.v(nearNodesIndex(k)).dist > T.v(count).dist + sqrt(dis_near(nearNodesIndex(k)))                                           
               % remove old edge on the map
               plot([T.v(nearNodesIndex(k)).x,T.v(nearNodesIndex(k)).xPrev],[T.v(nearNodesIndex(k)).y,T.v(nearNodesIndex(k)).yPrev], 'w','LineWidth',1);
               % change the parent to the new node
               T.v(nearNodesIndex(k)).xPrev = x_new(1);
               T.v(nearNodesIndex(k)).yPrev = x_new(2);
               T.v(nearNodesIndex(k)).dist  = T.v(count).dist + sqrt(dis_near(nearNodesIndex(k)));
               T.v(nearNodesIndex(k)).indPrev = count;
%                T = update_cost_forward(T, nearNodesIndex(k),x_G,y_G,Thr);
               nodeIndex = nearNodesIndex(k);
               for idx = 1:length(T)
                   forwardIndex = [];
                   if T.v(idx).indPrev == nodeIndex
                        forwardIndex = [forwardIndex;idx];
                   end
               end
               while(~isempty(forwardIndex))
                    T.v(forwardIndex).dist = T.v(nodeIndex).dist + pdist([T.v(nodeIndex).x, T.v(nodeIndex).y;T.v(forwardIndex).x, T.v(forwardIndex).y]);
                    if pdist([T.v(forwardIndex).x, T.v(forwardIndex).y;x_G,y_G]) <= Thr
                        break;
                    end
                    nodeIndex = forwardIndex;
                    forwardIndex = T.v(nodeIndex).indPrev;
               end
               % plot new edge
               plot([T.v(nearNodesIndex(k)).x,x_new(1)],[T.v(nearNodesIndex(k)).y,x_new(2)], 'g','LineWidth',1);
             end
        end
    end
    
    % update the path if a shorter path is found
    if (pdist([x_new(1),x_new(2);x_G,y_G]) <= Thr) && bFind == false
        bFind = true;
%         if (T.v(count).dist + abs(pdist([x_new(1),x_new(2);x_G,y_G])) < path_min) 
             lastNode(1) = x_new(1);
             lastNode(2) = x_new(2);
             lastNodePrevIndex = PrevIndex;
             lastNodeIndex = count;
%             updatePlot = true;
%         end
    end
    
    % regularly update the path
    if mod(iter,20) == 0
        updatePlot = true;
    end
    
    if bFind && updatePlot
        if pathExist
            for j = 2:length(path.pos)
                plot([path.pos(j).x; path.pos(j-1).x;], [path.pos(j).y; path.pos(j-1).y], 'w', 'Linewidth', 2);
            end
            path.pos = [];
            pathIndexVector = [];
        end
            path.pos(1).x = x_G; path.pos(1).y = y_G;pathIndexVector(1) = 0;
%             path.pos(2).x = T.v(end).x; path.pos(2).y = T.v(end).y;
            path.pos(2).x = lastNode(1); path.pos(2).y = lastNode(2);
%             pathIndex = T.v(end).indPrev;
            pathIndex = lastNodePrevIndex;
            pathIndexVector(2) = pathIndex;
            j=0;
            path_cost = pdist([T.v(lastNodeIndex).x, T.v(lastNodeIndex).y;x_G,y_G]);
            while 1
                path.pos(j+3).x = T.v(pathIndex).x;
                path.pos(j+3).y = T.v(pathIndex).y;               
                pathIndex = T.v(pathIndex).indPrev;
                path_cost = path_cost + pdist([path.pos(j+3).x, path.pos(j+3).y; T.v(pathIndex).x, T.v(pathIndex).y]);
                pathIndexVector(j+3) = pathIndex;
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
            path_min = path_cost;
            % print the path cost
            fprintf("Path cost: %.2f \n", path_cost);
            pause(0.02); 
    end

end
% 
% function T = update_cost_forward(T, nodeIndex, x_G, y_G, Thr)
% 
%     while(1)
%         T.v(prevIndex).dist = T.v(nodeIndex).dist + pdist([T.v(nodeIndex).x, T.v(nodeIndex).y;T.v(prevIndex).x, T.v(prevIndex).y]);
%         if pdist([T.v(prevIndex).x, T.v(prevIndex).y;x_G,y_G]) <= Thr
%             break;
%         end
%         nodeIndex = prevIndex;
%         prevIndex = T.v(nodeIndex).indPrev;
%     end
%         
% end


