function [path, OPEN] = A_star_search(map,MAX_X,MAX_Y, hn_option)
%%
%This part is about map/obstacle/and other settings
    %pre-process the grid map, add offset
    size_map = size(map,1);
    Y_offset = 0;
    X_offset = 0;
    
    %Define the 2D grid map array.
    %Obstacle=-1, Target = 0, Start=1
    MAP=2*(ones(MAX_X,MAX_Y));
    
    %Initialize MAP with location of the target
    xval=floor(map(size_map, 1)) + X_offset;
    yval=floor(map(size_map, 2)) + Y_offset;
    xTarget=xval;
    yTarget=yval;
    MAP(xval,yval)=0;
    
    %Initialize MAP with location of the obstacle
    for i = 2: size_map-1
        xval=floor(map(i, 1)) + X_offset;
        yval=floor(map(i, 2)) + Y_offset;
        MAP(xval,yval)=-1;
    end 
    
    %Initialize MAP with location of the start point
    xval=floor(map(1, 1)) + X_offset;
    yval=floor(map(1, 2)) + Y_offset;
    xStart=xval;
    yStart=yval;
    MAP(xval,yval)=1;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %LISTS USED FOR ALGORITHM
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %OPEN LIST STRUCTURE
    %--------------------------------------------------------------------------
    %IS ON LIST 1/0 |X val |Y val |Parent X val |Parent Y val |h(n) |g(n)|f(n)|
    %--------------------------------------------------------------------------
    OPEN=[];
    %CLOSED LIST STRUCTURE
    %--------------
    %X val | Y val |
    %--------------
    % CLOSED=zeros(MAX_VAL,2);
    CLOSED=[];

    %Put all obstacles on the Closed list
    k=1;%Dummy counter
    for i=1:MAX_X
        for j=1:MAX_Y
            if(MAP(i,j) == -1)
                CLOSED(k,1)=i;
                CLOSED(k,2)=j;
                k=k+1;
            end
        end
    end
    CLOSED_COUNT=size(CLOSED,1);
    %set the starting node as the first node
    xNode=xval;
    yNode=yval;
    OPEN_COUNT=1;
    goal_distance=distance(xNode,yNode,xTarget,yTarget);
    path_cost=0;
    OPEN(OPEN_COUNT,:)=insert_open(xNode,yNode,xNode,yNode,goal_distance,path_cost,goal_distance);
    OPEN(OPEN_COUNT,1)=1;
    CLOSED_COUNT=CLOSED_COUNT+1;
    CLOSED(CLOSED_COUNT,1)=xNode;
    CLOSED(CLOSED_COUNT,2)=yNode;

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% START ALGORITHM
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    while(1)
        % Remove node with the lowest cost f in the open list and put it
        % into the close list
        EXPAND_INDEX = min_fn(OPEN,OPEN_COUNT,xTarget,yTarget); % find the lowest cost node
        if EXPAND_INDEX == -1, fprintf(' ==> No path found \n'); break, end 
        OPEN(EXPAND_INDEX,1) = 0; %remove the expanded node
        xParent = OPEN(EXPAND_INDEX, 2);
        yParent = OPEN(EXPAND_INDEX, 3);
        gParent = OPEN(EXPAND_INDEX, 7);
        
        if (xParent == xTarget) && (yParent == yTarget)
        % If the expanded node is the path, then break loop and show path
            fprintf(" ==> Find the path! %s as heuristic! \n", hn_option);
            fprintf(" ==> Path length is %.2f \n", gParent);
            break;
        else
        % If the expanded node is not the goal, then put neighbors into
        % the OPEN list
            exp_array = expand_array(xParent,yParent,gParent,xTarget,yTarget,CLOSED,MAX_X, MAX_Y, hn_option);
            % If no expanded nodes in the neighbor, then take another expanded
            % nodes (this might happen when the obstacle ratio is high)
            if isempty(exp_array), continue; end
            % Update the OPEN list
            [OPEN, OPEN_COUNT] = update_openlist(exp_array, OPEN, xParent,yParent);
        end
        
    end %End of While Loop
   
   nodes = [];
   nodes(1,1) = xTarget;
   nodes(1,2) = yTarget;
   
   count = 1;
   xNodeBack = xTarget;
   yNodeBack = yTarget;
   while(1)
        % find the parent node place
        c = find((OPEN(:, 2) == xNodeBack) & (OPEN(:, 3) == yNodeBack));
        xNodeBack = OPEN(c, 4);
        yNodeBack = OPEN(c, 5);
        count = count + 1;
        nodes(count, 1) = xNodeBack;
        nodes(count, 2) = yNodeBack;       
        if (xNodeBack == xStart) && (yNodeBack == yStart),break; end
   end
    
   path = flipud(nodes);
end
