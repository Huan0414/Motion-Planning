function [ OPEN, OPEN_COUNT ] = update_openlist(exp_array, OPEN, xParent,yParent)
    % This function is to take the expanded nodes array,
    % check whether they are in the open list and also update all the costs if
    % necessary. 
    
    %EXPANDED ARRAY FORMAT
    %--------------------------------
    %|X val |Y val ||h(n) |g(n)|f(n)|
    %--------------------------------
    
    %OPEN LIST FORMAT
    %--------------------------------------------------------------------------
    %IS ON LIST 1/0 |X val |Y val |Parent X val |Parent Y val |h(n) |g(n)|f(n)|
    %-------------------------------------------------------------------------
    
    EXP_COUNT = length(exp_array(:,1));
    OPEN_COUNT = length(OPEN(:,1));
    for c1 = 1:EXP_COUNT
        % find nodes with the same x and y coordinates
        index = find((exp_array(c1,1)==OPEN(:,2)) & (exp_array(c1,2)== OPEN(:,3)));
        
        % this expanded node is within the OPEN list
        if ~isempty(index)
            % gm > gn + Cnm, then update all the costs and parent node
            if OPEN(index,7) >  exp_array(c1, 4)
                OPEN(index,4) = xParent;
                OPEN(index,5) = yParent;
                OPEN(index,6) = exp_array(c1, 3);
                OPEN(index,7) = exp_array(c1, 4);
                OPEN(index,8) = exp_array(c1, 5);
            end
        else
            % this expanded node is not within the OPEN list, put it inside
            OPEN_COUNT = OPEN_COUNT + 1;
            OPEN(OPEN_COUNT, :) = insert_open(exp_array(c1,1),exp_array(c1,2),xParent,yParent,exp_array(c1,3),exp_array(c1,4),exp_array(c1,5));
        end
                
    end

    
    
    




end

