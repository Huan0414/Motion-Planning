function [Aeq, beq] = getAbeq(n_seg, n_order, ts, start_cond, end_cond)
    n_all_poly = n_seg*(n_order+1);
    %#####################################################
    % STEP 2.1 p,v,a constraint in start 
    Aeq_start = zeros(3, n_all_poly);
    s = ts(1);
    Aeq_start(1, n_order+1) = 1*s;
    Aeq_start(2, n_order:n_order+1) = [1,-1]*n_order;
    Aeq_start(3, n_order-1:n_order+1) = [1,-2,1]*n_order*(n_order-1)/s;
    beq_start = start_cond';

    %#####################################################
    % STEP 2.2 p,v,a constraint in end
    Aeq_end = zeros(3, n_all_poly);
    s = ts(end);
    Aeq_end(1, end-7) = 1*s;
    Aeq_end(2, end-7:end-6) = [1,-1]*n_order;
    Aeq_end(3, end-7:end-5) = [1,-2,1]*n_order*(n_order-1)/s;
    beq_end = end_cond';

    %#####################################################
    % STEP 2.3 position continuity constrain between 2 segments
    Aeq_con_p = zeros(n_seg-1, n_all_poly);
    beq_con_p = zeros(n_seg-1,1);
    
    for seg = 1:n_seg-1
        s1 = ts(seg);
        s2 = ts(seg+1);
        Aeq_con_p(seg, [(n_order+1)*(seg-1)+1,(n_order+1)*(seg+1)]) = [1*s1,-1*s2];
    end

    %#####################################################
    % STEP 2.4 velocity continuity constrain between 2 segments
    Aeq_con_v = zeros(n_seg-1, n_all_poly);
    beq_con_v = zeros(n_seg-1,1);
    for seg = 1:n_seg-1
        Aeq_con_v(seg, [(n_order+1)*(seg-1)+1:(n_order+1)*(seg-1)+2,(n_order+1)*(seg+1)-1:(n_order+1)*(seg+1)])=[1,-1,-1,1]*n_order;
    end

    %#####################################################
    % STEP 2.5 acceleration continuity constrain between 2 segments
    Aeq_con_a = zeros(n_seg-1, n_all_poly);
    beq_con_a = zeros(n_seg-1,1);
    for seg = 1:n_seg-1
        s1 = ts(seg);
        s2 = ts(seg+1);
        Aeq_con_a(seg, [(n_order+1)*(seg-1)+1:(n_order+1)*(seg-1)+3,(n_order+1)*(seg+1)-2:(n_order+1)*(seg+1)]) = [1/s1,-2/s1,1/s1,-1/s2,2/s2,-1/s2]*n_order*(n_order-1);
    end
    
    %#####################################################
    % combine all components to form Aeq and beq   
    Aeq_con = [Aeq_con_p; Aeq_con_v; Aeq_con_a];
    beq_con = [beq_con_p; beq_con_v; beq_con_a];
    Aeq = [Aeq_start; Aeq_end; Aeq_con];
    beq = [beq_start; beq_end; beq_con];
end