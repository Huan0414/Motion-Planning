function [Aeq beq]= getAbeq(n_seg, n_order, waypoints, ts, start_cond, end_cond)
    
    n_all_poly = n_seg*(n_order+1);
    %#####################################################
    % p,v,a,j constraint in start, 
    Aeq_start = zeros(4, n_all_poly);
    beq_start = zeros(4, 1);
    % STEP 2.1: write expression of Aeq_start and beq_start
    
    % position, k = 0 (derivative order)
    Aeq_start_p = zeros(1,n_order+1);
    ip = n_order:-1:0;
    Aeq_start_p(1:end) = (0.^(ip-0)).*factorial(ip)./factorial(ip-0);
    
    % velocity, k = 1
    Aeq_start_v = zeros(1,n_order+1);
    iv = n_order:-1:1;
    Aeq_start_v(1:end-1) = (0.^(iv-1)).*factorial(iv)./factorial(iv-1);

    % acceleration, k = 2
    Aeq_start_a = zeros(1,n_order+1);
    ia = n_order:-1:2;
    Aeq_start_a(1:end-2) = (0.^(ia-2)).*factorial(ia)./factorial(ia-2);
    
    % jerk, k = 3
    Aeq_start_j = zeros(1,n_order+1);
    ij = n_order:-1:3;
    Aeq_start_j(1:end-3) = (0.^(ij-3)).*factorial(ij)./factorial(ij-3);
   
    Aeq_start(:, 1:n_order+1) = [Aeq_start_p;Aeq_start_v;Aeq_start_a;Aeq_start_j];
    beq_start =  start_cond';
    
    %#####################################################
    % p,v,a constraint in end
    Aeq_end = zeros(4, n_all_poly);
    beq_end = zeros(4, 1);
    % STEP 2.2: write expression of Aeq_end and beq_end
    % position, k = 0 (derivative order)
    Aeq_end_p = zeros(1,n_order+1);
    Aeq_end_p(1:end) = ((ts(end)).^(ip-0)).*factorial(ip)./factorial(ip-0);
    
    % velocity, k = 1
    Aeq_end_v = zeros(1,n_order+1);
    Aeq_end_v(1:end-1) = ((ts(end)).^(iv-1)).*factorial(iv)./factorial(iv-1);

    % acceleration, k = 2
    Aeq_end_a = zeros(1,n_order+1);
    Aeq_end_a(1:end-2) = ((ts(end)).^(ia-2)).*factorial(ia)./factorial(ia-2);
   
    Aeq_end(1:3, end-n_order:end) = [Aeq_end_p;Aeq_end_v;Aeq_end_a];
    beq_end = end_cond';
    
    %#####################################################
    % position constraint in all middle waypoints
    Aeq_wp = zeros(n_seg-1, n_all_poly);
    beq_wp = zeros(n_seg-1, 1);
    % STEP 2.3: write expression of Aeq_wp and beq_wp
    % position, k = 0 (derivative order)
    % copy the Aeq_wp_p in certain shape n_seg-1 * n_seg
    for row = 1:n_seg-1
        Aeq_wp_p = zeros(1,n_order+1);
        Aeq_wp_p(1:end) = ((ts(row)).^(ip-0)).*factorial(ip)./factorial(ip-0);
        Aeq_wp(row, (row-1)*(n_order+1)+1:row*(n_order+1)) = Aeq_wp_p;
    end
    beq_wp = waypoints(2:end-1);
    
    %#####################################################
    % position continuity constraint between each 2 segments
    Aeq_con_p = zeros(n_seg-1, n_all_poly);
    beq_con_p = zeros(n_seg-1, 1);
    % STEP 2.4: write expression of Aeq_con_p and beq_con_p
    for row = 1:n_seg-1
        Aeq_con_p_first = zeros(1, n_order+1);
        Aeq_con_p_next  = zeros(1, n_order+1);
        Aeq_con_p_first(1:end) = ((ts(row)).^(ip-0)).*factorial(ip)./factorial(ip-0);
        Aeq_con_p_next(1:end)  = -(0.^(ip-0)).*factorial(ip)./factorial(ip-0);
            beq_con_p_unit = [Aeq_con_p_first Aeq_con_p_next];
        Aeq_con_p(row, (row-1)*(n_order+1)+1:(row+1)*(n_order+1)) = beq_con_p_unit;
    end

    %#####################################################
    % velocity continuity constraint between each 2 segments
    Aeq_con_v = zeros(n_seg-1, n_all_poly);
    beq_con_v = zeros(n_seg-1, 1);
    % STEP 2.5: write expression of Aeq_con_v and beq_con_v
    for row = 1:n_seg-1
        Aeq_con_v_first = zeros(1, n_order+1);
        Aeq_con_v_next  = zeros(1, n_order+1);
        Aeq_con_v_first(1:end-1) = ((ts(row)).^(iv-1)).*factorial(iv)./factorial(iv-1);
        Aeq_con_v_next(1:end-1)  = -(0.^(iv-1)).*factorial(iv)./factorial(iv-1);
        beq_con_v_unit = [Aeq_con_v_first Aeq_con_v_next];
        Aeq_con_v(row, (row-1)*(n_order+1)+1:(row+1)*(n_order+1)) = beq_con_v_unit;
    end
    

    %#####################################################
    % acceleration continuity constraint between each 2 segments
    Aeq_con_a = zeros(n_seg-1, n_all_poly);
    beq_con_a = zeros(n_seg-1, 1);
    % STEP 2.6: write expression of Aeq_con_a and beq_con_a
    for row = 1:n_seg-1
        Aeq_con_a_first = zeros(1, n_order+1);
        Aeq_con_a_next  = zeros(1, n_order+1);
        Aeq_con_a_first(1:end-2) = ((ts(row)).^(ia-2)).*factorial(ia)./factorial(ia-2);
        Aeq_con_a_next(1:end-2)  = -(0.^(ia-2)).*factorial(ia)./factorial(ia-2);
        beq_con_a_unit = [Aeq_con_a_first Aeq_con_a_next];
        Aeq_con_a(row, (row-1)*(n_order+1)+1:(row+1)*(n_order+1)) = beq_con_a_unit;
    end
    
    
    %#####################################################
    % jerk continuity constraint between each 2 segments
    Aeq_con_j = zeros(n_seg-1, n_all_poly);
    beq_con_j = zeros(n_seg-1, 1);
    % STEP 2.7: write expression of Aeq_con_j and beq_con_j´
    for row = 1:n_seg-1
        Aeq_con_j_first = zeros(1, n_order+1);
        Aeq_con_j_next = zeros(1, n_order+1);
        Aeq_con_j_first(1:end-3) = ((ts(row)).^(ij-3)).*factorial(ij)./factorial(ij-3);
        Aeq_con_j_next(1:end-3) = -(0.^(ij-3)).*factorial(ij)./factorial(ij-3);
        beq_con_j_unit = [Aeq_con_j_first Aeq_con_j_next];
        Aeq_con_j(row, (row-1)*(n_order+1)+1:(row+1)*(n_order+1)) = beq_con_j_unit;
    end
    
    
    %#####################################################
    % combine all components to form Aeq and beq   
    Aeq_con = [Aeq_con_p; Aeq_con_v; Aeq_con_a; Aeq_con_j];
    beq_con = [beq_con_p; beq_con_v; beq_con_a; beq_con_j];
    Aeq = [Aeq_start; Aeq_end; Aeq_wp; Aeq_con];
    beq = [beq_start; beq_end; beq_wp; beq_con];
end