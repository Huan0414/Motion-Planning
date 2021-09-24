function [Aieq, bieq] = getAbieq(n_seg, n_order, corridor_range, ts, v_max, a_max)
    n_all_poly = n_seg*(n_order+1);
    
    %#####################################################
    % STEP 3.2.1 p constraint
    bieq_p = [];
    bieq_p(1:n_all_poly) = corridor_range(:,2); % max range
    bieq_p(n_all_poly+1:2*n_all_poly) = -corridor_range(:,1);% min range
    bieq_p = bieq_p';
    
    Aieq_p = [];
    e = ones(n_order+1,1);
    Aieq_p_k = full(spdiags(e,0,n_order+1,n_order+1));
    for seg = 1:n_seg
        Aieq_p = blkdiag(Aieq_p, Aieq_p_k);
    end
    sMatrix = repmat(ts, [n_order+1,n_all_poly]);
    s = reshape(sMatrix, [n_all_poly, n_all_poly]);
    Aieq_p = Aieq_p.*s;
    Aieq_p = [Aieq_p;-Aieq_p];


    %#####################################################
    % STEP 3.2.2 v constraint   
    bieq_v = v_max.*ones(2*n_all_poly,1);

    Aieq_v = [];
    Aieq_v_k = full(spdiags([e -e],-1:0,n_order+1,n_order+1));
    Aieq_v_k(1,:) = 0;
    for seg = 1:n_seg
        Aieq_v = blkdiag(Aieq_v, Aieq_v_k);
    end
    Aieq_v = n_order*Aieq_v;
    Aieq_v = [Aieq_v;-Aieq_v];
    
    %#####################################################
    % STEP 3.2.3 a constraint 
    bieq_a = a_max.*ones(2*n_all_poly,1);
    
    Aieq_a = [];
    Aieq_a_k = full(spdiags([e -2*e e],-2:0,n_order+1,n_order+1));
    Aieq_a_k(1:2,:) = 0;
    for seg = 1:n_seg
        Aieq_a = blkdiag(Aieq_a, Aieq_a_k);
    end
    Aieq_a = Aieq_a*n_order*(n_order-1)./s;
    Aieq_a = [Aieq_a;-Aieq_a];
    
    %#####################################################
    % combine all components to form Aieq and bieq   
    Aieq = [Aieq_p; Aieq_v; Aieq_a];
    bieq = [bieq_p; bieq_v; bieq_a];

end