function M = getM(n_seg, n_order, ts)
    M = [];
    
    for k = 1:n_seg
        M_k = zeros(4, n_order+1);
        %#####################################################
        % STEP 1.1: calculate M_k of the k-th segment 
        t = ts(k);
        pi = n_order:-1:0;
        % start of this segment
        M_k(1,1:end) = 0.^pi;
        % end of this segment
        M_k(5,1:end) = t.^pi;
        
        vi = n_order:-1:1;
        M_k(2,1:end-1) = (0.^(vi-1)).*factorial(vi)./factorial(vi-1);
        M_k(6,1:end-1) = (t.^(vi-1)).*factorial(vi)./factorial(vi-1);
        
        ai = n_order:-1:2;
        M_k(3,1:end-2) = (0.^(ai-2)).*factorial(ai)./factorial(ai-2);
        M_k(7,1:end-2) = (t.^(ai-2)).*factorial(ai)./factorial(ai-2);
        
        ji = n_order:-1:3;
        M_k(4,1:end-3) = (0.^(ji-3)).*factorial(ji)./factorial(ji-3);
        M_k(8,1:end-3) = (t.^(ji-3)).*factorial(ji)./factorial(ji-3);
        
        M = blkdiag(M, M_k);
    end 
end