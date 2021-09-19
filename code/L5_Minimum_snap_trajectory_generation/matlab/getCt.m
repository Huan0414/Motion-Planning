function Ct = getCt(n_seg, n_order)
    %#####################################################
    % STEP 2.1: finish the expression of Ct
    Ct = zeros(8*n_seg, 4*n_seg+4);
    Ct(1,1) = 1;
    Ct(2,2) = 1;
    Ct(3,3) = 1;
    Ct(4,4) = 1;
    
    for k = 1:n_seg-1
        % pk, t=0
        Ct(8*k-3,4+k) = 1; 
        % vk, t=0
        Ct(8*k-2, 7+n_seg+3*(k-1)+1) = 1;
        % ak, t=0
        Ct(8*k-1, 7+n_seg+3*(k-1)+2) = 1;
        % jk, t=0
        Ct(8*k, 7+n_seg+3*(k-1)+3) = 1;
        
        % pk, t=T
        Ct(8*k+1, 4+k) = 1; 
        % vk, t=T
        Ct(8*k+2, 7+n_seg+3*(k-1)+1) = 1;
        % ak, t=T
        Ct(8*k+3, 7+n_seg+3*(k-1)+2) = 1;
        % jk, t=T
        Ct(8*k+4, 7+n_seg+3*(k-1)+3) = 1;
    end
    
    Ct(end-3,4+n_seg) = 1;
    Ct(end-2,5+n_seg) = 1;
    Ct(end-1,6+n_seg) = 1;
    Ct(end,7+n_seg) = 1;
    
end