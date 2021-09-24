function [Q, M] = getQM(n_seg, n_order, ts)
    Q = [];
    M = [];
    M_k = getM(n_order);
    for k = 1:n_seg        
        % initialize Q_k
        Q_k = zeros(n_order+1);
        %#####################################################
        % STEP 1.1: calculate Q_k of the k-th segment 
        % Q_k = [  0  0  0  0  0  0  0  0  
        %                   .......    
        %          1/2coff_q4q7  1/2coff_q4q6  1/2coff_q4q5  coff_q4q4   0  0  0  0 
        %          1/22coff_q5q7 1/2coff_q5q6  coff_q5q5     1/2coff_q5q4   0  0  0  0 
        %                   .......                                          ]               
        for i = flipud(4:n_order)
            for l = flipud(4:n_order)
                % s^(3-2k), s is ts, k=4 for minimum snap
                qil = ((ts(k))^(i+l-7))*(ts(k)^(-5)*i*(i-1)*(i-2)*(i-3)*l*(l-1)*(l-2)*(l-3)/(i+l-4));
                if i == l
                    Q_k(n_order-i+1,n_order-l+1) = qil;
                else
                    Q_k(n_order-i+1,n_order-l+1) = qil/2;
                end
            end
        end
        Q = blkdiag(Q, Q_k);
        M = blkdiag(M, M_k);
    end
    Q = Q*2; % quadprog solve 1/2x^T*H*x + f*x;
end