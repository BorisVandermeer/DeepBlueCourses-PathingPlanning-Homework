function Q = getQ(n_seg, n_order, ts)
    % Q = zeros((n_order-3)*n_seg,(n_order-3)*n_seg);
    Q = [];
    for k = 1:n_seg
        Q_k = zeros(n_order-3,n_order-3);
        %#####################################################
        % STEP 1.1: calculate Q_k of the k-th segment 
        factor = zeros(n_order-3);
        for i = 1:n_order
            factor(i) = i*(i-1)*(i-2)*(i-3);
        end
        for i=4:n_order
            for l=4:n_order         
                Q_k(i+1,l+1) = factor(i)*factor(l)/(i+l-7);
            end
        end
        Q = blkdiag(Q, Q_k);
    end
end