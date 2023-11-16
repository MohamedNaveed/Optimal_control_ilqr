function [V] = build_data_mat_ltv(U, y_mat, q, nu, nz, k, no_rollouts)


if k > q
    V = zeros((nz + nu)*q, no_rollouts);

    %V(1:nu,:) = U((k-1)*nu +1: k*nu,:);

    for i = 1:q
        %time_idxs = (k - i - 1)*nz + 1: (k-i)*nz;
        V((i-1)*nz + 1: i*nz, :) = y_mat((k - 1 - i)*nz + 1: (k-i)*nz, :);

        V(q*nz + (i-1)*nu + 1: q*nz + i*nu, :) = U((k - 1 - i)*nu +1: (k-i)*nu,:);
    end
    
else
    disp('More observations need to build information-state')

end
end



