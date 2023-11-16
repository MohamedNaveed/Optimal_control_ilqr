function [Z] = build_info_state(z,u,q,i,nZ)

Z = zeros(nZ,1); %information state
nz = size(z,1);
nu = size(u,1);

for k = 1:q
    Z((k-1)*nz+1:k*nz,1) = z(:,i-k+1);
end

for k = 1:q-1
    Z(q*nz+(k-1)*nu+1:q*nz+k*nu,1) = u(:,i-k);
end