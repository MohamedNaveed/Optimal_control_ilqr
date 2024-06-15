function [lx] = cal_lx_lxx(model, x)
  



%{
    pert = 0.1; %perturbation std dev.
    N = 10; % number of samples
    delta_x = normrnd(0,pert,[model.nx,N]);
    lx_sum = zeros(model.nx, 1);
    lxx_sum = zeros(model.nx, 1);

    for ns = 1:N
        
        % Check for zeros in delta_x
        while any(delta_x(:, ns) == 0)
            delta_x(:, ns) = normrnd(0,pert,[model.nx,1]);
        end
        
        latx = model.l(x);
        latx_pos = model.l(x + delta_x(:,ns));
        latx_neg = model.l(x - delta_x(:,ns));

        lx = (latx_pos - latx_neg)./delta_x(:, ns);%Central Diff
        lxx = (latx_pos - 2*latx + latx_neg)./(delta_x(:, ns).^2);%

        lx_sum = lx_sum + lx;
        lxx_sum = lxx_sum + lxx;
    end

    lx_avg = lx_sum / valid_samples;
    lxx_avg = lxx_sum / valid_samples;

    %}
end