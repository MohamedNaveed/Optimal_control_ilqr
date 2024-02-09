function [x_out] = forward_euler(t, x, u, model)


for j = 1:100
    
    x = x + model.nl_ode(t, x, u, model)*model.dt/100; 
end

x_out = x;

end