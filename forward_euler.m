function [temp, x_out] = forward_euler(t, x, u, model)

for i =1:1
    temp = x;
    
    for j = 1:100
        
        temp = temp + cartpole_nl_ode(i, temp, u(i), model)*model.dt/100; 
    end

    x = temp;
end

x_out = x;

end