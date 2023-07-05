function [A,B] = cos1d_A_B(model, X_bar, U_bar)


A = 1 + sin(X_bar)*model.dt;
B = model.dt;
end

