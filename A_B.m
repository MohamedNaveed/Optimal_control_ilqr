function [A,B] = A_B(model, X_bar, U_bar)


[A, B] = LLS_CD(model, X_bar, U_bar);

end