
c_obs_1 = [7.5; 0; 0; 0];
c_obs_2 = c_obs_1;

%lane change values.
%c_obs_1 = [2.25; 3; 0; 0]; % Ellipse center
%c_obs_2 = [17.75; 3; 0; 0];


E_obs_1 = [(1/2.5)^2, 0, 0, 0; 
           0, 1, 0, 0;
           0, 0, 0, 0;
           0, 0, 0, 0]; % Parameters of the ellipse
E_obs_2 = E_obs_1; 

M = 1; % Obstacle penalty factor