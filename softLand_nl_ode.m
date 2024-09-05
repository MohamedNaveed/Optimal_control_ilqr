function out = softLand_nl_ode(t,x,u,model)

psi = x(1);
theta = x(2);
phi = x(3);

w1 = x(4);
w2 = x(5);
w3 = x(6);

r1 = x(7);
r2 = x(8);
r3 = x(9);

v1 = x(10);
v2 = x(11);
v3 = x(12); 

out = zeros(12,1);

R = [                               cos(psi)*cos(theta),                                cos(theta)*sin(psi),          sin(theta);...
- cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta),   cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta), cos(theta)*sin(phi);...
  sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta), - cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta), cos(phi)*cos(theta)];

M = [0, sin(phi), cos(phi);...
     0, cos(theta)*cos(phi), -cos(theta)*sin(phi);...
     cos(theta), sin(theta)*sin(phi), sin(theta)*cos(phi)];

%F = model.d1*u(1) + model.d2*u(2) + model.d3*u(3) + model.d4*u(4);
%F = R'*F;

%torque = cross(model.r1,model.d1*u(1)) + cross(model.r2,model.d2*u(2)) +...
%    cross(model.r3,model.d3*u(3)) + cross(model.r4,model.d4*u(4));

torque = [u(1);u(2);u(3)];
F = [u(4);u(5);u(6)];
omega = [w1;w2;w3];

out(1:3) = (1/cos(theta))*M*omega;
out(4:6) = -inv(model.J)*cross(omega,model.J*omega) + inv(model.J)*torque;
out(7:9) = [v1*1000;v2*1000;v3*1000]/10000;
out(10)  = F(1)/2e6;
out(11)  = F(2)/2e6;
out(12)  = F(3)/2e6 - 3.7114/1000;

% out(12)  = F(3)/2e6 -(3.7114-3.7114*(tanh(10000*-r3)))/2e3;
end