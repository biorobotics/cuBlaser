function [] = numerical_diff(dim)
%NUMERICAL_DIFF Summary of this function goes here
%   Detailed explanation goes here
axang = [0.6, 0.0, 0.8, 0.5];
phi = axang(1:3) * axang(4);
p2 = [1; 2; 2.3];
p1 = [1.6344; 1.7954; 1.3992];
trans = [.1; .2; .35];
normal = [0.3; 0.2; 1.0];
normal = normal / norm(normal);

[e1, J1] = p2l_so3(phi, trans, p2, p1, normal);
delta = [0 0 0];
epsilon = 1e-11;
delta(dim) = epsilon;
[e2, J2] = p2l_so3(phi + delta, trans, p2, p1, normal);
disp("numerical diff:")
disp((e2 - e1) / epsilon);
disp("Jacobian:");
disp(J1(dim));

[e3, J3] = p2l_so3(phi, trans + reshape(delta, 3, 1), p2, p1, normal);
disp("trans numerical diff:")
disp((e3 - e1) / epsilon);
disp("normal:");
disp(-normal(dim));

end
