function [e, J] = p2l_so3(phi, trans, dst, src, normal)
%P2L_SO3 Summary of this function goes here
%   Detailed explanation goes here
    axang = [reshape(phi, [1,3]) / norm(phi), norm(phi)];
    R = axang2rotm(axang);
    normal = reshape(normal, [1, 3]);
    e = normal * (dst - (R * src + trans));
    J = -cross(R * reshape(src, [3,1]) + trans, normal);
end