function [e, J] = p2p_so3(phi, trans, dst, src)
%P2L_SO3 Summary of this function goes here
%   Detailed explanation goes here
    axang = [reshape(phi, [1,3]) / norm(phi), norm(phi)];
    R = axang2rotm(axang);
    e = dst - (R * src + trans);
    J = skew_sym(R * reshape(src, [3,1]) + trans);
end