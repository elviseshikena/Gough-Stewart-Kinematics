% Fixed angle Rotation Matrix
% Creates a fixed rotation matrix from the angles given.
% INPUT a_, b_, g_
%     a_ alpha - Angle around z-axis
%     b_ Beta angle around y-axis
%     g_ Gamma Angle arouund x-axis
%
% OUTPUT 3x3 Rotation Matrix

function Rot = rotmat(a_, b_, g_)
ca = cosd(a_);
cb = cosd(b_);
cg = cosd(g_);
sa = sind(a_);
sb = sind(b_);
sg = sind(g_);

Rz = [ca, -sa, 0;
      sa,  ca, 0;
       0,   0, 1];

Ry = [ cb, 0, sb;
        0, 1,  0;
      -sb, 0, cb];

Rx = [1,  0,   0;
      0, cg, -sg;
      0, sg,  cg];

Rot = Rz*Ry*Rx;
end
