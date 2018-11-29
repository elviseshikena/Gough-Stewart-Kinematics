% Find rotatated vector from one vector to another
% INPUT 3x1 Vectors A and B [x;y;z]
% OUTPUT 3x3 Rotation matrix
function RotationMatrix = findrot(A, B)
A = A/norm(A);
B = B/norm(B);
V = cross(A, B); % Cross product between vectors
% s = norm(V); % sin of angle between vectors
c = dot(A, B); % cos of angles between vectors
I = eye(3); % Identity matrix
% Form skew-symmetric matrix (Transpose equals negative)
Vx = [0 -V(3) V(2) ; V(3) 0 -V(1) ; -V(2) V(1) 0 ];

% Not applicable for when c < 0 i.e A and B are opposite facing
RotationMatrix = I + Vx + Vx^2/(1+c);

end
