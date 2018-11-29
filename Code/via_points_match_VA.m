%
% DO NOT MODIFY THIS FILE
%
%VIA_POINT_MATCH_VA Generates Trajectory
%
%  [disp,vel,acc]=via_points_match_VA(points, duration, step, 'type')
%
%  This code will generate a trajectory using cubic polynomials with a path
%  that contains n points and k joints or cartesian coordinates.
%  The velocity and acceleration are matched between swegments.
%
%  INPUT. The file requires an k x n matrix, i.e., where k is the number of 
%  joints or cartesian coordinates and n are all the points (initial, 
%  intermidiate and final), an vector of size n-1 that specifies the 
%  duration of each segment, the increment of time within each segment,
%  e.g., step=0.01s, and the type of motion ('cyclic' or 'prescribed' [V0,
%  Vf]).
%
%  OUTPUT. The file will output three matrices [disp,vel,acc] that will 
%  describe the  trajectory of the joint or cartesian coordinate - 
%  displacement, velocity and acceleration, respectively. 
%

function [position,velocity,acceleration,time]=via_points_match_VA(pts, dt, stp, type, vc)

[kt,n]=size(pts);
m=4*(n-1); a=zeros(m);

%Displacement Constraints
for s=1:n-1 
    i = 2*(s-1) + 1; j = 4*(s-1) + 1;
    a(i,j) = 1; a(i+1,j) = 1; a(i+1,j+1) = dt(s); a(i+1,j+2) = dt(s)^2; a(i+1,j+3) = dt(s)^3;
end

%Velocity and Acceleration Constraints
for s=1:n-2
    i = 2*(s-1) + m/2 + 1; j = 4*(s-1) + 1; 
    a(i,j+1) = 1; a(i,j+2) = 2*dt(s); a(i,j+3) = 3*dt(s)^2; a(i,j+5) = -1; 
    a(i+1,j+2) = 2; a(i+1,j+3) = 6*dt(s); a(i+1,j+6) = -2; 
end

%Initial and Final Posture Velocity (and Acceleration) Conditions 
if strcmp(type,'cyclic') 
     a(m-1,m-2) = 1; a(m-1,m-1) = 2*dt(n-1); a(m-1,m) = 3*dt(n-1)^2; a(m-1,2) = -1; 
     a(m-1,m-1) = 2; a(m-1,m) = 6*dt(n-1); a(m-1,3) = -2;      
elseif strcmp(type,'prescribed') 
     a(m-1,2) = 1; 
     a(m,m-2) = 1; a(m,m-1) = 2*dt(n-1); a(m,m) = 3*dt(n-1)^2;   
end

position = []; velocity = []; acceleration = []; time = []; 
bk=zeros(m,1);
for k=1:kt 
    bk(1)=pts(k,1); bk(m/2)=pts(k,n); 
    if strcmp(type,'prescribed'); bk(m-1)=vc(1); bk(m)=vc(2); end  
    for s=1:n-2
        i = 2*(s-1) + 2;
        bk(i)=pts(k,s+1);  bk(i+1)=pts(k,s+1);
    end
    ak = inv(a)*bk;
    
    disp=[]; vel=[]; acc=[]; tim=[]; toff=0;
    for s=1:n-1
        t=0:stp:dt(s);
        i = 4*(s-1) + 1;
        ds = ak(i) + ak(i+1).*t + ak(i+2).*t.^2 + ak(i+3).*t.^3;
        vs = ak(i+1) + 2.*ak(i+2).*t + 3.*ak(i+3).*t.^2;
        as = 2.*ak(i+2) + 6.*ak(i+3).*t;

        disp=[disp ds]; vel=[vel vs]; acc=[acc as]; tim=[tim t+toff]; toff=toff+dt(s);
    end
     position = [position; disp]; velocity = [velocity; vel]; acceleration = [acceleration; acc];      time=[time; tim];
end

return