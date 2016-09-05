function [I,check,t,u,v]=triangle_line_intersect(P,L) %Doesnt work
% Inputs: 
%       P: Vertices of the triangle 
%       L: Line segment points
%
%Outputs:
%      I    is the point of interection 
%     Check is an indicator:
%      0 => disjoint (no intersection)
%      1 => the plane intersects P0P1 in the unique point I
%      2 => the segment lies in the plane
%      3=>the intersection lies outside the segment P0P1

V0 = P(:,1);
V1 = P(:,2);
V2 = P(:,3);
P0 = L(:,1);
P1 = L(:,2);


p1 = V1-V0;
p2 = V2-V0;

n = cross(p1, p2);          % Normal of the plane
d = P1-P0;                  % Direction of the line
w = P0-V0;

t = 0;
u = 0;
v = 0;


D = dot(n,d);
N = -dot(n,w);

check=0;I=0;
if abs(D) < 10^-7           % The segment is parallel to plane
        if N == 0           % The segment lies in plane
            check=2;
            return
        else
            check=0;        % No intersection
            return
        end
end
    

% Cramer's Rule
D=inv(det([-d p1 p2]));
M=[det([w p1 p2]);det([-d w p2]);det([-d p1 w])];
T=D*M;

t = T(1);
u = T(2);
v = T(3);

if (0>u || 1<u)
    check=0;
    return
end
if (0>v || 1<v)
    check=0;
    return
end
if 1<(v+u)
    check=0;
    return
end
if (0>t || 1<t)
    check=0;
    return
end

I=P0+t*d;
check=1;
end














