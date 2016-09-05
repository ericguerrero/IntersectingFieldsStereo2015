%   [u v w]' = P * [X Y Z 1]'
%          x = u / w
%          y = v / w

function [V] = vertices3 (cam, tr, dmax)
cam.K=cam.P(1:3,1:3);

LU =transl(tr * transl(inv(cam.K) * [0 0 1]'*dmax));  % point
LD =transl(tr * transl(inv(cam.K) * [cam.w 0 1]'*dmax));
C =transl(tr);
RD =transl(tr * transl(inv(cam.K) * [cam.w cam.h 1]'*dmax));
RU =transl(tr * transl(inv(cam.K) * [0 cam.h 1]'*dmax));


V=[C LU LD RD RU];
end
