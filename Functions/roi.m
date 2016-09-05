%   [u v w]' = P * [X Y Z 1]'
%          x = u / w
%          y = v / w

function [roipoints, roiK] = roi(cam, ht, dmax, Points)
K=cam.P(1:3,1:3);

roipoints=zeros(2,length(Points));
for j=1:length(Points)
    p_world = transl(Points(j,:));
    p_camera = transl(inv(ht)*p_world)/dmax;
    M = K*p_camera;
    roipoints(:,j) = M(1:2)/M(3);
end
roiK=convhull(roipoints(1,:),roipoints(2,:));
end
