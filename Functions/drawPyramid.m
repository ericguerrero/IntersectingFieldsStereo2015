function [] = drawPyramid(V,color,alpha)

% color='k'

PlaneL=V(:,1:3);
PlaneR=[V(:,1) V(:,4:5)];
PlaneU=[V(:,1:2) V(:,5)];
PlaneD=[V(:,1) V(:,3:4)];
PlaneB=[V(:,2:5)];

drawTriangle (PlaneL,color,alpha)
drawTriangle (PlaneR,color,alpha)
drawTriangle (PlaneU,color,alpha)
drawTriangle (PlaneD,color,alpha)
drawTriangle (PlaneB,color,alpha)

end
