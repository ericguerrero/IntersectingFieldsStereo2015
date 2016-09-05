function [] = drawTriangle (P,color,a)

plot3(P(1,:), P(2,:), P(3,:), color);
S = fill3(P(1,:), P(2,:), P(3,:), color);

alpha(S,a)
end
