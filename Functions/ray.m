function [p] = ray (cam, tr, dim, m, color, plot)
cam.K=cam.P(1:3,1:3);

p_camera = inv(cam.K) * [m, 1]';
p_world = tr * transl(p_camera*dim);
p =transl(p_world);  % point

if plot
    plot3([tr(1,4) ; p(1)],[tr(2,4) ; p(2)],[tr(3,4) ; p(3)],color)
end
end
