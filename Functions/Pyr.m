function [Inter, Points, FlagInter]=Pyr(ht1, ht2, htM, cam1, cam2, dmax, plot3D, plot)


%% Pyramids 

% Vertices
V1 = vertices3(cam1, ht1, dmax);
V2 = vertices3(cam2, ht2, dmax);
K1 = convhulln(V1');
K2 = convhulln(V2');

% Planes and Lines
[P1, L1] = planes_lines(V1);
[P2, L2] = planes_lines(V2);

%% Intersection
FlagInter = false;
Points=[];
% Plane-Line Intersections
for i = 1:6
    for j = 1:8
        [I,check,t,u,v]=triangle_line_intersect(P1(:,:,i),L2(:,:,j));
        if check==1
            Points=[Points; I'];
        end
    end
end
for i = 1:6
    for j = 1:8
        [I,check,t,u,v]=triangle_line_intersect(P2(:,:,i),L1(:,:,j));
        if check==1
            Points=[Points; I'];
        end
    end
end

% Vertex inside the Pyramid
in = inhull(V1',V2',K2);
I = V1(:,find(in));
Points=[Points; I'];
in = inhull(V2',V1',K1);
I = V2(:,find(in));
Points=[Points; I'];

% Intersection Flag
if size(Points,1)>3
    FlagInter = true;
    Inter = convhull(Points(:,1),Points(:,2),Points(:,3));
end


%% Plot
if plot3D==true
    %set(figure(1), 'Position', [1921 31 1366 661])
    figure(1)
    subplot(221);
    % Frames
    axis equal; grid on; hold on
    trplot(htM,'frame', 'Marker','color', 'k','width', 0.5,'text_opts',{'FontSize', 8, 'FontWeight', 'bold'},...
        'length',0.3);
    trplot(ht1,'frame', 'R','color', 'r','width', 0.5,'text_opts',{'FontSize', 8, 'FontWeight', 'bold'},...
        'length',0.3);
    trplot(ht2,'frame', 'H','color', 'g','width', 0.5,'text_opts',{'FontSize', 8, 'FontWeight', 'bold'},...
        'length',0.3);
    % Pyramids
    if plot==true
        alpha=0.30;
        drawPyramid (V1, 'r', alpha);
        drawPyramid (V2 , 'g', alpha);
        scatter3(Points(:,1),Points(:,2),Points(:,3),'k')
        trisurf(Inter,Points(:,1),Points(:,2),Points(:,3),'FaceColor','cyan')    
    end
    %%
    subplot(223);
    % Frames
    axis equal; grid on; hold on
    trplot(htM,'frame', 'Marker','color', 'k','width', 0.5,'text_opts',{'FontSize', 8, 'FontWeight', 'bold'},...
        'length',0.3);
    trplot(ht1,'frame', 'R','color', 'r','width', 0.5,'text_opts',{'FontSize', 8, 'FontWeight', 'bold'},...
        'length',0.3);
    trplot(ht2,'frame', 'H','color', 'g','width', 0.5,'text_opts',{'FontSize', 8, 'FontWeight', 'bold'},...
        'length',0.3);
    % Pyramids
    if plot==true
        trisurf(Inter,Points(:,1),Points(:,2),Points(:,3),'FaceColor','cyan')    
    end
    %%
    subplot(122);
    % Frames
    axis equal; grid on; hold on
    trplot(htM,'frame', 'Marker','color', 'k','width', 0.5,'text_opts',{'FontSize', 8, 'FontWeight', 'bold'},...
        'length',0.3);
    trplot(ht1,'frame', 'R','color', 'r','width', 0.5,'text_opts',{'FontSize', 8, 'FontWeight', 'bold'},...
        'length',0.3);
    trplot(ht2,'frame', 'H','color', 'g','width', 0.5,'text_opts',{'FontSize', 8, 'FontWeight', 'bold'},...
        'length',0.3);
    % Pyramids
    if plot==true
        alpha=0.04;
        trisurf(Inter,Points(:,1),Points(:,2),Points(:,3),'FaceColor','black','FaceAlpha',alpha)    
    end
end

end
