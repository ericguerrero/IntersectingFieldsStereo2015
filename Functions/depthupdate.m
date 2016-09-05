function [featureloc]=depthupdate(match1, match2,cam1,cam2,ht1,ht2,dmax,plotray)
f = length(match1);     % Number of matches
featureloc = zeros(f,3);% Feature location

for i=1:f
    % Feature Position
    
    p1 = ray(cam1, ht1, dmax, match1(i,:),'r',plotray); %Feature Position 1
    p2 = ray(cam2, ht2, dmax, match2(i,:),'g',plotray); %Feature Position 2
    t1 = transl(ht1);t2 = transl(ht2);
    res = intersectLines(t1,p1,t2,p2);          %Feature Position (Mid Point)
    featureloc(i,:) = ((res(1:3)+res(4:6))/2)'; %Feature Position Array
    %figure(1);hold on;
    %plot3(res(1:3:4),res(2:3:5),res(3:3:6),'b')
end

%% Plot
K = convhulln(featureloc);
if plotray
    figure(1);hold on;
    plot3(featureloc(:,1),featureloc(:,2),featureloc(:,3),'*b')
    trisurf(K,featureloc(:,1),featureloc(:,2),featureloc(:,3),'FaceColor','b','FaceAlpha',0.2)
end


end